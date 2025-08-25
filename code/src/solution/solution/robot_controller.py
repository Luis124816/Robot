import sys

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException

# import by me from the week_5 robot_controller
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSPresetProfiles
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, Pose, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from auro_interfaces.msg import StringWithPose
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from assessment_interfaces.msg import Zone, ZoneList, ItemList, Item, ItemHolders, ItemHolder
from auro_interfaces.srv import ItemRequest
from .waypoints import create_zone_search_waypoints, create_item_search_waypoints

import angles
from enum import Enum
import random
import math



 # Array indexes for sensor sectors
SCAN_THRESHOLD = 0.5 
SCAN_FRONT = 0
SCAN_LEFT = 1
SCAN_BACK = 2
SCAN_RIGHT = 3


# Define the states and substates of the composite state machine
class State(Enum):
    INITIALISING = 0
    SEARCHING = 1
    COLLECTING = 2
    DELIVERING = 3
    CEASING = 4

class INITIALISING_SUBSTATE(Enum):
    SETTING_GOAL = 0
    NAVIGATING = 1
    CHECKING_ZONE = 2
    ASSIGNING_ZONES = 3

class SEARCHING_SUBSTATE(Enum):
    SETTING_GOAL = 0
    NAVIGATING = 2
    WAITING_FOR_NAV2_STOP = 3
    SPINNING = 4

class COLLECTING_SUBSTATE(Enum):
    APPROACHING = 0
    REQUESTING_PICKUP = 1
    WAITING_FOR_PICKUP = 2

class DELIVERING_SUBSTATE(Enum):
    NAVIGATING = 0
    REQUESTING_OFFLOAD = 1
    WAITING_FOR_OFFLOAD = 2

class ACTION(Enum):
    ENTRY = 0
    INTERRUPTABLE = 1
    EXIT = 2




class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')

        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)

        # Create callback groups 
        s_client_callback_group = MutuallyExclusiveCallbackGroup()
        s_timer_callback_group = MutuallyExclusiveCallbackGroup()
        s_sensor_callback_group = MutuallyExclusiveCallbackGroup()  # Separate group for sensors

        self.initial_x = self.get_parameter('x').get_parameter_value().double_value
        self.initial_y = self.get_parameter('y').get_parameter_value().double_value
        self.initial_yaw = self.get_parameter('yaw').get_parameter_value().double_value

        self.declare_parameter('robot_id', 'robot1')
        self.robot_id = self.get_parameter('robot_id').value

        # import waypoint sets
        self.zone_searching_waypoints = create_zone_search_waypoints()
        self.item_searching_waypoints = create_item_search_waypoints()

        # Initialize the navigator
        self.navigator = BasicNavigator()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.initial_x
        pose.pose.position.y = self.initial_y
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0
        self.navigator.setInitialPose(pose)
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Initial pose set.')

        # Create the timer for the control loop and set the initial states
        self.timer_period = 0.1 # 100 milliseconds = 10 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop, callback_group=s_timer_callback_group)
        self.state = State.INITIALISING
        self.ACTION = ACTION.ENTRY
        self.clock_counter = 0
        

        # Create the publisher for the cmd_vel
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.yaw = 0.0  # Initialize yaw for cmd_vel rotation operations
        
        # Create the array for the scan triggered
        self.scan_triggered = [False, False, False, False]  # [front, left, back, right]
        
        # Create the service clients for the item pickup and offload
        self.pick_up_service = self.create_client(ItemRequest, '/pick_up_item', callback_group=s_client_callback_group)
        self.offload_service = self.create_client(ItemRequest, '/offload_item', callback_group=s_client_callback_group)
        self.pickup_future = None  # Store the future for async service calls
        self.offload_future = None 
        
        # setup for the zones
        self.zone_searching_waypoint_index = 0
        self.current_zones = []
        self.discovered_zones = ['empty','empty','empty','empty']
        self.zone_mapping = None
        self.Initialising_complete = False
        self.zone_list_subscriber = self.create_subscription(ZoneList, 'zone', self.zone_list_callback, 10, callback_group=s_sensor_callback_group)
        

        # Prep Item callbacks
        self.items = ItemList()
        self.item_subscriber = self.create_subscription(ItemList,'items',self.item_callback,10, callback_group=s_sensor_callback_group)

        # Prep Item holders callbacks
        self.item_holders = ItemHolders()
        self.holding_item = None
        self.holding_item_colour = None
        self.item_holders_subscriber = self.create_subscription(
            ItemHolders,
            '/item_holders',
            self.item_holders_callback,
            10, callback_group=s_sensor_callback_group
        )

        # Prep Odometry callbacks
        self.odom_subscriber = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10, callback_group=s_sensor_callback_group
        )
        
        # Prep Laser scan callbacks
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10, callback_group=s_sensor_callback_group
        )
    
    def item_callback(self, msg):
        if len(msg.data) > 0:
            # Find the item with the largest diameter among target color items
            nearest_item = max(msg.data, key=lambda item: item.diameter)
            self.items = ItemList()
            self.items.data = [nearest_item]
        else:
            self.items = msg
    
    def odom_callback(self, msg):
        # Convert to yaw
        orientation = msg.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z))
            
    
    def zone_list_callback(self, msg):
        self.current_zones = msg.data
    
    
    def scan_callback(self, msg):
        # Group scan ranges into 4 segments
        # Front, left, and right segments are each 60 degrees
        # Back segment is 180 degrees
        front_ranges = msg.ranges[331:359] + msg.ranges[0:30] # 30 to 331 degrees (30 to -30 degrees)
        left_ranges  = msg.ranges[31:90] # 31 to 90 degrees (31 to 90 degrees)
        back_ranges  = msg.ranges[91:270] # 91 to 270 degrees (91 to -90 degrees)
        right_ranges = msg.ranges[271:330] # 271 to 330 degrees (-30 to -91 degrees)

        # Store True/False values for each sensor segment, based on whether the nearest detected obstacle is closer than SCAN_THRESHOLD
        self.scan_triggered[SCAN_FRONT] = min(front_ranges) < SCAN_THRESHOLD 
        self.scan_triggered[SCAN_LEFT]  = min(left_ranges)  < SCAN_THRESHOLD
        self.scan_triggered[SCAN_BACK]  = min(back_ranges)  < SCAN_THRESHOLD
        self.scan_triggered[SCAN_RIGHT] = min(right_ranges) < SCAN_THRESHOLD

    def item_holders_callback(self, msg):
        self.item_holders = msg
        self.holding_item = msg.data[0].holding_item
        self.holding_item_colour = msg.data[0].item_colour













    def control_loop(self):
       
        
        match self.state:
            
            case State.INITIALISING:
                match self.ACTION:
                    
                    # Wait for the items, world and robot to be ready
                    case ACTION.ENTRY:
                        self.clock_counter += 1
                        if self.clock_counter >= 100:
                            self.get_logger().info('WORLD IS READY')
                            self.ACTION = ACTION.INTERRUPTABLE
                            self.substate = INITIALISING_SUBSTATE.SETTING_GOAL
                            self.clock_counter = 0
                            


                    case ACTION.INTERRUPTABLE:
                        if self.robot_id == 'robot1':
                            match self.substate:
                                case INITIALISING_SUBSTATE.SETTING_GOAL:
                                    if self.zone_searching_waypoint_index < len(self.zone_searching_waypoints):
                                        self.zone_searching_waypoints[self.zone_searching_waypoint_index].header.stamp = self.get_clock().now().to_msg()
                                        self.navigator.goToPose(self.zone_searching_waypoints[self.zone_searching_waypoint_index])
                                        self.substate = INITIALISING_SUBSTATE.NAVIGATING
                                        self.get_logger().info(f'Navigating to waypoint {self.zone_searching_waypoint_index}')
                                    else:
                                        self.substate = INITIALISING_SUBSTATE.ASSIGNING_ZONES
                                        self.get_logger().info('All waypoints visited!')
                                    
                                case INITIALISING_SUBSTATE.NAVIGATING:
                                    if not self.navigator.isTaskComplete():
                                        feedback = self.navigator.getFeedback()
                                        if feedback and feedback.estimated_time_remaining:
                                            secs = feedback.estimated_time_remaining.sec
                                            self.get_logger().info(f'ETA to goal: {secs} sec')
                                    else:
                                        result = self.navigator.getResult()
                                        if result == TaskResult.SUCCEEDED:
                                            self.get_logger().info('Goal succeeded!')
                                            self.substate = INITIALISING_SUBSTATE.CHECKING_ZONE
                                        elif result == TaskResult.FAILED:
                                            self.get_logger().error('Goal failed!')
                                        elif result == TaskResult.CANCELED:
                                            self.get_logger().warn('Goal was canceled!')
                                        else:
                                            self.get_logger().warn(f'Unknown result: {result}')
                                        

                                case INITIALISING_SUBSTATE.CHECKING_ZONE:
                                    self.get_logger().info(f'INITIALISING: CHECKING ZONE')
                                    self.clock_counter += 1
                                    if self.clock_counter >= 4:
                                        self.get_logger().info(f'Im looking at zone: {self.current_zones}')
                                        if self.current_zones != []:
                                            self.discovered_zones[self.zone_searching_waypoint_index] = self.current_zones[0].zone
                                        self.clock_counter = 0
                                        self.zone_searching_waypoint_index += 1
                                        self.substate = INITIALISING_SUBSTATE.SETTING_GOAL

                                case INITIALISING_SUBSTATE.ASSIGNING_ZONES:
                                    self.get_logger().info(f'INITIALISING: ASSIGNING ZONES')
                                    # Filter out empty zones
                                    discovered_zones = [z for z in self.discovered_zones if z != 'empty']

                                    if len(discovered_zones) < 3:
                                        self.get_logger().info(f'Not enough zones! ')
                                        self.state = State.CEASING
                                        return

                                    # Define colors for assignment
                                    colors = ['RED', 'GREEN', 'BLUE']
                                    
                                    self.assigned_zones = random.sample(discovered_zones, 3)
                                    self.zone_mapping = dict(zip(colors, self.assigned_zones))

                                    # Define the mapping from zone ID to waypoint
                                    zone_id_to_waypoint = {
                                        1: self.zone_searching_waypoints[0],
                                        2: self.zone_searching_waypoints[1],
                                        3: self.zone_searching_waypoints[2],
                                        4: self.zone_searching_waypoints[3]
                                    }

                                    # Replace zone IDs with waypoints
                                    for color, zone_id in self.zone_mapping.items():
                                        self.zone_mapping[color] = zone_id_to_waypoint[zone_id]
                                    
                                    self.get_logger().info(f'The zone mapping is: {self.zone_mapping}')
                                    self.state = State.SEARCHING
                                    self.substate = SEARCHING_SUBSTATE.SETTING_GOAL
                        else:
                            if self.Initialising_complete == True:
                                # do this once one robot is working
                                self.state = State.CEASING
                                
                
                
                
                
            
            


            case State.SEARCHING:
                match self.substate:
                    case SEARCHING_SUBSTATE.SETTING_GOAL:
                        self.get_logger().info(f'SEARCHING: SETTING GOAL')
                        # Choose a random waypoint
                        i = random.randint(0, len(self.item_searching_waypoints) - 1)
                        self.item_searching_waypoints[i].header.stamp = self.get_clock().now().to_msg()
                        self.navigator.goToPose(self.item_searching_waypoints[i])
                        self.substate = SEARCHING_SUBSTATE.NAVIGATING
                        self.get_logger().info(f'Navigating to waypoint {i}')

                    case SEARCHING_SUBSTATE.NAVIGATING:
                        self.get_logger().info(f'items in the self.items.data: {self.items.data}')
                        self.get_logger().info(f'Type of self.items: {type(self.items)}')
                        self.get_logger().info(f'Type of self.items.data: {type(self.items.data)}')
                        self.get_logger().info(f'Length of self.items.data: {len(self.items.data)}')
                        # Always check for items while navigating
                        if len(self.items.data) > 0:
                            self.get_logger().info(f'Item detected! Canceling navigation to collect.')
                            self.navigator.cancelTask()
                            self.substate = SEARCHING_SUBSTATE.WAITING_FOR_NAV2_STOP
                            self.clock_counter = 0
                            return
                        
                        
                        # Check navigation status
                        if not self.navigator.isTaskComplete():
                            feedback = self.navigator.getFeedback()
                            if feedback and feedback.estimated_time_remaining:
                                secs = feedback.estimated_time_remaining.sec
                                self.get_logger().info(f'ETA to goal: {secs} sec')
                        else:
                            result = self.navigator.getResult()
                            if result == TaskResult.SUCCEEDED:
                                self.get_logger().info('Waypoint reached! Setting next goal.')
                                self.substate = SEARCHING_SUBSTATE.SPINNING
                            elif result == TaskResult.FAILED:
                                self.get_logger().error('Waypoint failed! Trying next waypoint.')
                                self.substate = SEARCHING_SUBSTATE.SETTING_GOAL
                            elif result == TaskResult.CANCELED:
                                self.get_logger().warn('Navigation was canceled!')
                                self.substate = SEARCHING_SUBSTATE.WAITING_FOR_NAV2_STOP
                                self.clock_counter = 0
                            else:
                                self.get_logger().warn(f'Unknown navigation result: {result}')
                                self.substate = SEARCHING_SUBSTATE.SETTING_GOAL


                    case SEARCHING_SUBSTATE.SPINNING:
                        if not hasattr(self, 'spin_start_yaw'):
                            self.spin_start_yaw = self.yaw
                            self.get_logger().info(f'Starting 360-degree spin from yaw: {self.yaw}')
                        
                        # Calculate how much we've rotated
                        yaw_difference = abs(self.yaw - self.spin_start_yaw)
                        if yaw_difference > math.pi:
                            yaw_difference = 2 * math.pi - yaw_difference
                        
                        degrees_rotated = math.degrees(yaw_difference)
                        self.get_logger().info(f'Spinning - rotated {degrees_rotated:.1f} degrees')
                        
                        # Check for items while spinning
                        if len(self.items.data) > 0:
                            self.get_logger().info(f'Item detected during spin! Stopping spin and approaching.')
                            cmd_vel = Twist()
                            cmd_vel.linear.x = 0.0
                            cmd_vel.angular.z = 0.0
                            self.cmd_vel_publisher.publish(cmd_vel)
                            # Transition directly to COLLECTING state
                            self.state = State.COLLECTING
                            self.ACTION = ACTION.ENTRY
                            self.spin_start_yaw = None
                            return
                        
                        # Continue spinning
                        cmd_vel = Twist()
                        cmd_vel.linear.x = 0.0
                        cmd_vel.angular.z = 0.5  # Radians per second
                        self.cmd_vel_publisher.publish(cmd_vel)
                        
                        # Check if we've completed 360 degrees (2π radians)
                        if yaw_difference >= 2 * math.pi - 0.1:  # Allow small tolerance
                            self.get_logger().info('360-degree spin completed!')
                            cmd_vel = Twist()
                            cmd_vel.linear.x = 0.0
                            cmd_vel.angular.z = 0.0
                            self.cmd_vel_publisher.publish(cmd_vel)
                            self.substate = SEARCHING_SUBSTATE.SETTING_GOAL
                            self.spin_start_yaw = None
                        


                    case SEARCHING_SUBSTATE.WAITING_FOR_NAV2_STOP:
                        self.get_logger().info(f'SEARCHING: WAITING FOR NAV2 STOP {self.clock_counter}')
                        self.clock_counter += 1
                        
                        # Check if navigation is actually stopped
                        if not self.navigator.isTaskComplete():
                            # Still canceling, wait a bit more
                            if self.clock_counter >= 50:  # 5 seconds timeout
                                self.get_logger().warn('Navigation cancel timeout, forcing transition')
                                self.state = State.COLLECTING
                                self.ACTION = ACTION.ENTRY
                                self.clock_counter = 0
                        else:
                            # Navigation is complete (canceled), safe to transition
                            result = self.navigator.getResult()
                            if result == TaskResult.CANCELED:
                                self.get_logger().info('Navigation successfully canceled, transitioning to COLLECTING')
                                self.state = State.COLLECTING
                                self.ACTION = ACTION.ENTRY
                                self.clock_counter = 0
                            else:
                                self.get_logger().warn(f'Unexpected navigation result: {result}')
                                self.state = State.COLLECTING
                                self.ACTION = ACTION.ENTRY
                                self.clock_counter = 0

            
            
            
            
            
            





            
            case State.COLLECTING:
                match self.ACTION:
                    case ACTION.ENTRY:
                        self.ACTION = ACTION.INTERRUPTABLE
                        self.substate = COLLECTING_SUBSTATE.APPROACHING
                    
                    case ACTION.INTERRUPTABLE:
                        match self.substate:

                            case COLLECTING_SUBSTATE.APPROACHING:
                                item = self.items.data[0]

                                estimated_distance = 32.4 * float(item.diameter) ** -0.75 #69.0 * float(item.diameter) ** -0.89
                                if self.scan_triggered[SCAN_FRONT] == True or self.scan_triggered[SCAN_LEFT] == True or self.scan_triggered[SCAN_RIGHT] == True:
                                    self.get_logger().info('Obstacle detected, stopping')
                                    msg = Twist()
                                    msg.linear.x = 0.0
                                    msg.angular.z = 0.0
                                    self.cmd_vel_publisher.publish(msg)
                                    self.state = State.SEARCHING
                                    self.ACTION = ACTION.ENTRY
                                    self.substate = SEARCHING_SUBSTATE.SETTING_GOAL
                                    return

                                self.get_logger().info(f'Estimated distance {estimated_distance}')
                                if estimated_distance <= 0.35:
                                        msg = Twist()
                                        msg.linear.x = 0.0
                                        msg.angular.z = 0.0
                                        self.cmd_vel_publisher.publish(msg)
                                        self.substate = COLLECTING_SUBSTATE.REQUESTING_PICKUP
                                        return  # Exit and wait for next control loop iteration
                                else:
                                    self.get_logger().info('Moving towards item')
                                    msg = Twist()
                                    msg.linear.x = 0.25 * estimated_distance
                                    msg.angular.z = item.x / 320.0
                                    self.cmd_vel_publisher.publish(msg)

                            
                            case COLLECTING_SUBSTATE.REQUESTING_PICKUP:
                                if self.pickup_future is None:
                                    self.get_logger().info('Requesting Pick up...')
                                    rqt = ItemRequest.Request()
                                    rqt.robot_id = self.robot_id
                                    self.pickup_future = self.pick_up_service.call_async(rqt)
                                    self.pickup_start_time = self.get_clock().now()
                                    self.get_logger().info('Pickup request sent, waiting for response...')
                                    self.substate = COLLECTING_SUBSTATE.WAITING_FOR_PICKUP

                            case COLLECTING_SUBSTATE.WAITING_FOR_PICKUP:
                                elapsed_time = (self.get_clock().now() - self.pickup_start_time).nanoseconds / 1e9
                                if elapsed_time % 10 == 0:
                                    self.get_logger().info(f'Waiting for pickup - elapsed time: {elapsed_time:.1f}s')
                                

                                if self.pickup_future.done():
                                    try:
                                        response = self.pickup_future.result()
                                        self.get_logger().info(f'Pickup response received after {elapsed_time:.1f}s')
                                        if response.success:
                                            self.get_logger().info('Item picked up successfully!')
                                            self.ACTION = ACTION.EXIT
                                            self.pickup_future = None
                                            self.pickup_start_time = None
                                        else:
                                            self.get_logger().info('Unable to pickup item: ' + response.message)
                                            self.state = State.SEARCHING
                                            self.ACTION = ACTION.ENTRY
                                            self.pickup_future = None
                                            self.pickup_start_time = None
                                    except Exception as e:
                                        self.get_logger().error(f'Exception during pickup: {e}')
                                        self.pickup_future = None
                                        self.pickup_start_time = None
                                        self.state = State.SEARCHING
                                        self.ACTION = ACTION.ENTRY
                                

                    case ACTION.EXIT:
                        self.get_logger().info('Pickup complete!')
                        self.state = State.DELIVERING
                        self.ACTION = ACTION.ENTRY
                                
















                           
            case State.DELIVERING:
                match self.ACTION:
                    case ACTION.ENTRY:
                        # Set the navigation goal to the appropriate zone
                        if self.holding_item:
                            colour = self.holding_item_colour
                            goal = self.zone_mapping[colour]
                            self.navigator.goToPose(goal)
                            self.get_logger().info(f'Navigating to delivery goal {goal}')
                            self.substate = DELIVERING_SUBSTATE.NAVIGATING
                            self.ACTION = ACTION.INTERRUPTABLE

                    case ACTION.INTERRUPTABLE:
                        match self.substate:
                            case DELIVERING_SUBSTATE.NAVIGATING:
                                if not self.navigator.isTaskComplete():
                                    feedback = self.navigator.getFeedback()
                                    if feedback and feedback.estimated_time_remaining:
                                        secs = feedback.estimated_time_remaining.sec
                                        self.get_logger().info(f'ETA to delivery goal: {secs} sec')
                                else:
                                    result = self.navigator.getResult()
                                    if result == TaskResult.SUCCEEDED:
                                        self.get_logger().info('Delivery goal reached!')
                                        self.substate = DELIVERING_SUBSTATE.REQUESTING_OFFLOAD
                                    elif result == TaskResult.FAILED:
                                        self.get_logger().error('Delivery goal failed!')
                                        self.state = State.DELIVERING
                                        self.ACTION = ACTION.ENTRY
                                    elif result == TaskResult.CANCELED:
                                        self.get_logger().warn('Delivery goal was canceled!')
                                        self.state = State.DELIVERING
                                        self.ACTION = ACTION.ENTRY
                                    else:
                                        self.get_logger().warn(f'Unknown navigation result: {result}')
                                        self.state = State.DELIVERING
                                        self.ACTION = ACTION.ENTRY
                            # change this implementation of the request offload
                            case DELIVERING_SUBSTATE.REQUESTING_OFFLOAD:
                                if self.offload_future is None:
                                    self.get_logger().info('Requesting offload')
                                    rqt = ItemRequest.Request()
                                    rqt.robot_id = self.robot_id
                                    self.offload_future = self.offload_service.call_async(rqt)
                                    self.offload_start_time = self.get_clock().now()
                                    self.substate = DELIVERING_SUBSTATE.WAITING_FOR_OFFLOAD

                            case DELIVERING_SUBSTATE.WAITING_FOR_OFFLOAD:
                                
                                
                                elapsed_time = (self.get_clock().now() - self.offload_start_time).nanoseconds / 1e9
                                self.get_logger().info(f'Waiting for offload - elapsed time: {elapsed_time:.1f}s')
                                

                                if self.offload_future.done():
                                    try:
                                        response = self.offload_future.result()
                                        self.get_logger().info(f'Offload response received after {elapsed_time:.1f}s')
                                        if response.success:
                                            self.get_logger().info('Item offloaded successfully!')
                                            self.ACTION = ACTION.EXIT
                                            self.offload_future = None
                                            self.offload_start_time = None
                                        else:
                                            self.get_logger().info('Unable to offload item: ' + response.message)
                                            self.state = State.SEARCHING
                                            self.ACTION = ACTION.ENTRY
                                            self.offload_future = None
                                            self.offload_start_time = None
                                    except Exception as e:
                                        self.get_logger().error(f'Exception during offload: {e}')
                                        self.offload_future = None
                                        self.offload_start_time = None
                                        self.state = State.SEARCHING
                                        self.ACTION = ACTION.ENTRY
                                

                    case ACTION.EXIT:
                        self.get_logger().info('Delivery complete, returning to searching')
                        self.state = State.SEARCHING
                        self.ACTION = ACTION.ENTRY
                        self.substate = SEARCHING_SUBSTATE.SETTING_GOAL
                
                
            case State.CEASING:
                self.get_logger().info('Ceasing')
                



























    def destroy_node(self):
        msg = Twist()
        self.cmd_vel_publisher.publish(msg)
        self.get_logger().info(f"Stopping: {msg}")
        super().destroy_node()


def main(args=None):

    rclpy.init(args = args, signal_handler_options = SignalHandlerOptions.NO)

    node = RobotController()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()



if __name__ == '__main__':
    main()