"""
Waypoint definitions for robot navigation.
Simple coordinate storage for zone searching and other navigation tasks.
"""

from geometry_msgs.msg import PoseStamped

def create_zone_search_waypoints():
    waypoints = []
    waypoint_coords = [
        
        # x, y, qz, qw
        [-3.4, 2.4, 0.856, 0.517],
        [2.4, 2.4, 0.254, 0.967],
        [2.4, -2.4, -0.265, 0.964],
        [-3.4, -2.4, -0.99, 0.17]
    ]

    for x, y, qz, qw in waypoint_coords:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        waypoints.append(pose)
    
    return waypoints


def create_item_search_waypoints():
    waypoints = []
        
        # x, y
    waypoint_coords = [
        [-2.0, 2.0],
        [0.0, 2.0],
        [2.0, 2.0],
        [2.0, 0.0],
        [0.0, 0.0],
        [-2.0, 0.0],
        [-2.0, -2.0],
        [0.0, -2.0],
        [2.0, -2.0]
    ]


    for x, y in waypoint_coords:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0
        waypoints.append(pose)
    
    return waypoints
