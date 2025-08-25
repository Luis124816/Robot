Hello there.

My solution needs the simple commander installed please:
sudo apt install ros-humble-navigation2 ros2-humble-nav2-bringup
sudo apt install ros-humble-nav2-simple-commander

Note - During some simulations the provided services /pick_up_item and / can take a long time to respond (1+ minute), other times less than 5s. The implementation of the item manager was left unchanged. - I dont understand why this is occuring - something to do with potential race conditions of the item manager? - delays seem to be exacerbated when simulation is sped up. 

To run the solution:
1.In a new terminal of this folder, colcon build
2.In another new terminal at this folder type: source ./install/setup.bash followed by ros2 launch solution solution_nav2_launch.py


To modify parameters for different simulation scenairoes:
1. Missing 1 zone, change line 260 of assessment_launch.py: default_value='true' set to 'false'
2. Missing 2 zones, change line 260 of assessment_launch.py: default_value='true' set to 'false' and change line 254 of  assessment_launch.py : default_value='true' to 'false'



