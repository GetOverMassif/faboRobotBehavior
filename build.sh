rm -rf build devel
catkin_make arm_control_generate_messages
catkin_make rm_msgs_generate_messages
catkin_make arm_control
catkin_make BehaviorModule_generate_messages
catkin_make PerformModule_generate_messages
catkin_make