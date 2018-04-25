## Start ROS Master
    roscore

## Start Youbot Driver
    roslaunch youbot_driver_ros_interface youbot_driver.launch

## Command for Object Recognition Position
    rostopic pub /arm_1/arm_controller/position_command brics_actuator/JointPositions '{positions:[ 
    {joint_uri: arm_joint_1, unit: rad, value: 2.95}, 
    {joint_uri: arm_joint_2, unit: rad, value: 1.8}, 
    {joint_uri: arm_joint_3, unit: rad, value: -1.7}, 
    {joint_uri: arm_joint_4, unit: rad, value: 3.42}, 
    {joint_uri: arm_joint_5, unit: rad, value: 2.95}]}'
