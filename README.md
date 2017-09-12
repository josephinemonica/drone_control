SRC:
drone_control.cpp = drone_control node to control drone(PX4 simulation mavros_posix) by specifying velocity or location
revolute_control.cpp = revolute_control node to control revolute joint, publishing joint velocity command. Currently the topic is set to control 1DOF model

LAUNCH:
demo.launch = launches the PX4 simulation & drone_control node

PLUGINS:
revolute_control_plugin = Plugin for controlling a revolute joint (assuming only 1 joint)
It subscribes to ROS topic of <model_name>/vel_cmd and uses the message (std_msgs/Float32) to set the joint velocity

MODELS:
1DOF = sdf model to demonstrate a revolute joint (The model is based on the velodyne model in gazebo tutorial)

WORLDS:
1DOF.world = includes 1DOF model and attaches it to revolute_control_plugin to demonstrate controlling joint through ROS topic
