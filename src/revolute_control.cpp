/* Controlling revolute joint by publishing to topic (model_name/vel_cmd) right now 1DOF/vel_cmd (TODO change it) subscribed by revolute_control_plugin*/
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

#include <sstream>

int main(int argc, char **argv)
{
  //Initialize ROS. This is where we specify the name of our node
  ros::init(argc, argv, "revolute_control");

  //node handler
  ros::NodeHandle n;

  //publisher to <model_name>/vel_cmd to control the model's revolute joint
  ros::Publisher joint_vel_pub = n.advertise<std_msgs::Float32>("/1DOF/vel_cmd", 1000);  //TODO change the topic's name and match it with the model we desire to control

  // message for velocity command
  std_msgs::Float32 joint_vel_msg;
  
  ros::Rate loop_rate(10);  //to control the rate of the loop
  while (ros::ok())
  {
    
    
    //fill the message
    joint_vel_msg.data=1.0;
    
    //publish the joint velocity command
    joint_vel_pub.publish(joint_vel_msg);

    ros::spinOnce();

    loop_rate.sleep();

  }


  return 0;
}
