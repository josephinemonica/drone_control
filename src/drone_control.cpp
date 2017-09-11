#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <sstream>

//save the current state of the autopilot. This will allow us to check connection, arming and Offboard flags.
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

/********************************************************************************************************************/
int main(int argc, char **argv)
{
  //Initialize drone_control node
  ros::init(argc, argv, "drone_control");

  //node handle
  ros::NodeHandle n;

  //publisher
  ros::Publisher location_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 2);
  ros::Publisher velocity_pub = n.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel",2);
  
  //subscriber
  ros::Subscriber state_sub = n.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);  //get the current state
  
  //client
  ros::ServiceClient arming_client = n.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");   //to request arming
  ros::ServiceClient set_mode_client = n.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");       //to request mode change
  
  //message
  geometry_msgs::PoseStamped location_msg;
  geometry_msgs::TwistStamped velocity_msg;
  
  mavros_msgs::SetMode offb_set_mode;   //to request offboard mode
  offb_set_mode.request.custom_mode = "OFFBOARD";
  
  mavros_msgs::CommandBool arm_cmd;     //to request arming
  arm_cmd.request.value = true;
  
  /*
    The px4 flight stack has a timeout of 500ms between two Offboard commands. If this timeout is exceeded, the commander will  fall back to the last mode the vehicle was in before entering Offboard mode.
*/
  //run the loop at 10 Hz
  ros::Rate loop_rate(10);
  

  ros::Time last_request = ros::Time::now();
  
  while (ros::ok())
  {

    if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
    {
      if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.success)
      {
        ROS_INFO("Offboard enabled");
      }
      last_request = ros::Time::now();
    } 
    else 
    {
      if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
      {
        if( arming_client.call(arm_cmd) && arm_cmd.response.success)
        {
          ROS_INFO("Vehicle armed");
        }
       last_request = ros::Time::now();
      }
    }
    
    
    location_msg.pose.position.x=10;
    location_msg.pose.position.y=1;
    location_msg.pose.position.z=10;
    location_msg.pose.orientation.x=0;
    location_msg.pose.orientation.y=0;
    location_msg.pose.orientation.z=0;
    location_msg.pose.orientation.w=0;
    
    location_pub.publish(location_msg);

/*
    velocity_msg.twist.linear.x=1;
    velocity_msg.twist.linear.y=0;
    velocity_msg.twist.linear.z=1;
    velocity_msg.twist.angular.x=0;
    velocity_msg.twist.angular.y=0;
    velocity_msg.twist.angular.z=0;
    
    velocity_pub.publish(velocity_msg);
*/    
    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}

/********************************************************************************************************************/
