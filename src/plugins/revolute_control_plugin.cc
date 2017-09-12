#ifndef _REVOLUTE_CONTROL_PLUGIN_HH_
#define _REVOLUTE_CONTROL_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

//for GAZEBO transport for setting joint velocity API purpose
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

//for ROS transport for setting joint velocity API purpose
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
namespace gazebo
{
  //A plugin to control a revolute joint
  class revolute_control_plugin : public ModelPlugin
  {
    //Constructor
    public: revolute_control_plugin() {}

    //The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    // _model A pointer to the model that this plugin is attached to.
    // _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Safety check
      if (_model->GetJointCount() == 0)
      {
        std::cerr << "Invalid joint count, 0 joint\n";
        return;
      }
      else
      {
        std::cout<<_model->GetJointCount()<<" joints detected\n";
      }
      
      // Store the model pointer for convenience.
      this->model = _model;
      // Get the first joint(REV joint)
      this->joint = _model->GetJoints()[0];
      // PID controller: Setup a P-controller, with a gain of 0.1.
      this->pid = common::PID(0.1, 0, 0);
      // Apply the PID-controller to the joint.
      this->model->GetJointController()->SetVelocityPID(this->joint->GetScopedName(), this->pid);
      
      /*----------------------GAZEBO setting velocity joint API------------*/
      // Create the gazebo node
      this->node = transport::NodePtr(new transport::Node());
      #if GAZEBO_MAJOR_VERSION < 8
      this->node->Init(this->model->GetWorld()->GetName());
      #else
      this->node->Init(this->model->GetWorld()->Name());
      #endif
      
      // Create a gazebo topic of vel_cmd
      std::string topicName = "~/" + this->model->GetName() + "/vel_cmd";
      
      // Subscribe to the topic, and register a callback
      this->sub = this->node->Subscribe(topicName,
   &revolute_control_plugin::OnMsg, this);

      /*-----------------------------------------------------------*/
      
      /*----------------------ROS setting velocity joint API------------*/
      // Initialize ros, if it has not already bee initialized.
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client",
        ros::init_options::NoSigintHandler);
      }

      // Create ROS node. This acts in a similar manner to the Gazebo node
      this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

      // Create a ROS topic for velocity command, and subscribe to it.
      ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float32>("/" + this->model->GetName() + "/vel_cmd",1,
      boost::bind(&revolute_control_plugin::OnRosMsg, this, _1),
      ros::VoidPtr(), &this->rosQueue);
      this->rosSub = this->rosNode->subscribe(so);

      // Spin up the queue helper thread.
      this->rosQueueThread = std::thread(std::bind(&revolute_control_plugin::QueueThread, this));
      /*-----------------------------------------------------------*/
      
      std::cerr << "\nThe revolute_control_plugin is attach to model[" <<
        _model->GetName() << "]\n";
        
        
    }//------------------------------------------------------------------------end of load function
    
    //Pointer to the model.
    private: physics::ModelPtr model;
    //Pointer to the joint.
    private: physics::JointPtr joint;
    //A PID controller for the joint.
    private: common::PID pid;
    
    /*----------------------GAZEBO setting velocity joint API------------*/
    //A node used for transport
    private: transport::NodePtr node;
    //A subscriber to a named topic.
    private: transport::SubscriberPtr sub;
    
    //Handle incoming message. Repurpose a vector3 message. will only use the x component.
    private: void OnMsg(ConstVector3dPtr &_msg)
    {
      this->SetVelocity(_msg->x());
    }
    
    /*-----------------------------------------------------------*/
    
    /*----------------------ROS setting velocity joint API------------*/
    // A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;
    // A ROS subscriber
    private: ros::Subscriber rosSub;
    // A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;
    // A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;
    
    //Handle an incoming message from ROS.
    //  _msg A float value that is used to set the velocity of the joint.
    public: void OnRosMsg(const std_msgs::Float32ConstPtr &_msg)
    {
      this->SetVelocity(_msg->data);
    }

    //ROS helper function that processes messages
    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }
    /*-----------------------------------------------------------*/
    
    //Set velocity of the revolute JOINT
    public: void SetVelocity(const double &_vel)
    {
      // Set the joint's target velocity.
      this->model->GetJointController()->SetVelocityTarget(
      this->joint->GetScopedName(), _vel);
    }
        
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(revolute_control_plugin)
}
#endif
