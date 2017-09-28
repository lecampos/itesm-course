#ifndef _WROBOT_PLUGIN_HH_
#define _WROBOT_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "wrobot_comm/WheelVelocity.h"
#include "wrobot_comm/RobotPosition.h"

namespace gazebo
{
  /// \brief A plugin to control a wheeled robot via wheel velocity.
  class WRobotPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: WRobotPlugin() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
	      // Safety check
	  if (_model->GetJointCount() == 0)
	  {
	    std::cerr << "Invalid joint count, robot plugin not loaded\n";
	    return;
	  }

	  // Store the model pointer for convenience.
	  this->model = _model;

	  // Get the first joint. We are making an assumption about the model
	  // having one joint that is the rotational joint.
	  this->joint_1 = _model->GetJoints()[0];

	  // Get the second joint. We are making an assumption about the model
	  // having two joint that is the rotational joint.
	  this->joint_2 = _model->GetJoints()[1];

	  // Initialize ros, if it has not already bee initialized.
		// Initialize ros, if it has not already bee initialized.
		if (!ros::isInitialized())
		{
		  int argc = 0;
		  char **argv = NULL;
		  ros::init(argc, argv, "gazebo_client",
		      ros::init_options::NoSigintHandler);
		}

		// Create our ROS node. This acts in a similar manner to
		// the Gazebo node
		this->rosNode.reset(new ros::NodeHandle(this->model->GetName()));
		
		// Create a publisher
		rosPub_1 = this->rosNode->advertise<wrobot_comm::RobotPosition>("robot_position", 1);

		// Create a named topic, and subscribe to it.
		ros::SubscribeOptions so_1 =
		  ros::SubscribeOptions::create<wrobot_comm::WheelVelocity>(
		      "/" + this->model->GetName() + "/left_wheel",
		      1,
		      boost::bind(&WRobotPlugin::OnRosMsg_1, this, _1),
		      ros::VoidPtr(), &this->rosQueue);


		// Create a named topic, and subscribe to it.
		ros::SubscribeOptions so_2 =
		  ros::SubscribeOptions::create<wrobot_comm::WheelVelocity>(
		      "/" + this->model->GetName() + "/right_wheel",
		      1,
		      boost::bind(&WRobotPlugin::OnRosMsg_2, this, _1),
		      ros::VoidPtr(), &this->rosQueue);


		this->rosSub_1 = this->rosNode->subscribe(so_1);
		this->rosSub_2 = this->rosNode->subscribe(so_2);

		// Spin up the queue helper thread.
		this->rosQueueThread =
		  std::thread(std::bind(&WRobotPlugin::QueueThread, this));
	}

	/// \brief Handle an incoming message from ROS
	/// \param[in] _msg A float value that is used to set the velocity
	/// of the robot's joint 1.
	public: void OnRosMsg_1(const wrobot_comm::WheelVelocityConstPtr &_msg)
	{
	  this->SetVelocity(_msg->velocity.data, this->joint_1);
	}

	/// \brief Handle an incoming message from ROS
	/// \param[in] _msg A float value that is used to set the velocity
	/// of the robot's joint 2.
	public: void OnRosMsg_2(const wrobot_comm::WheelVelocityConstPtr &_msg)
	{
	  this->SetVelocity(_msg->velocity.data, this->joint_2);
	}

	/// \brief ROS helper function that processes messages
	private: void QueueThread()
	{
	  static const double timeout = 0.01;
	  while (this->rosNode->ok())
	  {

	  	GetRobotPosition();
	  	this->msg.header.stamp = ros::Time::now();
	
		this->rosPub_1.publish(this->msg);

	    this->rosQueue.callAvailable(ros::WallDuration(timeout));
	  }
	}

	/// \brief Gets the robot position
	public: void GetRobotPosition(void)
	{
		// Get robot posw w.r.t wolrd frame.
	   	this->pose = this->model->GetWorldPose();

	  	// We only care about x, y and yaw.
	  	this->msg.pose.x = this->pose.pos.x;
	  	this->msg.pose.y = this->pose.pos.y;
	  	// TODO: angle between x and y
	  	this->msg.pose.theta = 0.0;
	}

	/// \brief Set the velocity of the robot
	/// \param[in] _vel 	New target velocity
	/// \param[in] joint_ 	The joint to apply the velocity
	public: void SetVelocity(const double &_vel, physics::JointPtr joint_)
	{
	  // Set the joint's target velocity.
	  this->model->GetJointController()->SetVelocityTarget(
	      joint_->GetScopedName(), _vel);
	}

	/// \brief Pointer to the model.
	private: physics::ModelPtr model;

	/// \brief Pointer to the joint.
	private: physics::JointPtr joint_1;

		/// \brief Pointer to the joint.
	private: physics::JointPtr joint_2;

	// \brief math pose descriptor
	private: gazebo::math::Pose pose;

	/// \brief A node use for ROS transport
	private: std::unique_ptr<ros::NodeHandle> rosNode;

	/// \brief A ROS subscriber
	private: ros::Subscriber rosSub_1;
	/// \brief A ROS subscriber
	private: ros::Subscriber rosSub_2;
	/// \brief A ROS publisher
	private: ros::Publisher rosPub_1;
	/// \brief Custom message type for publication
	private: wrobot_comm::RobotPosition msg;
	/// \brief A ROS callbackqueue that helps process messages
	private: ros::CallbackQueue rosQueue;
	/// \brief A thread the keeps running the rosQueue
	private: std::thread rosQueueThread;

  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(WRobotPlugin)
}
#endif
