#ifndef _ROBOT_PLUGIN_HH_
#define _ROBOT_PLUGIN_HH_

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_braitenberg_robot/Sensor.h>
#include <thread>
#include <math.h>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

typedef Matrix<float, 2, 4> Matrix2_4f; // custom Matrix otherwise initialization doesn't seem to be working

namespace gazebo{

  /// \brief A plugin to control a MyRobot sensor.
  class RobotPlugin : public ModelPlugin{

  private:
    /// \brief Pointer to the model.
    physics::ModelPtr model;

    /// \brief A node use for ROS transport
    unique_ptr<ros::NodeHandle> rosNode;

    /// \brief A ROS subscriber
    ros::Subscriber rosSub;

    /// \brief A ROS callbackqueue that helps process messages
    ros::CallbackQueue rosQueue;

    /// \brief A thread the keeps running the rosQueue
    thread rosQueueThread;

    /// \brief Matrix used for each iteration
    Matrix2f cst;
    Matrix2_4f coeff;
    
    int MAX_SPEED; // speed in radian/s of wheels

    int BEHAVIOR; // behavior of robot (following/avoiding light)
  
  public:
    /// \brief tied to behavior
    const static int FOLLOW = 0;
    const static int AVOID = 1;
    
    /// \brief Constructor
    RobotPlugin() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
      // Safety check
      if(_model->GetJointCount() == 0){
	cerr << "Invalid joint count, MyRobot plugin not loaded\n";
	return;
      }

      // Store the model pointer for convenience.
      this->model = _model;

      // Check that the sdf elements exist, then read the values
      if (_sdf->HasElement("velocity"))
	MAX_SPEED = _sdf->Get<int>("velocity");
      if (_sdf->HasElement("behavior"))
	BEHAVIOR = _sdf->Get<int>("behavior");

      // Set up matrix
      cst << 1, 1,
	1, -1;
      switch(BEHAVIOR){
      case FOLLOW :
	coeff << 4, 6, 6, 4, 
	  -4, -4, 4, 4;
	break;
      case AVOID :
	coeff << 4, 6, 6, 4, 
	  4, 4, -4, -4;
	break;
      default:
	// FOLLOW
	coeff << 4, 6, 6, 4, 
	  -4, -4, 4, 4;
	break;
      }
      
      // Initialize ros, if it has not already bee initialized.
      if(!ros::isInitialized()){
	int argc = 0;
	char **argv = NULL;
	ros::init(argc, argv, "gazebo",
		  ros::init_options::NoSigintHandler);
      }

      // Create our ROS node. This acts in a similar manner to
      // the Gazebo node
      this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

      // Create a named topic, and subscribe to it.
      ros::SubscribeOptions so =
	ros::SubscribeOptions::create<gazebo_braitenberg_robot::Sensor>(
									"/lightSensor",
									100,
									boost::bind(&RobotPlugin::onRosMsg, this, _1),
									ros::VoidPtr(), &this->rosQueue);
      this->rosSub = this->rosNode->subscribe(so);

      // Spin up the queue helper thread.
      this->rosQueueThread =
	thread(bind(&RobotPlugin::QueueThread, this));
    }

    /// \brief Handle an incoming message from ROS
    /// \param[in] data Sensors data that is used to set the velocity
    /// of the MyRobot.
    void onRosMsg(const gazebo_braitenberg_robot::SensorConstPtr &msg){
      VectorXf sensors(msg->data.size());
      for(int i = 0; i < msg->data.size(); i++)
	sensors(i) = msg->data[i] / 60;

      Vector2f vel = coeff * sensors;
      Vector2f wheel_speed = cst * vel;
      
      float k = max(wheel_speed(0), wheel_speed(1)); // scale wheel speed on MAX_SPEED
      if(k == 0)
	k = 1;
      setVelocity(wheel_speed(0) * MAX_SPEED / k, wheel_speed(1) * MAX_SPEED / k);
    }

    /// \brief Set the velocity of the MyRobot
    /// \param[in] l New left target velocity
    /// \param[in] r New right target velocity
    void setVelocity(const double &l, const double &r){
      this->model->GetJoint("my_robot::left_wheel_hinge")->SetVelocity(0, l);
      this->model->GetJoint("my_robot::right_wheel_hinge")->SetVelocity(0, r);
    }

  private:
    /// \brief ROS helper function that processes messages
    void QueueThread(){
      static const double timeout = 0.01;
      while (this->rosNode->ok())
	{
	  this->rosQueue.callAvailable(ros::WallDuration(timeout));
	}
    }
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(RobotPlugin)
}
#endif
