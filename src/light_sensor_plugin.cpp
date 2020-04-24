#include <ros/ros.h>
#include <gazebo_braitenberg_robot/Sensor.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo_plugins/gazebo_ros_camera.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/mat.hpp>
#include "gazebo_braitenberg_robot/light_sensor_plugin.hpp"

namespace gazebo
{
  // Constructor
  GazeboRosLight::GazeboRosLight(): n("light_sensor_plugin"), it(n), fov(6), range(10){
    // Subscrive to input video feed
    image_sub = it.subscribe("/camera/rgb/image_raw", 100,
			     &gazebo::GazeboRosLight::imagePub, this);

    sensorPublisher = n.advertise<gazebo_braitenberg_robot::Sensor>("/lightSensor", 1);
  }

  // Destructor
  GazeboRosLight::~GazeboRosLight()
  {
    ROS_DEBUG_STREAM_NAMED("camera","Unloaded");
  }

  void GazeboRosLight::Load(sensors::SensorPtr parent, sdf::ElementPtr sdf)
  {
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
      {
	ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
			 << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
	return;
      }

    CameraPlugin::Load(parent, sdf);
    // copying from CameraPlugin into GazeboRosCameraUtils
    this->parentSensor_ = this->parentSensor;
    this->width_ = this->width;
    this->height_ = this->height;
    this->depth_ = this->depth;
    this->format_ = this->format;
    this->camera_ = this->camera;

    GazeboRosCameraUtils::Load(parent, sdf);
  }

  // Update the controller
  void GazeboRosLight::OnNewFrame(const unsigned char *_image,
				  unsigned int _width, unsigned int _height, unsigned int _depth,
				  const std::string &_format){
    
    this->sensor_update_time_ = this->parentSensor_->LastUpdateTime();

    if(!this->parentSensor->IsActive()){
      if((*this->image_connect_count_) > 0)
	// do this first so there's chance for sensor to run once after activated
	this->parentSensor->SetActive(true);
    }
    else{
      if((*this->image_connect_count_) > 0){
	common::Time cur_time = this->world_->SimTime();
	if(cur_time - this->last_update_time_ >= this->update_period_){
	  this->PutCameraData(_image);
	  this->PublishCameraInfo();
	  this->last_update_time_ = cur_time;
	}
      }
    }
  }
  
  void GazeboRosLight::imagePub(const sensor_msgs::ImageConstPtr &_img){
    gazebo_braitenberg_robot::Sensor msg;

    cv_bridge::CvImagePtr cv_ptr;
    try{
      cv_ptr = cv_bridge::toCvCopy(_img, _img->encoding);
    }catch(const cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat img = cv_ptr->image;
    int r = img.rows;
    int c = img.cols;
    
    auto pixel = img.at<cv::Vec3b>(r/2, 0);
    float illum = 0.299 * (float)pixel[2] + 0.587 * (float)pixel[1] + 0.114 * (float)pixel[0];
    msg.data.push_back(illum);

    pixel = img.at<cv::Vec3b>(r/2, c - 1);
    illum = 0.299 * (float)pixel[2] + 0.587 * (float)pixel[1] + 0.114 * (float)pixel[0];
    msg.data.push_back(illum);
    
    sensorPublisher.publish(msg);
  }
  
  // Register this plugin with the simulator
  GZ_REGISTER_SENSOR_PLUGIN(GazeboRosLight)
}
