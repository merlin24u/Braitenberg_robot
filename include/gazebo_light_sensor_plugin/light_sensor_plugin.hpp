#ifndef GAZEBO_ROS_LIGHT_SENSOR_HH
#define GAZEBO_ROS_LIGHT_SENSOR_HH
    
#include <gazebo/plugins/CameraPlugin.hh> // library for processing camera data for gazebo / ros conversions
#include <gazebo_plugins/gazebo_ros_camera_utils.h>
#include <string>
    
namespace gazebo
{
  class GazeboRosLight : public CameraPlugin, GazeboRosCameraUtils
  {
    ros::NodeHandle n;
    ros::Publisher sensorPublisher;
    
    double fov;
    double range;

    /// \brief Constructor
    /// \param parent The parent entity, must be a Model or a Sensor
  public: GazeboRosLight();
    
    /// \brief Destructor
  public: ~GazeboRosLight();
    
    /// \brief Load the plugin
    /// \param take in SDF root element
  public: void Load(sensors::SensorPtr parent, sdf::ElementPtr sdf);
    
    /// \brief Update the controller
  protected: virtual void OnNewFrame(const unsigned char *image,
				     unsigned int width, unsigned int height,
				     unsigned int depth, const std::string &format);
  };
}
#endif
