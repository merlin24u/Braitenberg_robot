#ifndef GAZEBO_ROS_LIGHT_SENSOR_HH
#define GAZEBO_ROS_LIGHT_SENSOR_HH
    
#include <gazebo/plugins/CameraPlugin.hh> // library for processing camera data for gazebo / ros conversions
#include <gazebo_plugins/gazebo_ros_camera_utils.h>
#include <sensor_msgs/image_encodings.h>
#include <string>
    
namespace gazebo{
  
  class GazeboRosLight : public CameraPlugin, GazeboRosCameraUtils{

  private:
    ros::NodeHandle n;
    ros::Publisher sensorPublisher;
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;
    
    double fov;
    double range;

  public:
    /// \brief Constructor
    /// \param parent The parent entity, must be a Model or a Sensor
    GazeboRosLight();
    
    /// \brief Destructor
    ~GazeboRosLight();
    
    /// \brief Load the plugin
    /// \param take in SDF root element
    void Load(sensors::SensorPtr parent, sdf::ElementPtr sdf);

    void imagePub(const sensor_msgs::ImageConstPtr &img);

  protected:
    /// \brief Update the controller
    virtual void OnNewFrame(const unsigned char *_image,
			    unsigned int _width, unsigned int _height,
			    unsigned int _depth, const std::string &_format);
  };
}
#endif
