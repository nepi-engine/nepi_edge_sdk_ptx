#ifndef _PTX_INTERFACE_H
#define _PTX_INTERFACE_H

#include "sdk_interface.h"
#include "sdk_node.h" // NodeParam

#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include "nepi_ros_interfaces/PanTiltPosition.h"
#include "nepi_ros_interfaces/PanTiltLimits.h"

namespace Numurus
{

struct PTXSettings
{
    std::string frame_id;
    std::string yaw_joint_name;
    std::string pitch_joint_name;

    float min_yaw_hardstop_deg;
    float max_yaw_hardstop_deg;
    float min_pitch_hardstop_deg;
    float max_pitch_hardstop_deg;
    float min_speed_driver_units;
    float max_speed_driver_units;
    float speed_ratio;
};

struct PTXStatus
{
    uint32_t loop_count;

    std::string serial_num;
    std::string hw_version;
    std::string sw_version;

    float yaw_goal;
    float yaw_now;
    
    float pitch_goal;
    float pitch_now;
    
    uint32_t speed_driver_units;

    std::vector<std::string> driver_errors;
};

class PTXNode; // forward declaration

class PTXInterface : public SDKInterface
{
public:
    PTXInterface(PTXNode *parent, ros::NodeHandle *parent_pub_nh, ros::NodeHandle *parent_priv_nh, const PTXSettings &default_settings);
    PTXInterface() = delete; // No default constructor available

    virtual ~PTXInterface();

    // SDKInterface overrides
	void retrieveParams() override;
	void initSubscribers() override;
	void initPublishers() override;

    bool positionIsValid(float yaw_deg, float pitch_deg) const;
    inline uint16_t currentSpeedRatioToDriverUnits() const 
    { 
        const float speed = speed_ratio; 
        return (uint16_t)(speed * max_speed_driver_units + ((1.0 - speed) * min_speed_driver_units));
    }
    void publishJointStateAndStatus();
    void setSpeedRatioHandler(const std_msgs::Float32::ConstPtr &msg);
    void setHomePositionHandler(const nepi_ros_interfaces::PanTiltPosition::ConstPtr &msg);
    void setSoftLimitsHandler(const nepi_ros_interfaces::PanTiltLimits::ConstPtr &msg);
    void goHomeHandler(const std_msgs::Empty::ConstPtr &msg);
    void jogToPositionHandler(const nepi_ros_interfaces::PanTiltPosition::ConstPtr &msg);
    void jogToYawRatioHandler(const std_msgs::Float32::ConstPtr &msg);
    void jogToPitchRatioHandler(const std_msgs::Float32::ConstPtr &msg);  
    void stopMovingHandler(const std_msgs::Empty::ConstPtr &msg);

private:
    SDKNode::NodeParam<std::string> frame_id;
	SDKNode::NodeParam<std::string> yaw_joint_name;
	SDKNode::NodeParam<std::string> pitch_joint_name;
    SDKNode::NodeParam<float> yaw_home_pos_deg;
    SDKNode::NodeParam<float> pitch_home_pos_deg;
    SDKNode::NodeParam<float> min_yaw_hardstop_deg;
    SDKNode::NodeParam<float> max_yaw_hardstop_deg;
    SDKNode::NodeParam<float> min_yaw_softstop_deg;
    SDKNode::NodeParam<float> max_yaw_softstop_deg;
    SDKNode::NodeParam<float> min_pitch_hardstop_deg;
    SDKNode::NodeParam<float> max_pitch_hardstop_deg;
    SDKNode::NodeParam<float> min_pitch_softstop_deg;
    SDKNode::NodeParam<float> max_pitch_softstop_deg;
    SDKNode::NodeParam<float> speed_ratio;

    float max_speed_driver_units;
    float min_speed_driver_units;

    ros::Publisher jointPub;
    ros::Publisher statusPub;
        
    sensor_msgs::JointState joint_state;
};

}

#endif