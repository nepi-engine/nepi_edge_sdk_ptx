/*
 * NEPI Dual-Use License
 * Project: nepi_edge_sdk_ptx
 *
 * This license applies to any user of NEPI Engine software
 *
 * Copyright (C) 2023 Numurus, LLC <https://www.numurus.com>
 * see https://github.com/numurus-nepi/nepi_edge_sdk_ptx
 *
 * This software is dual-licensed under the terms of either a NEPI software developer license
 * or a NEPI software commercial license.
 *
 * The terms of both the NEPI software developer and commercial licenses
 * can be found at: www.numurus.com/licensing-nepi-engine
 *
 * Redistributions in source code must retain this top-level comment block.
 * Plagiarizing this software to sidestep the license obligations is illegal.
 *
 * Contact Information:
 * ====================
 * - https://www.numurus.com/licensing-nepi-engine
 * - mailto:nepi@numurus.com
 *
 */
#include "ptx_interface.h"
#include "ptx_node.h"

#include "nepi_ros_interfaces/PanTiltStatus.h"

namespace Numurus
{

PTXInterface::PTXInterface(PTXNode *parent, ros::NodeHandle *parent_pub_nh, ros::NodeHandle *parent_priv_nh, const PTXSettings &default_settings):
    SDKInterface{parent, parent_pub_nh, parent_priv_nh},
    frame_id{"frame_id", default_settings.frame_id, parent},
	yaw_joint_name{"yaw_joint_name", default_settings.yaw_joint_name, parent},
	pitch_joint_name{"pitch_joint_name", default_settings.pitch_joint_name, parent},
    yaw_home_pos_deg{"home_position/yaw_deg", 0.0f, parent},
    pitch_home_pos_deg{"home_position/pitch_deg", 0.0f, parent},
    min_yaw_hardstop_deg{"limits/min_yaw_hardstop_deg", default_settings.min_yaw_hardstop_deg, parent},
    max_yaw_hardstop_deg{"limits/max_yaw_hardstop_deg", default_settings.max_yaw_hardstop_deg, parent},
    min_yaw_softstop_deg{"limits/min_yaw_softstop_deg", default_settings.min_yaw_hardstop_deg, parent},
    max_yaw_softstop_deg{"limits/max_yaw_softstop_deg", default_settings.max_yaw_hardstop_deg, parent},
    min_pitch_hardstop_deg{"limits/min_pitch_hardstop_deg", default_settings.min_pitch_hardstop_deg, parent},
    max_pitch_hardstop_deg{"limits/max_pitch_hardstop_deg", default_settings.max_pitch_hardstop_deg, parent},
    min_pitch_softstop_deg{"limits/min_pitch_softstop_deg", default_settings.min_pitch_hardstop_deg, parent},
    max_pitch_softstop_deg{"limits/max_pitch_softstop_deg", default_settings.max_pitch_hardstop_deg, parent},
    speed_ratio{"speed_ratio", default_settings.speed_ratio, parent},
    max_speed_driver_units{default_settings.max_speed_driver_units},
    min_speed_driver_units{default_settings.min_speed_driver_units}
{ 
    // Initialize the joint_state message
    joint_state.name.resize(2);
    joint_state.position.resize(2);
    joint_state.velocity.resize(2);
    joint_state.effort.resize(2);
    joint_state.name[0] = default_settings.yaw_joint_name;
    joint_state.name[1] = default_settings.pitch_joint_name;
}

PTXInterface::~PTXInterface(){}

void PTXInterface::retrieveParams()
{
    SDKInterface::retrieveParams();

    frame_id.retrieve();
	yaw_joint_name.retrieve();
	pitch_joint_name.retrieve();
    yaw_home_pos_deg.retrieve();
    pitch_home_pos_deg.retrieve();
    min_yaw_hardstop_deg.retrieve();
    max_yaw_hardstop_deg.retrieve();
    min_yaw_softstop_deg.retrieve();
    max_yaw_softstop_deg.retrieve();
    min_pitch_hardstop_deg.retrieve();
    max_pitch_hardstop_deg.retrieve();
    min_pitch_softstop_deg.retrieve();
    max_pitch_softstop_deg.retrieve();
    speed_ratio.retrieve();
}

void PTXInterface::initSubscribers()
{
    SDKInterface::initSubscribers();

    subscribers.push_back(_parent_priv_nh->subscribe("ptx/set_speed_ratio", 3, &PTXInterface::setSpeedRatioHandler, this));
    subscribers.push_back(_parent_priv_nh->subscribe("ptx/set_home_position", 3, &PTXInterface::setHomePositionHandler, this));
    subscribers.push_back(_parent_priv_nh->subscribe("ptx/set_soft_limits", 3, &PTXInterface::setSoftLimitsHandler, this));
    subscribers.push_back(_parent_priv_nh->subscribe("ptx/go_home", 3, &PTXInterface::goHomeHandler, this));
    subscribers.push_back(_parent_priv_nh->subscribe("ptx/jog_to_position", 3, &PTXInterface::jogToPositionHandler, this));
    subscribers.push_back(_parent_priv_nh->subscribe("ptx/jog_to_yaw_ratio", 3, &PTXInterface::jogToYawRatioHandler, this));
    subscribers.push_back(_parent_priv_nh->subscribe("ptx/jog_to_pitch_ratio", 3, &PTXInterface::jogToPitchRatioHandler, this));
    subscribers.push_back(_parent_priv_nh->subscribe("ptx/stop_moving", 3, &PTXInterface::stopMovingHandler, this));
}

void PTXInterface::initPublishers()
{
    SDKInterface::initPublishers();

    //set publisher
    jointPub = _parent_pub_nh->advertise<sensor_msgs::JointState>("/joint_states", 10);
    statusPub = _parent_priv_nh->advertise<nepi_ros_interfaces::PanTiltStatus>("ptx/status", 10);
}

bool PTXInterface::positionIsValid(float yaw_deg, float pitch_deg) const
{
    const float min_yaw = min_yaw_softstop_deg;
    const float max_yaw = max_yaw_softstop_deg;
    const float min_pitch = min_pitch_softstop_deg;
    const float max_pitch = max_pitch_softstop_deg;

    if ((yaw_deg < min_yaw) || (yaw_deg > max_yaw) || (pitch_deg < min_pitch) || (pitch_deg > max_pitch))
    {
        return false;
    }

    return true;
}

void PTXInterface::publishJointStateAndStatus()
{
    PTXStatus status;
    static_cast<PTXNode*>(_parent_node)->getStatus(status);
  
    const double yawRad = 0.01745329 * status.yaw_now;
    const double pitchRad = 0.01745329 * status.pitch_now;

    joint_state.header.stamp = ros::Time::now();
    joint_state.position[0] = yawRad;
    joint_state.position[1] = pitchRad;
    jointPub.publish(joint_state);

    nepi_ros_interfaces::PanTiltStatus status_msg;
    status_msg.header.seq = status.loop_count;
    status_msg.header.stamp = ros::Time::now();
    status_msg.header.frame_id = frame_id;
    
    status_msg.serial_num = status.serial_num;
    status_msg.hw_version = status.hw_version;
    status_msg.sw_version = status.sw_version;
    status_msg.speed_ratio = (float)(status.speed_driver_units - min_speed_driver_units) / (max_speed_driver_units - min_speed_driver_units);

    status_msg.yaw_home_pos_deg = yaw_home_pos_deg;
    status_msg.yaw_goal_deg = status.yaw_goal;
    status_msg.yaw_now_deg = status.yaw_now;
    status_msg.yaw_min_hardstop_deg = min_yaw_hardstop_deg;
    status_msg.yaw_max_hardstop_deg = max_yaw_hardstop_deg;
    status_msg.yaw_min_softstop_deg = min_yaw_softstop_deg;
    status_msg.yaw_max_softstop_deg = max_yaw_softstop_deg;
    
    status_msg.pitch_home_pos_deg = pitch_home_pos_deg;
    status_msg.pitch_goal_deg = status.pitch_goal;
    status_msg.pitch_now_deg = status.pitch_now;
    status_msg.pitch_min_hardstop_deg = min_pitch_hardstop_deg;
    status_msg.pitch_max_hardstop_deg = max_pitch_hardstop_deg;
    status_msg.pitch_min_softstop_deg = min_pitch_softstop_deg;
    status_msg.pitch_max_softstop_deg = max_pitch_softstop_deg;
    
    status_msg.error_msgs = status.driver_errors;

    statusPub.publish(status_msg);
}

void PTXInterface::setSpeedRatioHandler(const std_msgs::Float32::ConstPtr &msg)
{
    const float requested_speed_ratio = msg->data;
    if (requested_speed_ratio < 0.0f || requested_speed_ratio > 1.0f)
    {
        ROS_WARN("Invalid speed ratio requested (%0.2f): Must be in [0.0, 1.0]", requested_speed_ratio);
        return;
    }

    ROS_DEBUG("Setting speed ratio to %0.2f", requested_speed_ratio);
    speed_ratio = requested_speed_ratio;

    // Preserve current motion goal at new speed
    float yaw_goal, pitch_goal;
    if ( true == static_cast<PTXNode*>(_parent_node)->inMotion(yaw_goal, pitch_goal))
    {
        const float new_speed = currentSpeedRatioToDriverUnits();
        static_cast<PTXNode*>(_parent_node)->gotoPosition(yaw_goal, pitch_goal, new_speed);
    }
}

void PTXInterface::setHomePositionHandler(const nepi_ros_interfaces::PanTiltPosition::ConstPtr &msg)
{
    const float yaw_deg = msg->yaw_deg;
    const float pitch_deg = msg->pitch_deg;

    if (false == positionIsValid(yaw_deg, pitch_deg))
    {
        ROS_ERROR("Invalid position requested [%0.2f, %0.2f] for new Home position... ignoring", yaw_deg, pitch_deg);
        return;
    }

    ROS_INFO("Updating home position: [%0.2f, %0.2f]", yaw_deg, pitch_deg);
    yaw_home_pos_deg = yaw_deg;
    pitch_home_pos_deg = pitch_deg;
}

void PTXInterface::setSoftLimitsHandler(const nepi_ros_interfaces::PanTiltLimits::ConstPtr &msg)
{
    const float hard_yaw_min = min_yaw_hardstop_deg;
    const float hard_yaw_max = max_yaw_hardstop_deg;
    const float hard_pitch_min = min_pitch_hardstop_deg;
    const float hard_pitch_max = max_pitch_hardstop_deg;
    
    if ((msg->min_yaw_softstop_deg < hard_yaw_min) ||
        (msg->max_yaw_softstop_deg > hard_yaw_max) ||
        (msg->min_pitch_softstop_deg < hard_pitch_min) ||
        (msg->max_pitch_softstop_deg > hard_pitch_max))
    {
        ROS_ERROR("Soft limits cannot exceed hard limits... ignoring");
        return;
    }

    ROS_INFO("Updating soft limits: Yaw = [%0.2f, %0.2f], Pitch = [%0.2f, %0.2f]", 
             msg->min_yaw_softstop_deg, msg->max_yaw_softstop_deg, msg->min_pitch_softstop_deg, msg->max_pitch_softstop_deg);
    
    min_yaw_softstop_deg = msg->min_yaw_softstop_deg;
    max_yaw_softstop_deg = msg->max_yaw_softstop_deg;
    min_pitch_softstop_deg = msg->min_pitch_softstop_deg;
    max_pitch_softstop_deg = msg->max_pitch_softstop_deg;

    // TODO: Move to soft limits if current position not within (or at least report it)?
    // At present, the next valid motion command (relative or absolute) will move system to inside soft limits
}

void PTXInterface::goHomeHandler(const std_msgs::Empty::ConstPtr &msg)
{
    ROS_INFO("Returning home by request");
    const float yaw_deg = yaw_home_pos_deg;
    const float pitch_deg = pitch_home_pos_deg;
    const uint16_t speed = currentSpeedRatioToDriverUnits();
    
    static_cast<PTXNode*>(_parent_node)->gotoPosition(yaw_deg, pitch_deg, speed);
}

void PTXInterface::jogToPositionHandler(const nepi_ros_interfaces::PanTiltPosition::ConstPtr &msg)
{
    const float yaw_deg = msg->yaw_deg;
    const float pitch_deg = msg->pitch_deg;
    const uint16_t speed = currentSpeedRatioToDriverUnits();

    if (false == positionIsValid(yaw_deg, pitch_deg))
    {
        ROS_ERROR("Invalid jog position requested [%0.2f, %0.2f]... ignoring", yaw_deg, pitch_deg);
        return;
    }

    static_cast<PTXNode*>(_parent_node)->gotoPosition(yaw_deg, pitch_deg, speed);    
}

void PTXInterface::jogToYawRatioHandler(const std_msgs::Float32::ConstPtr &msg)
{
    if (msg->data < 0.0f || msg->data > 1.0f)
    {
        ROS_ERROR("Invalid yaw ratio %0.2f... ignoring", msg->data);
        return;
    }

    const float ratio = msg->data;
    const float min_yaw = min_yaw_softstop_deg;
    const float max_yaw = max_yaw_softstop_deg;
    const float yaw_goal = (ratio * max_yaw) + (( 1.0f - ratio ) *  min_yaw);
    const float speed = currentSpeedRatioToDriverUnits();
    
    float dummy, pitch_goal;
    static_cast<PTXNode*>(_parent_node)->inMotion(dummy, pitch_goal); // Just to capture pitch_goal
    static_cast<PTXNode*>(_parent_node)->gotoPosition(yaw_goal, pitch_goal, speed);
}

void PTXInterface::jogToPitchRatioHandler(const std_msgs::Float32::ConstPtr &msg)
{
    if (msg->data < 0.0f || msg->data > 1.0f)
    {
        ROS_ERROR("Invalid pitch ratio %0.2f... ignoring", msg->data);
        return;
    }

    const float ratio = msg->data;
    const float min_pitch = min_pitch_softstop_deg;
    const float max_pitch = max_pitch_softstop_deg;
    const float pitch_goal = (ratio * max_pitch) + (( 1.0f - ratio ) *  min_pitch);
    const float speed = currentSpeedRatioToDriverUnits();
    
    float dummy, yaw_goal;
    static_cast<PTXNode*>(_parent_node)->inMotion(yaw_goal, dummy); // Just to capture yaw_goal
    static_cast<PTXNode*>(_parent_node)->gotoPosition(yaw_goal, pitch_goal, speed);
}

void PTXInterface::stopMovingHandler(const std_msgs::Empty::ConstPtr &msg)
{
    static_cast<PTXNode*>(_parent_node)->stopMotion();
}

}