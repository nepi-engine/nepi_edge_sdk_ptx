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
#include "ptx_node.h"

#define DEFAULT_STATUS_UPDATE_RATE_HZ   50

namespace Numurus
{

PTXNode::PTXNode() :
    status_update_rate_hz{"status_update_rate_hz", DEFAULT_STATUS_UPDATE_RATE_HZ, this}
{
    // Initialize the jog stop timer
    move_stop_timer = n.createTimer(ros::Duration(1000000.0), &PTXNode::stopMovingTimerCb, this, true);
    move_stop_timer.stop();    
}

PTXNode::~PTXNode()
{
    if (ptx_interface)
    {
        delete ptx_interface;
        ptx_interface = nullptr;
    }
}

void PTXNode::retrieveParams()
{
    SDKNode::retrieveParams();

    status_update_rate_hz.retrieve();
}

void PTXNode::run()
{
    init();
    ros::Duration(0.1).sleep();

    reportPanTiltIdentity();

    ROS_INFO("Driving to configured Home position");
    const std_msgs::Empty::ConstPtr msg;
    ptx_interface->goHomeHandler(msg);

    const float rate_hz = status_update_rate_hz;
    ros::Rate r(rate_hz);
    ROS_INFO("Starting main loop (%0.2fHz)", rate_hz);
    while (ros::ok())
    {
        ros::spinOnce();
        ptx_interface->publishJointStateAndStatus();
        ++loop_count;
        r.sleep();
    }
}

}