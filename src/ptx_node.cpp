#include "ptx_node.h"

#define DEFAULT_STATUS_UPDATE_RATE_HZ   50

namespace Numurus
{

PTXNode::PTXNode() :
    status_update_rate_hz{"status_update_rate_hz", DEFAULT_STATUS_UPDATE_RATE_HZ, this}
{}

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