#ifndef _IQR_ROS_PAN_TILT_NODE_H
#define _IQR_ROS_PAN_TILT_NODE_H

#include "ptx_node.h"

namespace IQR
{
  class PanTiltDriver; // Forward declaration
}

namespace Numurus
{

class IqrRosPanTiltNode : public PTXNode
{
public:
  IqrRosPanTiltNode(int modbus_id, const std::string &dev_fs_path);
  ~IqrRosPanTiltNode();

  // PTXNode pure virtual functions must be implemented
  // PTXInterface requires these (and that they are public)
  void gotoPosition(float yaw_deg, float pitch_deg, float speed);
  void getCurrentPosition(float &yaw_deg_out, float &pitch_deg_out);
  void getStatus(PTXStatus &status_out);
  bool inMotion(float &yaw_goal_out, float &pitch_goal_out);
  void stopMotion();
  void reportPanTiltIdentity() const;

private:
  IQR::PanTiltDriver *driver = nullptr;
};

} // namespace Numurus

#endif