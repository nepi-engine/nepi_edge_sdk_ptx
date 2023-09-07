#ifndef _PTX_NODE_H
#define _PTX_NODE_H

#include "sdk_node.h"
#include "ptx_interface.h"

namespace Numurus
{

class PTXInterface; // Forward declaration

class PTXNode :  public SDKNode
{
public:
  PTXNode();
  virtual ~PTXNode();

  // SDKNode overrides
  void retrieveParams() override;
  void run() override;

  // Pure virtual methods must be implemented in concrete base classes (e.g., with driver support)
  // PTXInterface requires these (and that they are public)
  virtual void gotoPosition(float yaw_deg, float pitch_deg, float speed) = 0;
  virtual void getCurrentPosition(float &yaw_deg_out, float &pitch_deg_out) = 0;
  virtual void getStatus(PTXStatus &status_out) = 0;
  virtual bool inMotion(float &yaw_goal_out, float &pitch_goal_out) = 0;
  virtual void stopMotion() = 0;
  virtual void reportPanTiltIdentity() const = 0;

protected:
  NodeParam<float> status_update_rate_hz;
  uint32_t loop_count = 0;
  PTXInterface *ptx_interface = nullptr;
};

}

#endif