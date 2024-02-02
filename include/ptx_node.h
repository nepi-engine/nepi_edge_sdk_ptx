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

  typedef uint8_t PTX_DIRECTION;
  static constexpr PTX_DIRECTION PTX_DIRECTION_POSITIVE = 1;
  static constexpr PTX_DIRECTION PTX_DIRECTION_NEGATIVE = -1;
  

  // Pure virtual methods must be implemented in concrete base classes (e.g., with driver support)
  // PTXInterface requires these (and that they are public)
  virtual void moveYaw(PTX_DIRECTION direction, float speed, float time_s = 1000000.0f) = 0;
  virtual void movePitch(PTX_DIRECTION direction, float speed, float time_s = 1000000.0f) = 0;
  virtual void gotoPosition(float yaw_deg, float pitch_deg, float speed, float move_timeout_s = 1000000.0f) = 0;
  virtual void getCurrentPosition(float &yaw_deg_out, float &pitch_deg_out) = 0;
  virtual void getStatus(PTXStatus &status_out) = 0;
  virtual bool inMotion(float &yaw_goal_out, float &pitch_goal_out) = 0;
  virtual void stopMotion() = 0;
  virtual void reportPanTiltIdentity() const = 0;

protected:
  NodeParam<float> status_update_rate_hz;
  uint32_t loop_count = 0;
  PTXInterface *ptx_interface = nullptr;
  ros::Timer move_stop_timer;

  void stopMovingTimerCb(const ros::TimerEvent& ev){stopMotion();}
};

}

#endif