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