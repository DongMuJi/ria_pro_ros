/*
Software License Agreement (BSD License)

Authors : Brighten Lee <shlee@gaitech.co.kr>

Copyright (c) 2020, Gaitech Korea Co., Ltd.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef RIA_PRO_BASE__RIA_PRO_BASE_H_
#define RIA_PRO_BASE__RIA_PRO_BASE_H_

#include "ros/ros.h"
#include "controller_manager/controller_manager.h"
#include "realtime_tools/realtime_publisher.h"

#include "gaitech_base/gaitech_controller.h"
#include "gaitech_base/gaitech_diagnostics.h"
#include "gaitech_base/gaitech_hardware.h"

#include "ria_pro_msgs/Command.h"
#include "ria_pro_msgs/Feedback.h"
#include "ria_pro_msgs/Reset.h"

using namespace std;

class RiaProBase
{
public:
  RiaProBase(ros::NodeHandle& nh, ros::NodeHandle& nh_priv);

  virtual ~RiaProBase() = default;

  /**
   * \brief Initialize
   */
  bool init();

  /**
   * \brief Publish feedback data
   */
  void publishFeedback();

  /**
   * \brief Publish estop state
   */
  void publishEstopState();

  /**
   * \brief Publish docking signal
   */
  void publishDocked();

  /**
   * \brief Loop for publishing
   */
  void publishLoop();

  /**
   * \brief Loop for controller manager
   */
  void controlLoop();

  /// Communication system with motor controller
  shared_ptr<GaitechController> controller_;

  /// Hardware interface for robot
  shared_ptr<GaitechHardware> hw_;

  /// Controller manager for the infrastructure to interact with controllers
  shared_ptr<controller_manager::ControllerManager> cm_;

  /// Diagnostics system to collect information from hardware drivers and robot
  shared_ptr<GaitechDiagnostics> diagnostics_;

private:
  /// ROS parameters
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  realtime_tools::RealtimePublisher<ria_pro_msgs::Feedback> rp_feedback_;
  realtime_tools::RealtimePublisher<std_msgs::Bool> rp_estop_;
  realtime_tools::RealtimePublisher<std_msgs::Bool> rp_docked_;

  ros::ServiceServer reset_;

  /// Serial parameters
  string port_;
  int32_t baud_;

  /// Robot name
  string robot_;

  /// Joint name
  vector<string> joint_;

  /// Robot parameters
  double wheel_radius_, max_speed_, max_rpm_;

  /// Control frequency
  double control_frequency_;

  /// Command
  Command cmd_;

  /// Feedback related
  Feedback feedback_;

  /// Estop state
  std_msgs::Bool estop_state_;

  /// Docking state
  std_msgs::Bool docked_;

  /// Previous state of Estop
  bool previous_state_;
};

#endif  // RIA_PRO_BASE__RIA_PRO_BASE_H_