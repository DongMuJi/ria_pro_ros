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

#include "ria_pro_base/ria_pro_base.h"

RiaProBase::RiaProBase(ros::NodeHandle& nh, ros::NodeHandle& nh_priv)
  : nh_(nh)
  , nh_priv_(nh_priv)
  , port_("/dev/ttyUSB0")
  , baud_(115200)
  , robot_("ria_pro")
  , joint_()
  , wheel_radius_(0.07)
  , max_speed_(1.5)
  , max_rpm_(200)
  , control_frequency_(50.0)
  , previous_state_(false)
{
  nh_priv_.getParam("port", port_);
  nh_priv_.getParam("baud", baud_);
  nh_priv_.getParam("robot", robot_);
  nh_priv_.getParam("joint", joint_);
  // nh_priv_.getParam("wheel_radius", wheel_radius_);
  // nh_priv_.getParam("max_speed", max_speed_);
  // nh_priv_.getParam("max_rpm", max_rpm_);
  nh_priv_.getParam("control_frequency", control_frequency_);

  controller_ = make_shared<GaitechController>(robot_, joint_, wheel_radius_, max_speed_, max_rpm_, port_, baud_,
                                               feedback_, estop_state_, docked_);
  hw_ = make_shared<GaitechHardware>(joint_, robot_);
  cm_ = make_shared<controller_manager::ControllerManager>(hw_.get(), nh_);
  diagnostics_ = make_shared<GaitechDiagnostics>(robot_, feedback_);
}

bool RiaProBase::init()
{
  cmd_.command.assign(joint_.size(), 0.0);

  rp_feedback_.init(nh_, robot_ + "/feedback", 1);
  rp_feedback_.msg_.robot = robot_;

  for (size_t i = 0; i < ceil(joint_.size() / 2.0); i++)
  {
    rp_feedback_.msg_.controller_state.push_back(ria_pro_msgs::ControllerState());
    rp_feedback_.msg_.motor_state.push_back(ria_pro_msgs::MotorState());
    feedback_.controller_state.push_back(ControllerState());
    feedback_.motor_state.push_back(MotorState());
  }

  rp_estop_.init(nh_, "/hardware_estop", 1);
  rp_estop_.msg_.data = false;

  // rp_docked_.init(nh_, robot_ + "/docked", 1);
  // rp_docked_.msg_.data = false;

  if (!controller_->init())
    return false;

  if (!controller_->connect())
    return false;

  return true;
}

void RiaProBase::publishFeedback()
{
  if (rp_feedback_.trylock())
  {
    rp_feedback_.msg_.header.stamp = ros::Time::now();

    for (size_t i = 0; i < ceil(joint_.size() / 2.0); i++)
    {
      rp_feedback_.msg_.controller_state[i].battery_voltage = feedback_.controller_state[i].battery_voltage;
      rp_feedback_.msg_.controller_state[i].temperature = feedback_.controller_state[i].temperature;
      rp_feedback_.msg_.controller_state[i].fault_flags = feedback_.controller_state[i].fault_flags;
      rp_feedback_.msg_.controller_state[i].emergency_stop = feedback_.controller_state[i].emergency_stop;

      rp_feedback_.msg_.motor_state[i].velocity = feedback_.motor_state[i].velocity;
      rp_feedback_.msg_.motor_state[i].position = feedback_.motor_state[i].position;
      rp_feedback_.msg_.motor_state[i].current = feedback_.motor_state[i].current;
    }
    rp_feedback_.unlockAndPublish();
  }
}

void RiaProBase::publishEstopState()
{
  if (estop_state_.data != previous_state_)
  {
    if (rp_estop_.trylock())
    {
      rp_estop_.msg_ = estop_state_;
      rp_estop_.unlockAndPublish();
    }
    previous_state_ = rp_estop_.msg_.data;
  }
}

/*void RiaProBase::publishDocked()
{
  if (rp_docked_.trylock())
  {
    rp_docked_.msg_ = docked_;
    rp_docked_.unlockAndPublish();
  }
}*/

void RiaProBase::publishLoop()
{
  while (ros::ok())
  {
    publishFeedback();
    publishEstopState();
    // publishDocked();
    diagnostics_->updateDiagnosticsMessage();

    ros::Rate(control_frequency_).sleep();
  }
}

void RiaProBase::controlLoop()
{
  chrono::steady_clock::time_point last_time = chrono::steady_clock::now();

  while (ros::ok())
  {
    chrono::steady_clock::time_point current_time = chrono::steady_clock::now();
    chrono::duration<double> elapsed_time = current_time - last_time;
    ros::Duration elapsed(elapsed_time.count());
    last_time = current_time;

    hw_->receiveFeedback(feedback_);
    cm_->update(ros::Time::now(), elapsed);
    hw_->writeCommandToHardware(cmd_);
    controller_->sendCommand(cmd_);

    ros::Rate(control_frequency_).sleep();
  }
}

int main(int argc, char** argv)
{
  // Initialize ROS node
  ros::init(argc, argv, "ria_pro_base_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  auto ria_pro = make_shared<RiaProBase>(nh, nh_priv);

  if (ria_pro->init())
  {
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Create thread for controller manager loop
    thread contol([&ria_pro]() -> void { ria_pro->controlLoop(); });

    // Create thread for publishing the feedback data
    thread publish([&ria_pro]() -> void { ria_pro->publishLoop(); });

    while (ros::ok())
      ria_pro->controller_->read();

    spinner.stop();
  }

  return EXIT_FAILURE;
}