/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, Universitaet Hamburg
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Michael 'v4hn' Goerner */

#include <pr2_mechanism_model/robot.h>
#include <pr2_controller_interface/controller.h>

#include <ros/ros.h>
#include <pluginlib/class_list_macros.hpp>

#include <tams_pr2_refine_startup_calibration/SetZeroOffset.h>
#include <tams_pr2_refine_startup_calibration/GetZeroOffset.h>

namespace controller {

class SetZeroOffsetController : public pr2_controller_interface::Controller
{
public:
  SetZeroOffsetController();
  ~SetZeroOffsetController() override;

  bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle& n) override;
  void starting() override;
  void stopping() override;
  void update() override;

  bool setZeroOffset(tams_pr2_refine_startup_calibration::SetZeroOffset::Request& req, tams_pr2_refine_startup_calibration::SetZeroOffset::Response& resp);
  bool getZeroOffset(tams_pr2_refine_startup_calibration::GetZeroOffset::Request& req, tams_pr2_refine_startup_calibration::GetZeroOffset::Response& resp);

protected:
  ros::NodeHandle node_;
  ros::ServiceServer set_zero_offset_srv_;
  ros::ServiceServer get_zero_offset_srv_;
  bool new_zero_offset_available_;
  double new_zero_offset_;

  pr2_hardware_interface::Actuator *actuator_;
  pr2_mechanism_model::JointState *joint_;
  std::string joint_name_;
};

SetZeroOffsetController::SetZeroOffsetController()
: actuator_(NULL)
{
}

SetZeroOffsetController::~SetZeroOffsetController()
{
}

bool SetZeroOffsetController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n)
{
  node_ = n;

  // Joint
  if (!node_.getParam("joint", joint_name_))
  {
    ROS_ERROR_STREAM("No joint given (namespace: " << node_.getNamespace() << ")");
    return false;
  }
  joint_ = robot->getJointState(joint_name_);
  if (!joint_)
  {
    ROS_ERROR_STREAM("Could not find joint " << joint_name_ << " (namespace: " << node_.getNamespace() << ")");
    return false;
  }

  // Actuator
  std::string actuator_name;
  if (!node_.getParam("actuator", actuator_name))
  {
    ROS_ERROR_STREAM("No actuator given (namespace: " << node_.getNamespace() << ")");
    return false;
  }
  actuator_ = robot->model_->getActuator(actuator_name);
  if (!actuator_)
  {
    ROS_ERROR_STREAM("Could not find actuator " << actuator_name << " (namespace: " << node_.getNamespace() << ")");
    return false;
  }

  set_zero_offset_srv_ = node_.advertiseService("set_zero_offset", &SetZeroOffsetController::setZeroOffset, this);
  get_zero_offset_srv_ = node_.advertiseService("get_zero_offset", &SetZeroOffsetController::getZeroOffset, this);

  return true;
}

void SetZeroOffsetController::starting()
{
}

void SetZeroOffsetController::stopping()
{
  if(new_zero_offset_available_){
    ROS_ERROR("Failed to set zero offset because the controller was stopped right before the value got updated. ignoring value");
    new_zero_offset_available_= false; 
  }
}

bool SetZeroOffsetController::setZeroOffset(
                tams_pr2_refine_startup_calibration::SetZeroOffset::Request& req,
					      tams_pr2_refine_startup_calibration::SetZeroOffset::Response& resp)
{
  if(!isRunning()){
    ROS_ERROR("Could not set zero offset because controller is not active");
    return true;
  }

  ROS_INFO_STREAM("setting zero offset for joint " << joint_name_ << " to " << req.offset);
  new_zero_offset_ = req.offset;
  new_zero_offset_available_ = true;
  return true;
}

bool SetZeroOffsetController::getZeroOffset(
                tams_pr2_refine_startup_calibration::GetZeroOffset::Request& req,
					      tams_pr2_refine_startup_calibration::GetZeroOffset::Response& resp)
{
  resp.offset = actuator_->state_.zero_offset_;
  return true;
}

void SetZeroOffsetController::update()
{
  if(new_zero_offset_available_) {
    new_zero_offset_available_ = false;
    actuator_->state_.zero_offset_ = new_zero_offset_;
    joint_->calibrated_ = true;
  }
}
} // namespace

PLUGINLIB_EXPORT_CLASS(controller::SetZeroOffsetController, pr2_controller_interface::Controller)

