//
// Created by nkalish on 10/28/20.
//
#include "plug_in_controller.h"

#include <cmath>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

bool scrap_burning_controllers::PlugInController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) {
    return true;
}

void scrap_burning_controllers::PlugInController::starting(const ros::Time &) {

}

void scrap_burning_controllers::PlugInController::update(const ros::Time &, const ros::Duration &period) {

}

//void scrap_burning_controllers::PlugInController::updateDynamicReconfigure() {
//
//}
//
//void scrap_burning_controllers::PlugInController::plugInParamCallback(scrap_burning::plug_in_paramConfig &config, uint32_t level) {
//
//}

PLUGINLIB_EXPORT_CLASS(scrap_burning_controllers::PlugInController,
                       controller_interface::ControllerBase)
