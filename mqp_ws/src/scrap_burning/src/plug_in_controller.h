//
// Created by nkalish on 10/28/20.
//
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>

namespace scrap_burning_controllers {

    class PlugInController : public controller_interface::MultiInterfaceController<
            franka_hw::FrankaModelInterface,
            hardware_interface::EffortJointInterface,
            franka_hw::FrankaStateInterface> {

    public:
        bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
        void starting(const ros::Time&) override;
        void update(const ros::Time&, const ros::Duration& period) override;

    private:
        std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
        std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
        std::vector<hardware_interface::JointHandle> joint_handles_;
        
    };
}