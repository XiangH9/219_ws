//
// Created by tlab-uav on 24-9-11.
//

#include "controller_common/FSM/StateFixedDown.h"

#include <cmath>

#include <rclcpp/rclcpp.hpp>

StateFixedDown::StateFixedDown(CtrlInterfaces& ctrl_interfaces,
                               const std::vector<double>& target_pos,
                               const double kp,
                               const double kd)
    : FSMState(FSMStateName::FIXEDDOWN, "fixed down", ctrl_interfaces),
      kp_(kp), kd_(kd)
{
    duration_ = ctrl_interfaces_.frequency_ * 1.2;
    for (int i = 0; i < 12; i++)
    {
        target_pos_[i] = target_pos[i];
    }
    // ========== 新增：初始化初始增益为0 ==========
    kp_start_ = 0.0;
    kd_start_ = 0.0;
}

void StateFixedDown::enter()
{
    for (int i = 0; i < 12; i++)
    {
        start_pos_[i] = ctrl_interfaces_.joint_position_state_interface_[i].get().get_value();
    }
    ctrl_interfaces_.control_inputs_.command = 0;
    for (int i = 0; i < 12; i++)
    {
        ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(start_pos_[i]);
        ctrl_interfaces_.joint_velocity_command_interface_[i].get().set_value(0);
        ctrl_interfaces_.joint_torque_command_interface_[i].get().set_value(0);
        ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(kp_start_);
        ctrl_interfaces_.joint_kd_command_interface_[i].get().set_value(kd_start_);
    }
    // ========== 新增：重置percent，确保从0开始 ==========
    percent_ = 0.0;
    if (ctrl_interfaces_.node)
    {
        RCLCPP_INFO(
            ctrl_interfaces_.node->get_logger(),
            "[StateFixedDown::enter] start hip=%.3f thigh=%.3f calf=%.3f | kp_start=%.1f kd_start=%.1f | kp_target=%.1f kd_target=%.1f",
            start_pos_[0], start_pos_[1], start_pos_[2],
            kp_start_, kd_start_, kp_, kd_);
    }
}

void StateFixedDown::run(const rclcpp::Time&/*time*/, const rclcpp::Duration&/*period*/)
{
    percent_ += 1 / duration_;
    // 用tanh做平滑过渡，范围[0, 1]
    phase = std::tanh(percent_);
    // ========== 核心修改3：不仅平滑位置，还同步平滑PID增益 ==========
    double current_kp = phase * kp_ + (1 - phase) * kp_start_;
    double current_kd = phase * kd_ + (1 - phase) * kd_start_;
    for (int i = 0; i < 12; i++)
    {
        ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(
            phase * target_pos_[i] + (1 - phase) * start_pos_[i]);
        // ========== 新增：同步平滑过渡PID增益 ==========
        ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(current_kp);
        ctrl_interfaces_.joint_kd_command_interface_[i].get().set_value(current_kd);
    }
    if (ctrl_interfaces_.node)
    {
        RCLCPP_INFO_THROTTLE(
            ctrl_interfaces_.node->get_logger(),
            *ctrl_interfaces_.node->get_clock(),
            2000,
            "[StateFixedDown::run] percent=%.3f phase=%.3f | kp=%.2f kd=%.2f | "
            "hip target=%.3f cmd=%.3f q=%.3f | "
            "thigh target=%.3f cmd=%.3f q=%.3f | "
            "calf target=%.3f cmd=%.3f q=%.3f",
            percent_, phase, current_kp, current_kd,
            target_pos_[0], ctrl_interfaces_.joint_position_command_interface_[0].get().get_value(),
            ctrl_interfaces_.joint_position_state_interface_[0].get().get_value(),
            target_pos_[1], ctrl_interfaces_.joint_position_command_interface_[1].get().get_value(),
            ctrl_interfaces_.joint_position_state_interface_[1].get().get_value(),
            target_pos_[2], ctrl_interfaces_.joint_position_command_interface_[2].get().get_value(),
            ctrl_interfaces_.joint_position_state_interface_[2].get().get_value());
    }

}

void StateFixedDown::exit()
{
    percent_ = 0;
}

FSMStateName StateFixedDown::checkChange()
{
    if (percent_ < 1.5)
    {
        if (ctrl_interfaces_.node)
        {
            RCLCPP_INFO_THROTTLE(
                ctrl_interfaces_.node->get_logger(),
                *ctrl_interfaces_.node->get_clock(),
                1000,
                "[StateFixedDown::checkChange] blocked percent=%.3f cmd=%d",
                percent_, ctrl_interfaces_.control_inputs_.command);
        }
        return FSMStateName::FIXEDDOWN;
    }

    if (ctrl_interfaces_.node)
    {
        RCLCPP_INFO_THROTTLE(
            ctrl_interfaces_.node->get_logger(),
            *ctrl_interfaces_.node->get_clock(),
            2000,
            "[StateFixedDown::checkChange] percent=%.3f cmd=%d",
            percent_, ctrl_interfaces_.control_inputs_.command);
    }
    switch (ctrl_interfaces_.control_inputs_.command)
    {
    case 1:
        return FSMStateName::PASSIVE;
    case 2:
        return FSMStateName::FIXEDSTAND;
    default:
        return FSMStateName::FIXEDDOWN;
    }
}
