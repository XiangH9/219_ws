//
// Created by biao on 24-9-12.
//

#include <iostream>
#include "unitree_guide_controller/FSM/StateSwingTest.h"

#include <unitree_guide_controller/control/CtrlComponent.h>

#include "unitree_guide_controller/common/mathTools.h"

StateSwingTest::StateSwingTest(CtrlInterfaces &ctrl_interfaces,
                            CtrlComponent &ctrl_component)
    : FSMState(
          FSMStateName::SWINGTEST, "swing test",
          ctrl_interfaces),
      robot_model_(ctrl_component.robot_model_) {
    _xMin = -0.15;
    _xMax = 0.10;
    _yMin = -0.15;
    _yMax = 0.15;
    _zMin = -0.05;
    _zMax = 0.20;
}

void StateSwingTest::enter() {
    for (int i = 0; i < 3; i++) {
        ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(3);
        ctrl_interfaces_.joint_kd_command_interface_[i].get().set_value(2);
    }
    for (int i = 3; i < 12; i++) {
        ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(180);
        ctrl_interfaces_.joint_kd_command_interface_[i].get().set_value(5);
    }

    Kp = KDL::Vector(20, 20, 50);
    Kd = KDL::Vector(5, 5, 20);

    init_joint_pos_ = robot_model_->current_joint_pos_;
    init_foot_pos_ = robot_model_->getFeet2BPositions();

    target_foot_pos_ = init_foot_pos_;
    fr_init_pos_ = init_foot_pos_[0];
    fr_goal_pos_ = fr_init_pos_;

    RCLCPP_INFO(
        ctrl_interfaces_.node->get_logger(),
        "[SwingTest::enter] FR init_B=(%.3f %.3f %.3f) range_x=[%.3f, %.3f] range_y=[%.3f, %.3f] "
        "range_z=[%.3f, %.3f] Kp=(%.1f %.1f %.1f) Kd=(%.1f %.1f %.1f)",
        fr_init_pos_.p.x(), fr_init_pos_.p.y(), fr_init_pos_.p.z(),
        _xMin, _xMax, _yMin, _yMax, _zMin, _zMax,
        Kp.x(), Kp.y(), Kp.z(),
        Kd.x(), Kd.y(), Kd.z());
}

void StateSwingTest::run(const rclcpp::Time &/*time*/, const rclcpp::Duration &/*period*/) {
    if (ctrl_interfaces_.control_inputs_.ly > 0) {
        fr_goal_pos_.p.x(invNormalize(ctrl_interfaces_.control_inputs_.ly, fr_init_pos_.p.x(),
                                      fr_init_pos_.p.x() + _xMax, 0, 1));
    } else {
        fr_goal_pos_.p.x(invNormalize(ctrl_interfaces_.control_inputs_.ly, fr_init_pos_.p.x() + _xMin,
                                      fr_init_pos_.p.x(), -1, 0));
    }
    if (ctrl_interfaces_.control_inputs_.lx > 0) {
        fr_goal_pos_.p.y(invNormalize(ctrl_interfaces_.control_inputs_.lx, fr_init_pos_.p.y(),
                                      fr_init_pos_.p.y() + _yMax, 0, 1));
    } else {
        fr_goal_pos_.p.y(invNormalize(ctrl_interfaces_.control_inputs_.lx, fr_init_pos_.p.y() + _yMin,
                                      fr_init_pos_.p.y(), -1, 0));
    }
    if (ctrl_interfaces_.control_inputs_.ry > 0) {
        fr_goal_pos_.p.z(invNormalize(ctrl_interfaces_.control_inputs_.ry, fr_init_pos_.p.z(),
                                      fr_init_pos_.p.z() + _zMax, 0, 1));
    } else {
        fr_goal_pos_.p.z(invNormalize(ctrl_interfaces_.control_inputs_.ry, fr_init_pos_.p.z() + _zMin,
                                      fr_init_pos_.p.z(), -1, 0));
    }

    positionCtrl();
    torqueCtrl();

    const KDL::Frame fr_current_pos = robot_model_->getFeet2BPositions(0);
    const KDL::Vector fr_current_vel = robot_model_->getFeet2BVelocities(0);
    const KDL::Vector pos_err = fr_goal_pos_.p - fr_current_pos.p;

    RCLCPP_INFO_THROTTLE(
        ctrl_interfaces_.node->get_logger(),
        *ctrl_interfaces_.node->get_clock(),
        100,
        "[SwingTest::foot] cmd(lx=%.2f ly=%.2f ry=%.2f) goal_B=(%.3f %.3f %.3f) actual_B=(%.3f %.3f %.3f) "
        "pos_err=(%.3f %.3f %.3f) vel_B=(%.3f %.3f %.3f)",
        ctrl_interfaces_.control_inputs_.lx,
        ctrl_interfaces_.control_inputs_.ly,
        ctrl_interfaces_.control_inputs_.ry,
        fr_goal_pos_.p.x(), fr_goal_pos_.p.y(), fr_goal_pos_.p.z(),
        fr_current_pos.p.x(), fr_current_pos.p.y(), fr_current_pos.p.z(),
        pos_err.x(), pos_err.y(), pos_err.z(),
        fr_current_vel.x(), fr_current_vel.y(), fr_current_vel.z());
}

void StateSwingTest::exit() {
}

FSMStateName StateSwingTest::checkChange() {
    switch (ctrl_interfaces_.control_inputs_.command) {
        case 1:
            return FSMStateName::PASSIVE;
        case 2:
            return FSMStateName::FIXEDSTAND;
        default:
            return FSMStateName::SWINGTEST;
    }
}

void StateSwingTest::positionCtrl() {
    target_foot_pos_[0] = fr_goal_pos_;
    target_joint_pos_ = robot_model_->getQ(target_foot_pos_);
    for (int i = 0; i < 4; i++) {
        ctrl_interfaces_.joint_position_command_interface_[i * 3].get().set_value(target_joint_pos_[i](0));
        ctrl_interfaces_.joint_position_command_interface_[i * 3 + 1].get().set_value(target_joint_pos_[i](1));
        ctrl_interfaces_.joint_position_command_interface_[i * 3 + 2].get().set_value(target_joint_pos_[i](2));
    }
}

void StateSwingTest::torqueCtrl() const {
    const KDL::Frame fr_current_pos = robot_model_->getFeet2BPositions(0);

    const KDL::Vector pos_goal = fr_goal_pos_.p;
    const KDL::Vector pos0 = fr_current_pos.p;
    const KDL::Vector vel0 = robot_model_->getFeet2BVelocities(0);

    const KDL::Vector force0 = Kp * (pos_goal - pos0) + Kd * -vel0;
    KDL::JntArray torque0 = robot_model_->getTorque(force0, 0);

    for (int i = 0; i < 3; i++) {
        ctrl_interfaces_.joint_torque_command_interface_[i].get().set_value(torque0(i));
    }

    const KDL::JntArray &q_now = robot_model_->current_joint_pos_[0];
    const KDL::JntArray &q_goal = target_joint_pos_[0];

    RCLCPP_INFO_THROTTLE(
        ctrl_interfaces_.node->get_logger(),
        *ctrl_interfaces_.node->get_clock(),
        100,
        "[SwingTest::joint] FR goal(q=%.3f %.3f %.3f) state(q=%.3f %.3f %.3f) "
        "force_B=(%.3f %.3f %.3f) tau=(%.3f %.3f %.3f)",
        q_goal(0), q_goal(1), q_goal(2),
        q_now(0), q_now(1), q_now(2),
        force0.x(), force0.y(), force0.z(),
        torque0(0), torque0(1), torque0(2));
}
