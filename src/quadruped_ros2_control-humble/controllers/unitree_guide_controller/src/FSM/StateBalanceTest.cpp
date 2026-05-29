//
// Created by tlab-uav on 24-9-16.
//

#include "unitree_guide_controller/FSM/StateBalanceTest.h"

#include <unitree_guide_controller/UnitreeGuideController.h>

#include "unitree_guide_controller/common/mathTools.h"
#include "unitree_guide_controller/gait/WaveGenerator.h"

StateBalanceTest::StateBalanceTest(CtrlInterfaces &ctrl_interfaces,
                                   CtrlComponent &ctrl_component)
    : FSMState(FSMStateName::BALANCETEST,
               "balance test",
               ctrl_interfaces),
      estimator_(ctrl_component.estimator_),
      robot_model_(ctrl_component.robot_model_),
      balance_ctrl_(ctrl_component.balance_ctrl_),
      wave_generator_(
          ctrl_component.wave_generator_) {
    _xMax = 0.05;
    _xMin = -_xMax;
    _yMax = 0.05;
    _yMin = -_yMax;
    _zMax = 0.04;
    _zMin = -_zMax;
    _yawMax = 20 * M_PI / 180;
    _yawMin = -_yawMax;

    Kp_p_ = Vec3(150, 150, 150).asDiagonal();
    Kd_p_ = Vec3(25, 25, 25).asDiagonal();

    // Use a softer attitude loop first so we can inspect whether large d_wbd
    // comes from true pose error or from overly aggressive gains.
    kp_w_ = 120;
    Kd_w_ = Vec3(12, 12, 12).asDiagonal();
}

void StateBalanceTest::enter() {
    pcd_init_ = estimator_->getPosition();
    pcd_ = pcd_init_;
    init_rotation_ = estimator_->getRotation();
    wave_generator_->status_ = WaveStatus::STANCE_ALL;
    stable_joint_sum_.setZero();
    stable_sample_count_ = 0;
    stable_reported_ = false;

    const Vec34 feet_body = estimator_->getFeetPos2Body();
    RCLCPP_INFO(
        ctrl_interfaces_.node->get_logger(),
        "[BalanceTest::enter] pcd_init=(%.3f %.3f %.3f) feet_B FR=(%.3f %.3f %.3f) FL=(%.3f %.3f %.3f) RR=(%.3f %.3f %.3f) RL=(%.3f %.3f %.3f)",
        pcd_init_(0), pcd_init_(1), pcd_init_(2),
        feet_body(0, 0), feet_body(1, 0), feet_body(2, 0),
        feet_body(0, 1), feet_body(1, 1), feet_body(2, 1),
        feet_body(0, 2), feet_body(1, 2), feet_body(2, 2),
        feet_body(0, 3), feet_body(1, 3), feet_body(2, 3));
}

void StateBalanceTest::run(const rclcpp::Time &/*time*/, const rclcpp::Duration &/*period*/) {
    pcd_(0) = pcd_init_(0) + invNormalize(ctrl_interfaces_.control_inputs_.ly, _xMin, _xMax);
    pcd_(1) = pcd_init_(1) - invNormalize(ctrl_interfaces_.control_inputs_.lx, _yMin, _yMax);
    pcd_(2) = pcd_init_(2) + invNormalize(ctrl_interfaces_.control_inputs_.ry, _zMin, _zMax);

    const float yaw = -invNormalize(ctrl_interfaces_.control_inputs_.rx, _yawMin, _yawMax);
    Rd_ = rotz(yaw) * init_rotation_;

    for (int i = 0; i < 12; i++) {
        ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(0.8);
        ctrl_interfaces_.joint_kd_command_interface_[i].get().set_value(0.8);
    }

    calcTorque();
}

void StateBalanceTest::exit() {
    if (stable_sample_count_ > 0) {
        const Vec12 q_avg = stable_joint_sum_ / static_cast<double>(stable_sample_count_);
        RCLCPP_INFO(
            ctrl_interfaces_.node->get_logger(),
            "[BalanceTest::stand-candidate-exit] samples=%d FR=(%.4f %.4f %.4f) FL=(%.4f %.4f %.4f) RR=(%.4f %.4f %.4f) RL=(%.4f %.4f %.4f)",
            stable_sample_count_,
            q_avg(0), q_avg(1), q_avg(2),
            q_avg(3), q_avg(4), q_avg(5),
            q_avg(6), q_avg(7), q_avg(8),
            q_avg(9), q_avg(10), q_avg(11));
    } else {
        RCLCPP_WARN(
            ctrl_interfaces_.node->get_logger(),
            "[BalanceTest::stand-candidate-exit] no stable samples captured before exit");
    }
    wave_generator_->status_ = WaveStatus::SWING_ALL;
}

FSMStateName StateBalanceTest::checkChange() {
    switch (ctrl_interfaces_.control_inputs_.command) {
        case 1:
            return FSMStateName::FIXEDDOWN;
        case 2:
            return FSMStateName::FIXEDSTAND;
        default:
            return FSMStateName::BALANCETEST;
    }
}

void StateBalanceTest::calcTorque() {
    const auto B2G_Rotation = estimator_->getRotation();
    const RotMat G2B_Rotation = B2G_Rotation.transpose();

    const Vec3 pose_body = estimator_->getPosition();
    const Vec3 vel_body = estimator_->getVelocity();
    const Vec3 gyro_global = estimator_->getGyroGlobal();
    const Vec3 rot_err = rotMatToExp(Rd_ * G2B_Rotation);

    // expected body acceleration
    dd_pcd_ = Kp_p_ * (pcd_ - pose_body) + Kd_p_ * (Vec3(0, 0, 0) - vel_body);

    // expected body angular acceleration
    d_wbd_ = kp_w_ * rot_err +
             Kd_w_ * (Vec3(0, 0, 0) - gyro_global);

    // calculate foot force
    const Vec34 pos_feet_2_body_global = estimator_->getFeetPos2Body();
    const Vec34 force_feet_global = -balance_ctrl_->calF(dd_pcd_, d_wbd_, B2G_Rotation,
                                                         pos_feet_2_body_global, wave_generator_->contact_);
    const Vec34 force_feet_body = G2B_Rotation * force_feet_global;

    const Vec3 pos_err = pcd_ - pose_body;
    const Vec3 vel_err = Vec3(0, 0, 0) - vel_body;
    const double sum_fz = force_feet_global.row(2).sum();
    const double front_fz = force_feet_global(2, 0) + force_feet_global(2, 1);
    const double rear_fz = force_feet_global(2, 2) + force_feet_global(2, 3);
    const double left_fz = force_feet_global(2, 1) + force_feet_global(2, 3);
    const double right_fz = force_feet_global(2, 0) + force_feet_global(2, 2);

    const bool zero_cmd =
        std::abs(ctrl_interfaces_.control_inputs_.lx) < 1e-3 &&
        std::abs(ctrl_interfaces_.control_inputs_.ly) < 1e-3 &&
        std::abs(ctrl_interfaces_.control_inputs_.rx) < 1e-3 &&
        std::abs(ctrl_interfaces_.control_inputs_.ry) < 1e-3;
    const bool stable_now =
        zero_cmd &&
        rot_err.norm() < 0.12 &&
        gyro_global.norm() < 0.03 &&
        vel_body.norm() < 0.05 &&
        dd_pcd_.norm() < 1.0 &&
        std::abs(sum_fz + 148.0) < 8.0;

    if (stable_now) {
        Vec12 q_now;
        for (int leg = 0; leg < 4; ++leg) {
            q_now.segment<3>(leg * 3) = robot_model_->current_joint_pos_[leg].data;
        }
        stable_joint_sum_ += q_now;
        stable_sample_count_++;
    } else {
        stable_joint_sum_.setZero();
        stable_sample_count_ = 0;
        stable_reported_ = false;
    }

    RCLCPP_INFO_THROTTLE(
        ctrl_interfaces_.node->get_logger(),
        *ctrl_interfaces_.node->get_clock(),
        200,
        "[BalanceTest::body] cmd(lx=%.2f ly=%.2f rx=%.2f ry=%.2f) pos_err=(%.3f %.3f %.3f) vel_err=(%.3f %.3f %.3f) rot_err=(%.3f %.3f %.3f) gyro_G=(%.3f %.3f %.3f) dd_pcd=(%.3f %.3f %.3f) d_wbd=(%.3f %.3f %.3f)",
        ctrl_interfaces_.control_inputs_.lx,
        ctrl_interfaces_.control_inputs_.ly,
        ctrl_interfaces_.control_inputs_.rx,
        ctrl_interfaces_.control_inputs_.ry,
        pos_err(0), pos_err(1), pos_err(2),
        vel_err(0), vel_err(1), vel_err(2),
        rot_err(0), rot_err(1), rot_err(2),
        gyro_global(0), gyro_global(1), gyro_global(2),
        dd_pcd_(0), dd_pcd_(1), dd_pcd_(2),
        d_wbd_(0), d_wbd_(1), d_wbd_(2));

    RCLCPP_INFO_THROTTLE(
        ctrl_interfaces_.node->get_logger(),
        *ctrl_interfaces_.node->get_clock(),
        200,
        "[BalanceTest::force] FR=(%.2f %.2f %.2f) FL=(%.2f %.2f %.2f) RR=(%.2f %.2f %.2f) RL=(%.2f %.2f %.2f) sum_fz=%.2f front=%.2f rear=%.2f left=%.2f right=%.2f",
        force_feet_global(0, 0), force_feet_global(1, 0), force_feet_global(2, 0),
        force_feet_global(0, 1), force_feet_global(1, 1), force_feet_global(2, 1),
        force_feet_global(0, 2), force_feet_global(1, 2), force_feet_global(2, 2),
        force_feet_global(0, 3), force_feet_global(1, 3), force_feet_global(2, 3),
        sum_fz, front_fz, rear_fz, left_fz, right_fz);

    if (stable_sample_count_ >= 20 && !stable_reported_) {
        const Vec12 q_avg = stable_joint_sum_ / static_cast<double>(stable_sample_count_);
        RCLCPP_INFO(
            ctrl_interfaces_.node->get_logger(),
            "[BalanceTest::stand-candidate] samples=%d FR=(%.4f %.4f %.4f) FL=(%.4f %.4f %.4f) RR=(%.4f %.4f %.4f) RL=(%.4f %.4f %.4f)",
            stable_sample_count_,
            q_avg(0), q_avg(1), q_avg(2),
            q_avg(3), q_avg(4), q_avg(5),
            q_avg(6), q_avg(7), q_avg(8),
            q_avg(9), q_avg(10), q_avg(11));
        stable_reported_ = true;
    }

    std::vector<KDL::JntArray> current_joints = robot_model_->current_joint_pos_;
    for (int i = 0; i < 4; i++) {
        KDL::JntArray torque = robot_model_->getTorque(force_feet_body.col(i), i);
        for (int j = 0; j < 3; j++) {
            ctrl_interfaces_.joint_torque_command_interface_[i * 3 + j].get().set_value(torque(j));
            ctrl_interfaces_.joint_position_command_interface_[i * 3 + j].get().set_value(
                current_joints[i](j));
        }
    }
}
