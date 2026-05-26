//
// Created by tlab-uav on 24-9-19.
//

#include "leg_pd_controller/LegPdController.h"

namespace leg_pd_controller {
    using config_type = controller_interface::interface_configuration_type;

    controller_interface::CallbackReturn LegPdController::on_init() {
        try {
            joint_names_ = auto_declare<std::vector<std::string> >("joints", joint_names_);
            reference_interface_types_ =
                    auto_declare<std::vector<std::string> >("reference_interfaces", reference_interface_types_);
            state_interface_types_ = auto_declare<std::vector<
                std::string> >("state_interfaces", state_interface_types_);
        } catch (const std::exception &e) {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        const size_t joint_num = joint_names_.size();
        joint_effort_command_.assign(joint_num, 0);
        joint_position_command_.assign(joint_num, 0);
        joint_velocities_command_.assign(joint_num, 0);
        joint_kp_command_.assign(joint_num, 0);
        joint_kd_command_.assign(joint_num, 0);

        return CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration LegPdController::command_interface_configuration() const {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        conf.names.reserve(joint_names_.size());
        for (const auto &joint_name: joint_names_) {
            conf.names.push_back(joint_name + "/effort");
        }

        return conf;
    }

    controller_interface::InterfaceConfiguration LegPdController::state_interface_configuration() const {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};
        conf.names.reserve(joint_names_.size() * state_interface_types_.size());
        for (const auto &joint_name: joint_names_) {
            for (const auto &interface_type: state_interface_types_) {
                conf.names.push_back(joint_name + "/" += interface_type);
            }
        }
        return conf;
    }

    controller_interface::CallbackReturn LegPdController::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/) {
        reference_interfaces_.resize(joint_names_.size() * 5, std::numeric_limits<double>::quiet_NaN());
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn LegPdController::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/) {
        joint_effort_command_interface_.clear();
        joint_position_state_interface_.clear();
        joint_velocity_state_interface_.clear();

        // assign effort command interface
        for (auto &interface: command_interfaces_) {
            joint_effort_command_interface_.emplace_back(interface);
        }

        // assign state interfaces
        for (auto &interface: state_interfaces_) {
            state_interface_map_[interface.get_interface_name()]->push_back(interface);
        }

        RCLCPP_INFO(
            get_node()->get_logger(),
            "[LegPdController::on_activate] effort_cmd=%zu q_state=%zu dq_state=%zu",
            joint_effort_command_interface_.size(),
            joint_position_state_interface_.size(),
            joint_velocity_state_interface_.size());

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn LegPdController::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/) {
        release_interfaces();
        return CallbackReturn::SUCCESS;
    }

    bool LegPdController::on_set_chained_mode(bool /*chained_mode*/) {
        return true;
    }

    controller_interface::return_type LegPdController::update_and_write_commands(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
        if (joint_names_.size() != joint_effort_command_.size() ||
            joint_names_.size() != joint_kp_command_.size() ||
            joint_names_.size() != joint_position_command_.size() ||
            joint_names_.size() != joint_position_state_interface_.size() ||
            joint_names_.size() != joint_velocity_state_interface_.size() ||
            joint_names_.size() != joint_effort_command_interface_.size()) {
            std::cout << "joint_names_.size() = " << joint_names_.size() << std::endl;
            std::cout << "joint_effort_command_.size() = " << joint_effort_command_.size() << std::endl;
            std::cout << "joint_kp_command_.size() = " << joint_kp_command_.size() << std::endl;
            std::cout << "joint_position_command_.size() = " << joint_position_command_.size() << std::endl;
            std::cout << "joint_position_state_interface_.size() = " << joint_position_state_interface_.size() <<
                    std::endl;
            std::cout << "joint_velocity_state_interface_.size() = " << joint_velocity_state_interface_.size() <<
                    std::endl;
            std::cout << "joint_effort_command_interface_.size() = " << joint_effort_command_interface_.size() <<
                    std::endl;

            throw std::runtime_error("Mismatch in vector sizes in update_and_write_commands");
        }

        for (size_t i = 0; i < joint_names_.size(); ++i) {
            const double q_des = joint_position_command_[i];
            const double dq_des = joint_velocities_command_[i];
            const double tau_ff = joint_effort_command_[i];
            const double kp = joint_kp_command_[i];
            const double kd = joint_kd_command_[i];
            const double q = joint_position_state_interface_[i].get().get_value();
            const double dq = joint_velocity_state_interface_[i].get().get_value();

            const double p_term = kp * (q_des - q);
            const double d_term = kd * (dq_des - dq);
            const double torque = tau_ff + p_term + d_term;

            joint_effort_command_interface_[i].get().set_value(torque);

            if (i < 3) {
                RCLCPP_INFO_THROTTLE(
                    get_node()->get_logger(),
                    *get_node()->get_clock(),
                    2000,
                    "[LegPD] %s q_des=%.3f q=%.3f dq_des=%.3f dq=%.3f kp=%.1f kd=%.1f ff=%.3f p=%.3f d=%.3f tau=%.3f",
                    joint_names_[i].c_str(),
                    q_des, q, dq_des, dq, kp, kd, tau_ff, p_term, d_term, torque);
            }
        }
        return controller_interface::return_type::OK;
    }

    std::vector<hardware_interface::CommandInterface> LegPdController::on_export_reference_interfaces() {
        std::vector<hardware_interface::CommandInterface> reference_interfaces;

        int ind = 0;
        std::string controller_name = get_node()->get_name();
        for (const auto &joint_name: joint_names_) {
            std::cout << joint_name << std::endl;
            reference_interfaces.emplace_back(controller_name, joint_name + "/position", &joint_position_command_[ind]);
            reference_interfaces.emplace_back(controller_name, joint_name + "/velocity",
                                              &joint_velocities_command_[ind]);
            reference_interfaces.emplace_back(controller_name, joint_name + "/effort", &joint_effort_command_[ind]);
            reference_interfaces.emplace_back(controller_name, joint_name + "/kp", &joint_kp_command_[ind]);
            reference_interfaces.emplace_back(controller_name, joint_name + "/kd", &joint_kd_command_[ind]);
            ind++;
        }

        return reference_interfaces;
    }

#ifdef ROS2_CONTROL_VERSION_LT_3
    controller_interface::return_type LegPdController::update_reference_from_subscribers() {
        return controller_interface::return_type::OK;
    }
#else
    controller_interface::return_type LegPdController::update_reference_from_subscribers(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
        return controller_interface::return_type::OK;
    }
#endif
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(leg_pd_controller::LegPdController, controller_interface::ChainableControllerInterface);
