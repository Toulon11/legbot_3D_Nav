//
// Created by biao on 24-9-10.
//

#include "rl_quadruped_controller/FSM/StateFixedStand.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

StateFixedStand::StateFixedStand(CtrlInterfaces &ctrl_interfaces, const std::vector<double> &target_pos,
                                 const double kp,
                                 const double kd,
                                 const double transition_seconds)
    : BaseFixedStand(ctrl_interfaces, target_pos, kp, kd),
      transition_seconds_(transition_seconds) {
    if (!std::isfinite(transition_seconds_) || transition_seconds_ <= 0.0)
        throw std::invalid_argument("stand_transition_seconds must be finite and positive");
}

void StateFixedStand::enter() {
    elapsed_seconds_ = 0.0;
    pending_walk_ = false;
    down_armed_ = false;
    const auto read = [](const auto& interface, double fallback) {
        const auto value = interface.get().get_optional();
        return value && std::isfinite(*value) ? *value : fallback;
    };
    for (int i = 0; i < 12; ++i) {
        // Preserve the complete last actuator command, including the position
        // error supplying support torque. Replacing q with measured q would
        // remove that torque even when the body has already stopped moving.
        const double measured = read(ctrl_interfaces_.joint_position_state_interface_[i], target_pos_[i]);
        initial_[i] = {
            read(ctrl_interfaces_.joint_position_command_interface_[i], measured),
            read(ctrl_interfaces_.joint_velocity_command_interface_[i], 0.0),
            std::max(0.0, read(ctrl_interfaces_.joint_kp_command_interface_[i], kp_)),
            std::max(0.0, read(ctrl_interfaces_.joint_kd_command_interface_[i], kd_)),
            read(ctrl_interfaces_.joint_torque_command_interface_[i], 0.0)};
    }
    ctrl_interfaces_.control_inputs_.command = 0;
}

void StateFixedStand::run(const rclcpp::Time&, const rclcpp::Duration& period) {
    const double dt = period.seconds();
    if (std::isfinite(dt) && dt > 0.0)
        elapsed_seconds_ = std::min(transition_seconds_, elapsed_seconds_ + dt);
    const double u = elapsed_seconds_ / transition_seconds_;
    // Quintic easing has zero first/second derivatives at both endpoints.
    const double weight = u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
    const auto blend = [weight](double a, double b) { return a + weight * (b - a); };
    for (int i = 0; i < 12; ++i) {
        const auto& c = initial_[i];
        std::ignore = ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(blend(c.q, target_pos_[i]));
        std::ignore = ctrl_interfaces_.joint_velocity_command_interface_[i].get().set_value(blend(c.dq, 0.0));
        std::ignore = ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(blend(c.kp, kp_));
        std::ignore = ctrl_interfaces_.joint_kd_command_interface_[i].get().set_value(blend(c.kd, kd_));
        std::ignore = ctrl_interfaces_.joint_torque_command_interface_[i].get().set_value(blend(c.tau, 0.0));
    }
}

void StateFixedStand::exit() {
    elapsed_seconds_ = 0.0;
    pending_walk_ = false;
}

FSMStateName StateFixedStand::checkChange() {
    const int command = ctrl_interfaces_.control_inputs_.command;
    if (command == 1) return FSMStateName::PASSIVE;
    if (command == 3) pending_walk_ = true;
    if (command != 2) down_armed_ = true;
    if (elapsed_seconds_ < transition_seconds_) {
        return FSMStateName::FIXEDSTAND;
    }
    // A short mode-3 pulse may end before the stand transition completes.
    if (pending_walk_) return FSMStateName::RL;
    // The mode-2 pulse which entered stand must not toggle it into crouch.
    if (command == 2 && down_armed_) return FSMStateName::FIXEDDOWN;
    return FSMStateName::FIXEDSTAND;
}
