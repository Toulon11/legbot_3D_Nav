//
// Created by biao on 24-9-10.
//

#ifndef STATEFIXEDSTAND_H
#define STATEFIXEDSTAND_H

#include <controller_common/FSM/BaseFixedStand.h>
#include <array>

class StateFixedStand final : public BaseFixedStand {
public:
    explicit StateFixedStand(CtrlInterfaces &ctrl_interfaces,
                             const std::vector<double> &target_pos,
                             double kp,
                             double kd,
                             double transition_seconds = 1.5);

    void enter() override;
    void run(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    void exit() override;
    FSMStateName checkChange() override;

private:
    struct JointCommand {
        double q, dq, kp, kd, tau;
    };
    std::array<JointCommand, 12> initial_{};
    double transition_seconds_;
    double elapsed_seconds_ = 0.0;
    bool pending_walk_ = false;
    bool down_armed_ = false;
};


#endif //STATEFIXEDSTAND_H
