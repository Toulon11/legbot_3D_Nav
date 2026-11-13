#include "rl_quadruped_controller/FSM/StateFixedStand.h"
#include <array>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>

void require(bool ok, const char* reason) {
    if (!ok) throw std::runtime_error(reason);
}
void near(double a, double b) { require(std::abs(a-b)<1e-9, "joint command mismatch"); }

struct Fixture {
    CtrlInterfaces ctrl;
    std::array<std::array<double, 5>, 12> values{};
    std::array<double, 12> measured{};
    std::vector<hardware_interface::CommandInterface::SharedPtr> commands;
    std::vector<hardware_interface::StateInterface::SharedPtr> states;
    std::vector<hardware_interface::LoanedCommandInterface> loaned;
    std::vector<hardware_interface::LoanedStateInterface> borrowed;
    Fixture() {
        ctrl.frequency_ = 200;
        commands.reserve(60); loaned.reserve(60); states.reserve(12); borrowed.reserve(12);
        for (int i=0; i<12; ++i) {
            values[i]={0.4, 0.1, 20., 0.5, 1.2}; measured[i]=-0.2;
            const auto name="joint"+std::to_string(i);
            const std::array<const char*,5> names{"position","velocity","kp","kd","effort"};
            for (int f=0;f<5;++f) {
                commands.push_back(std::make_shared<hardware_interface::CommandInterface>(name,names[f],&values[i][f]));
                loaned.emplace_back(commands.back(),nullptr);
            }
            ctrl.joint_position_command_interface_.push_back(loaned[5*i]);
            ctrl.joint_velocity_command_interface_.push_back(loaned[5*i+1]);
            ctrl.joint_kp_command_interface_.push_back(loaned[5*i+2]);
            ctrl.joint_kd_command_interface_.push_back(loaned[5*i+3]);
            ctrl.joint_torque_command_interface_.push_back(loaned[5*i+4]);
            states.push_back(std::make_shared<hardware_interface::StateInterface>(name,"position",&measured[i]));
            borrowed.emplace_back(states.back());
            ctrl.joint_position_state_interface_.push_back(borrowed.back());
        }
    }
};
int main() {
    try {
        Fixture f;
        StateFixedStand stand(f.ctrl,std::vector<double>(12,0.9),80.,3.5,1.0);
        const auto initial=f.values;
        stand.enter();
        require(initial==f.values,"enter changed the last RL actuator command");
        // Zero elapsed time preserves q/dq/gains/feedforward, not measured q.
        stand.run(rclcpp::Time(0),rclcpp::Duration::from_seconds(0.0));
        require(initial==f.values,"zero-time transition discontinuity");
        stand.run(rclcpp::Time(0),rclcpp::Duration::from_seconds(0.2));
        stand.run(rclcpp::Time(0),rclcpp::Duration::from_seconds(0.3));
        for(const auto& j:f.values) {
            near(j[0],0.65); near(j[1],0.05); near(j[2],50.); near(j[3],2.); near(j[4],0.6);
        }
        // A pending short walking pulse must survive the remaining transition.
        f.ctrl.control_inputs_.command=3;
        require(stand.checkChange()==FSMStateName::FIXEDSTAND,"walk interrupted stand interpolation");
        f.ctrl.control_inputs_.command=0;
        stand.run(rclcpp::Time(0),rclcpp::Duration::from_seconds(0.5));
        require(stand.checkChange()==FSMStateName::RL,"lost pending walking pulse");
        for(const auto& j:f.values) {
            near(j[0],0.9); near(j[1],0.); near(j[2],80.); near(j[3],3.5); near(j[4],0.);
        }
        stand.exit(); stand.enter();
        f.ctrl.control_inputs_.command=2;
        stand.run(rclcpp::Time(0),rclcpp::Duration::from_seconds(2.0));
        require(stand.checkChange()==FSMStateName::FIXEDSTAND,"held stand pulse toggled crouch");
        f.ctrl.control_inputs_.command=0; stand.checkChange();
        f.ctrl.control_inputs_.command=2;
        require(stand.checkChange()==FSMStateName::FIXEDDOWN,"fresh crouch request ignored");
        stand.exit(); stand.enter();
        f.ctrl.control_inputs_.command=1;
        require(stand.checkChange()==FSMStateName::PASSIVE,"passive request delayed by transition");
        bool rejected=false;
        try { StateFixedStand bad(f.ctrl,std::vector<double>(12,0.9),80.,3.5,0.0); }
        catch(const std::invalid_argument&) {rejected=true;}
        require(rejected,"zero transition duration accepted");
        std::cout<<"GO2 stand command continuity, timed blend and mode requests passed\n";
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
