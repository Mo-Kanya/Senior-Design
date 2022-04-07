//
// Created by Shen Zheng on 2022.
//

#include "Fix_route_decision.h"

decision_maker::DecisionThd decision_maker::decision_thd;
float decision_maker::target_theta, target_vx, target_vy, cur_vx, cur_vy, angle;
uint8_t decision_maker::mode;


void decision_maker::init(tprio_t decision_maker_prio_) {
    decision_thd.start(decision_maker_prio_);
}

void decision_maker::compute_v(float FR, float FL, float BL, float BR) {
    
} 

void decision_maker::DecisionThd::main() {
    setName("Decision_maker");
    while(!shouldTerminate()) {

        float motor_v1 = ChassisSKD::get_actual_velocity(ChassisSKD::FR); // degree/s
        float motor_v2 = ChassisSKD::get_actual_velocity(ChassisSKD::FL);
        float motor_v3 = ChassisSKD::get_actual_velocity(ChassisSKD::BL);
        float motor_v4 = ChassisSKD::get_actual_velocity(ChassisSKD::BR);

        decision_maker::compute_v(motor_v1, motor_v2, motor_v3, motor_v4);

        angle = ChassisSKD::get_last_angle();
        mode = UserI::get_mode();


    }
}
