//
// Created by liuzikai on 2019-06-15.
//

/**
 * @file    chassis_logic.cpp
 * @brief   Generate target values for ChassisSKD. Support follow-gimbal mode and dodge mode.
 *
 * @addtogroup chassis
 * @{
 */

#include "chassis_logic.h"
#include "chassis_scheduler.h"
#include "remote_interpreter.h"

ChassisLG::action_t ChassisLG::action = FORCED_RELAX_MODE;
float ChassisLG::target_vx;
float ChassisLG::target_vy;
float ChassisLG::target_theta;

void ChassisLG::init() {
}

ChassisLG::action_t ChassisLG::get_action() {
    return action;
}

void ChassisLG::set_action(ChassisLG::action_t value) {
    if (value == action) return;  // avoid repeating setting target_theta, etc.

    action = value;
    if (action == FORCED_RELAX_MODE) {
        ChassisSKD::load_pid_params(CHASSIS_FOLLOW_PID_THETA2V_PARAMS, CHASSIS_PID_V2I_PARAMS);
        ChassisSKD::set_mode(ChassisSKD::FORCED_RELAX_MODE);
    } else if (action == FOLLOW_MODE) {
        ChassisSKD::load_pid_params(CHASSIS_FOLLOW_PID_THETA2V_PARAMS, CHASSIS_PID_V2I_PARAMS);
        ChassisSKD::set_mode(ChassisSKD::GIMBAL_COORDINATE_MODE);
        apply_target();
    }
    // Sending client data will be complete by higher level thread
}

void ChassisLG::set_target(float vx, float vy) {
    target_vx = vx;
    target_vy = vy;
    if (action == FOLLOW_MODE) {
        target_theta = 0.0f;
    }
    // For DODGE_MODE keep current target_theta unchanged
    apply_target();
}

void ChassisLG::apply_target() {
    if (action == FOLLOW_MODE) {
        ChassisSKD::set_target(target_vx, target_vy, target_theta);
    }
}

//GimbalLG::VisionControlThread GimbalLG::vision_control_thread;
//void GimbalLG::init(tprio_t vision_control_thread_prio, tprio_t sentry_control_thread_prio, float pitch_min_angle_,
//                    float pitch_max_angle_, float sub_pitch_min_angle_, float sub_pitch_max_angle_) {
//    if (vision_control_thread_prio != IDLEPRIO) {
//        vision_control_thread.start(vision_control_thread_prio);
//    }
//}
//void GimbalLG::VisionControlThread::main() {
//    setName("GimbalLG_Vision");
//    chEvtRegisterMask(&Vision::gimbal_updated_event, &vision_listener, EVENT_MASK(0));
//
//    while (!shouldTerminate()) {
//        chEvtWaitAny(ALL_EVENTS);
//        if (action == VISION_MODE) {
//            float yaw, pitch;
//            bool can_reach_the_target;
//
//            chSysLock();  /// --- ENTER S-Locked state. DO NOT use LOG, printf, non S/I-Class functions or return ---
//            {
//                can_reach_the_target = Vision::get_gimbal_target_angles(yaw, pitch);
//            }
//            chSysUnlock();  /// --- EXIT S-Locked state ---
//
//            if (can_reach_the_target) {
//                VAL_CROP(pitch, PITCH_MAX_ANGLE, PITCH_MIN_ANGLE);
//                GimbalSKD::set_target_angle(yaw, pitch);
//            }  // otherwise, keep current target angles
//        }
//    }
//}
/** @} */