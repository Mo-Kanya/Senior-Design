//
// Created by liuzikai on 2018-12-29.
// Zhu Kerui wrote code about processing gimbal feedback.
// Feng Chuhao wrote code about sending gimbal currents.
// Mo Kanya wrote code about sending friction wheels' currents and processing friction wheels' feedback
// Qian Chen wrote about motor can channel distribution mechanism.
//

/**
 * @file    gimbal_interface.cpp
 * @brief   Interface to interact with low level driver of gimbal, including processing chassis motor feedback and
 *          sending target currents.
 *
 * @addtogroup gimbal
 * @{
 */

#include "gimbal_interface.h"
#include "common_macro.h"

CANInterface::motor_feedback_t *GimbalIF::feedback[MOTOR_COUNT];
float *GimbalIF::lidar_dist = nullptr;
int *GimbalIF::target_current[MOTOR_COUNT];
CANInterface *GimbalIF::can1_ = nullptr;
CANInterface *GimbalIF::can2_ = nullptr;

void GimbalIF::init(CANInterface *can1_interface, CANInterface *can2_interface,
                    motor_can_config_t motor_can_config[MOTOR_COUNT],
                    uint16_t yaw_front_angle_raw, uint16_t pitch_front_angle_raw, uint16_t sub_pitch_front_angle_raw, motor_can_channel_t lidar_channel) {

    // Get the CAN address.
    can1_ = can1_interface;
    can2_ = can2_interface;

    /*
     * Link the feedback to feedback info stored in CANInterface;
     * The feedback[i] is the i'th motor to receive feedback and it needs an address of the real motor feedback;
     * The i is between [YAW,FW_RIGHT] and gimbal::init will receive the motor_can_config which contains{can channel, can's id, motor type };
     * There are 6 motors in gimbal, so the can_config array passed in should contains 6 elements;
     * There are two cans being used, so the i'th motor in the gimbal should first judge which can does it belong to:
     *          motor_can_config[i].motor_can_channel == can_channel_1;
     * After knowing which can does the i'th motor belongs to, this motor's feedback array should be linked to the data in the can Interface,
     * because the real feedback data was stored in the processFeedbackThread.feedback[] and the feedback array stored in the gimbal interface was an array of pointers;
     * Then the i'th motor should use the code below to get the address:
     *          can1_->get_feedback_address(motor_can_config[i].motor_can_id);
     * or       can2_->register_feedback_address(motor_can_config[i].motor_can_id);
     * Note the function return:
     *          return &processFeedbackThread.feedback[id];
     * Note that the i'th motor has the i'th motor_can_config value which was loaded on the motor_can_id'th position of the corresponding can wire;
     */
    for (int i = 0; i < MOTOR_COUNT; i++) {

        if (motor_can_config[i].motor_can_channel == can_channel_1) {

            // Link the feedback and target_current to can1's feedback
            feedback[i] = can1_->register_feedback_address(motor_can_config[i].motor_can_id, motor_can_config[i].motor_type, motor_can_config[i].deceleration_ratio);
            if (feedback[i] == nullptr) while(true) LOG_ERR("GimbalIF: failed to register motor %d feedback", i);
            target_current[i] = can1_->register_target_current_address(motor_can_config[i].motor_can_id, motor_can_config[i].motor_type);
            if (target_current[i] == nullptr) while(true) LOG_ERR("GimbalIF: failed to register motor %d target_current", i);
            *target_current[i] = 0;

        } else if (motor_can_config[i].motor_can_channel == can_channel_2) {

            // Link the feedback and target_current to can2's feedback
            feedback[i] = can2_->register_feedback_address(motor_can_config[i].motor_can_id, motor_can_config[i].motor_type, motor_can_config[i].deceleration_ratio);
            if (feedback[i] == nullptr) while(true) LOG_ERR("GimbalIF: failed to register motor %d feedback", i);
            target_current[i] = can2_->register_target_current_address(motor_can_config[i].motor_can_id, motor_can_config[i].motor_type);
            if (target_current[i] == nullptr) while(true) LOG_ERR("GimbalIF: failed to register motor %d target_current", i);
            *target_current[i] = 0;

        }

        // Set front angle of yaw and pitch
        if (YAW == (motor_id_t) i) {
            feedback[i]->last_angle_raw = yaw_front_angle_raw;
        } else if (PITCH == (motor_id_t) i) {
            feedback[i]->last_angle_raw = pitch_front_angle_raw;
        } else if (SUB_PITCH == (motor_id_t) i) {
            feedback[i]->last_angle_raw = sub_pitch_front_angle_raw;
        }

    }

    if (lidar_channel == can_channel_1) lidar_dist = can1_->get_lidar_feedback_address();
    else lidar_dist = can2_->get_lidar_feedback_address();

    chThdSleep(TIME_MS2I(500));

}

void GimbalIF::clip_gimbal_current() {

#if GIMBAL_INTERFACE_ENABLE_CLIP
    ABS_CROP(*target_current[YAW], GIMBAL_INTERFACE_MAX_CURRENT);
#endif

    // Fill the current of Pitch
#if GIMBAL_INTERFACE_ENABLE_CLIP
    ABS_CROP(*target_current[PITCH], GIMBAL_INTERFACE_MAX_CURRENT);
#endif

    // Fill the current of bullet loader
#if GIMBAL_INTERFACE_ENABLE_CLIP
    ABS_CROP(*target_current[BULLET], GIMBAL_INTERFACE_BULLET_LOADER_MAX_CURRENT);
#endif

    // Fill the current of plate
#if GIMBAL_INTERFACE_ENABLE_CLIP
    ABS_CROP(*target_current[PLATE], GIMBAL_INTERFACE_BULLET_PLATE_MAX_CURRENT);
#endif

#if GIMBAL_INTERFACE_ENABLE_CLIP
    ABS_CROP(*target_current[FW_LEFT], GIMBAL_INTERFACE_MAX_CURRENT);
#endif

#if GIMBAL_INTERFACE_ENABLE_CLIP
    ABS_CROP(*target_current[FW_RIGHT], GIMBAL_INTERFACE_MAX_CURRENT);
#endif

}


/** @} */