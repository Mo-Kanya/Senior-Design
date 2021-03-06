//
// Created by Quoke on 11/24/2021.
//

#include "Communicator.h"

Communicator::CommunicatorThd Communicator::communicator_thd;
uint8_t Communicator::tx_angles[16];
time_msecs_t Communicator::last_send_time = 0;
time_msecs_t Communicator::cur_send_time = 0;
int Communicator::last_transferred = 0;

void Communicator::init(tprio_t communicator_prio_) {
    communicator_thd.start(communicator_prio_);
}

void Communicator::CommunicatorThd::main() {
    setName("Communicator");
    while(!shouldTerminate()) {
        float motor_v1 = ChassisSKD::get_actual_velocity(ChassisSKD::FR)+2500.0f; // degree/s
        float motor_v2 = ChassisSKD::get_actual_velocity(ChassisSKD::FL)+2500.0f;
        float motor_v3 = ChassisSKD::get_actual_velocity(ChassisSKD::BL)+2500.0f;
        float motor_v4 = ChassisSKD::get_actual_velocity(ChassisSKD::BR)+2500.0f;
//        int16_t update_diff = VirtualCOMPort::cur_update_time - VirtualCOMPort::last_update_time;
//        int16_t send_diff = cur_send_time - last_send_time;
//        float tar = ChassisSKD::get_target_theta() + 360.0f;
//        float w = ChassisSKD::get_target_w() + 720.0f;

        tx_angles[1] = (uint8_t)(((int16_t)motor_v1) >> 8);
        tx_angles[2] = (uint8_t)((int16_t)(motor_v1));

        tx_angles[3] = (uint8_t)(((int16_t)motor_v2) >> 8);
        tx_angles[4] = (uint8_t)((int16_t)(motor_v2));

//        tx_angles[5] = (uint8_t)((send_diff) >> 8);
//        tx_angles[6] = (uint8_t)(send_diff);
//        tx_angles[7] = (uint8_t)((update_diff) >> 8);
//        tx_angles[8] = (uint8_t)(update_diff);

        tx_angles[5] = (uint8_t)(((int16_t)motor_v3) >> 8);
        tx_angles[6] = (uint8_t)((int16_t)(motor_v3));
        tx_angles[7] = (uint8_t)(((int16_t)motor_v4) >> 8);
        tx_angles[8] = (uint8_t)((int16_t)(motor_v4));

        float direction = ChassisSKD::get_last_angle() + 180.0f; // 0-360
        tx_angles[9] = (uint8_t)(((int16_t)(direction / 360.0f * 8192.0f)) >> 8);
        tx_angles[10] = (uint8_t)((int16_t)(direction / 360.0f * 8192.0f));
        tx_angles[11] = (uint8_t) UserI::get_mode();
        tx_angles[12] = (uint8_t) UserI::get_command();
        tx_angles[13] = (uint8_t) cur_send_time-last_send_time;
        tx_angles[14] = (uint8_t) VirtualCOMPort::cur_update_time - VirtualCOMPort::last_update_time;
        tx_angles[15] = 0;

        last_transferred =  VirtualCOMPort::send_data(tx_angles, 16);
        if (last_transferred != 16) {
            last_transferred = VirtualCOMPort::send_data(tx_angles, 16);
        }
        if (last_transferred == 16) {
            last_send_time = cur_send_time;
            cur_send_time = SYSTIME;
        }

        chThdSleepMilliseconds(5); //5
    }
}
