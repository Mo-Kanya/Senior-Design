//
// Created by Quoke on 11/24/2021.
//

#include "Communicator.h"

Communicator::CommunicatorThd Communicator::communicator_thd;
uint8_t Communicator::tx_angles[13];
//time_msecs_t Communicator::last_send_time = 0;
//int Communicator::last_transferred = 0;

void Communicator::init(tprio_t communicator_prio_) {
    communicator_thd.start(communicator_prio_);
}

void Communicator::CommunicatorThd::main() {
    setName("Communicator");
    while(!shouldTerminate()) {
// / 360.0f * 8192.0f
        float motor_v1 = ChassisSKD::get_actual_velocity(ChassisSKD::FR)+2500.0f; // degree/s
        float motor_v2 = ChassisSKD::get_actual_velocity(ChassisSKD::FL)+2500.0f;
        float motor_v3 = ChassisSKD::get_actual_velocity(ChassisSKD::BL)+2500.0f;
        float motor_v4 = ChassisSKD::get_actual_velocity(ChassisSKD::BR)+2500.0f;
//        int16_t update_time = VirtualCOMPort::last_update_time;
//        float tar = ChassisSKD::get_target_theta() + 360.0f;
//        float w = ChassisSKD::get_target_w() + 720.0f;

        tx_angles[1] = (uint8_t)(((int16_t)motor_v1) >> 8);
        tx_angles[2] = (uint8_t)((int16_t)(motor_v1));

//        tx_angles[1] = (uint8_t)(((int16_t)(SYSTIME-last_send_time)) >> 8);
//        tx_angles[2] = (uint8_t)((int16_t)(SYSTIME-last_send_time));
//        tx_angles[3] = (uint8_t)(((int16_t)last_transferred) >> 8);
//        tx_angles[4] = (uint8_t)((int16_t)last_transferred);

        tx_angles[3] = (uint8_t)(((int16_t)motor_v2) >> 8);
        tx_angles[4] = (uint8_t)((int16_t)(motor_v2));
        tx_angles[5] = (uint8_t)(((int16_t)motor_v3) >> 8);
        tx_angles[6] = (uint8_t)((int16_t)(motor_v3));
        tx_angles[7] = (uint8_t)(((int16_t)motor_v4) >> 8);
        tx_angles[8] = (uint8_t)((int16_t)(motor_v4));

//        tx_angles[3] = (uint8_t)(((int16_t)VirtualCOMPort::target_vy) >> 8);
//        tx_angles[4] = (uint8_t)((int16_t)VirtualCOMPort::target_vy);
//        tx_angles[5] = (uint8_t)(((int16_t)VirtualCOMPort::target_vx) >> 8);
//        tx_angles[6] = (uint8_t)((int16_t)VirtualCOMPort::target_vx);

//        tx_angles[7] = (uint8_t)((update_time) >> 8);
//        tx_angles[8] = (uint8_t)(update_time);

//        tx_angles[9] = (uint8_t)(((int16_t)(VirtualCOMPort::target_theta)) >> 8);
//        tx_angles[10] = (uint8_t)((int16_t)(VirtualCOMPort::target_theta));
        chSysLock();  ///
        float direction = ChassisSKD::get_last_angle() + 180.0f; // 0-360
        tx_angles[9] = (uint8_t)(((int16_t)(direction / 360.0f * 8192.0f)) >> 8);
        tx_angles[10] = (uint8_t)((int16_t)(direction / 360.0f * 8192.0f));
        tx_angles[11] = (uint8_t) UserI::get_mode();
        tx_angles[12] = (uint8_t) 0;
        VirtualCOMPort::send_data(tx_angles, 13);
        chSysUnlock(); ///
//        last_transferred =  VirtualCOMPort::send_data(tx_angles, 13);
//        if (last_transferred == 13) {
//            last_send_time = SYSTIME;
//        }

//        Shell::printf("torque:   %d %d, mode: %d" SHELL_NEWLINE_STR, VirtualCOMPort::target_torque[0], VirtualCOMPort::target_torque[1], VirtualCOMPort::rxmode);
//        Shell::printf("rxbuffer:");
//        for (int i = 0; i  < 5; i++) {
//            Shell::printf(" %d,", VirtualCOMPort::rxbuffer[i]);
//        }
//        Shell::printf(SHELL_NEWLINE_STR);

        chThdSleepMilliseconds(15); //5
    }
}
