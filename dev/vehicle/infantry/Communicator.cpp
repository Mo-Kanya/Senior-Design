//
// Created by Quoke on 11/24/2021.
//

#include "Communicator.h"

Communicator::CommunicatorThd Communicator::communicator_thd;
uint8_t Communicator::tx_angles[13];

void Communicator::init(tprio_t communicator_prio_) {
    communicator_thd.start(communicator_prio_);
}

void Communicator::CommunicatorThd::main() {
    setName("Communicator");
    while(!shouldTerminate()) {
// / 360.0f * 8192.0f
        float motor_v1 = ChassisSKD::get_actual_velocity(ChassisSKD::FR); // degree/s
        float motor_v2 = ChassisSKD::get_actual_velocity(ChassisSKD::FL);
        float motor_v3 = ChassisSKD::get_actual_velocity(ChassisSKD::BL);
        float motor_v4 = ChassisSKD::get_actual_velocity(ChassisSKD::BR);
        float direction = ChassisSKD::get_last_angle() + 180.0f; // 0-360
        float tar = ChassisSKD::get_target_theta() + 360.0f;
        float w = ChassisSKD::get_target_w() + 720.0f;
        tx_angles[1] = (uint8_t)(((int16_t)motor_v1) >> 8);
        tx_angles[2] = (uint8_t)((int16_t)(motor_v1));
        tx_angles[3] = (uint8_t)(((int16_t)motor_v2) >> 8);
        tx_angles[4] = (uint8_t)((int16_t)(motor_v2));
//        tx_angles[5] = (uint8_t)(((int16_t)motor_v3) >> 8);
//        tx_angles[6] = (uint8_t)((int16_t)(motor_v3));
//        tx_angles[7] = (uint8_t)(((int16_t)motor_v4) >> 8);
//        tx_angles[8] = (uint8_t)((int16_t)(motor_v4));

        tx_angles[5] = (uint8_t)(((int16_t)(w / 360.0f * 8192.0f)) >> 8);
        tx_angles[6] = (uint8_t)((int16_t)(w / 360.0f * 8192.0f));

        tx_angles[7] = (uint8_t)(((int16_t)(tar / 360.0f * 8192.0f)) >> 8);
        tx_angles[8] = (uint8_t)((int16_t)(tar / 360.0f * 8192.0f));

        tx_angles[9] = (uint8_t)(((int16_t)(direction / 360.0f * 8192.0f)) >> 8);
        tx_angles[10] = (uint8_t)((int16_t)(direction / 360.0f * 8192.0f));

        tx_angles[11] = (uint8_t) UserI::get_mode();
        tx_angles[12] = (uint8_t) 0;
        VirtualCOMPort::send_data(tx_angles, 13);
//        Shell::printf("torque:   %d %d, mode: %d" SHELL_NEWLINE_STR, VirtualCOMPort::target_torque[0], VirtualCOMPort::target_torque[1], VirtualCOMPort::rxmode);
//        Shell::printf("rxbuffer:");
//        for (int i = 0; i  < 5; i++) {
//            Shell::printf(" %d,", VirtualCOMPort::rxbuffer[i]);
//        }
//        Shell::printf(SHELL_NEWLINE_STR);

        sleep(TIME_MS2I(100)); //5
    }
}
