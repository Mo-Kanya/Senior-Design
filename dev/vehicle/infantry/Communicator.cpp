//
// Created by Quoke on 11/24/2021.
//

#include "Communicator.h"

Communicator::CommunicatorThd Communicator::communicator_thd;
uint8_t Communicator::tx_angles[5];

void Communicator::init(tprio_t communicator_prio_) {
    communicator_thd.start(communicator_prio_);
}

void Communicator::CommunicatorThd::main() {
    setName("Communicator");
    while(!shouldTerminate()) {
// / 360.0f * 8192.0f
        tx_angles[1] = (uint8_t)(((int16_t)(VirtualCOMPort::target_torque[0])) >> 8);
        tx_angles[2] = (uint8_t)((int16_t)(VirtualCOMPort::target_torque[0]));
        tx_angles[3] = (uint8_t)(((int16_t)(VirtualCOMPort::target_torque[1])) >> 8);
        tx_angles[4] = (uint8_t)((int16_t)(VirtualCOMPort::target_torque[1]));
        VirtualCOMPort::send_data(tx_angles, 5);
//        Shell::printf("torque:   %d %d, mode: %d" SHELL_NEWLINE_STR, VirtualCOMPort::target_torque[0], VirtualCOMPort::target_torque[1], VirtualCOMPort::rxmode);
//        Shell::printf("rxbuffer:");
//        for (int i = 0; i  < 5; i++) {
//            Shell::printf(" %d,", VirtualCOMPort::rxbuffer[i]);
//        }
//        Shell::printf(SHELL_NEWLINE_STR);

        sleep(TIME_MS2I(500));
    }
}
