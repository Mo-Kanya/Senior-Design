//
// Created by Chen Qian on 11/17/21.
//

#include "VirtualCOMPort.h"
#include "ch.h"
uint8_t VirtualCOMPort::rxbuffer[100];
uint8_t VirtualCOMPort::txbuffer[100];
uint8_t VirtualCOMPort::rxmode=0;
int16_t VirtualCOMPort::target_theta=4096;
int16_t VirtualCOMPort::target_vx = 3000;
int16_t VirtualCOMPort::target_vy = 3000;
VirtualCOMPort::DataReceiveThread VirtualCOMPort::data_receive_thd;
time_msecs_t VirtualCOMPort::last_update_time = 0;

void VirtualCOMPort::init(SerialUSBDriver *SDU_, tprio_t rx_thd_prio) {
    SDU = SDU_;
    sduObjectInit(SDU);
    sduStart(SDU, &serusbcfg);

    usbDisconnectBus(serusbcfg.usbp);
    chThdSleepMilliseconds(1500);
    usbStart(serusbcfg.usbp, &usbcfg);
    usbConnectBus(serusbcfg.usbp);

    data_receive_thd.start(rx_thd_prio);
}

int VirtualCOMPort::send_data(uint8_t *data, unsigned int size) {
    return chnWriteTimeout(SDU, data,  size, TIME_MS2I(3));
}

void VirtualCOMPort::DataReceiveThread::main() {
    setName("vcom_rx_thd");
    while (!shouldTerminate()) {

        int bytes_received = 0;

        chSysLock();  ///
        bytes_received = chnReadTimeout(SDU, rxbuffer, 8, 2);
        chSysUnlock(); ///

        if (bytes_received == 8) {
            rxmode = rxbuffer[6]; // to check
            target_vx = (int16_t)(rxbuffer[0] << 8 | rxbuffer[1]);
            target_vy = (int16_t)(rxbuffer[2] << 8 | rxbuffer[3]);
            target_theta = (int16_t)(rxbuffer[4] << 8 | rxbuffer[5]);

            // err_msg
            last_update_time = SYSTIME;
        }

        chThdSleepMilliseconds(15);
    }
}
