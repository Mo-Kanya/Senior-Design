//
// Created by Quoke on 11/24/2021.
//

#ifndef META_INFANTRY_COMMUNICATOR_H
#define META_INFANTRY_COMMUNICATOR_H

#include "VirtualCOMPort.h"
#include "chassis_scheduler.h"
#include "user_infantry.h"

using namespace chibios_rt;
class Communicator {
public:

    /**
     * @brief Initiate the communicator.
     */
    static void init(tprio_t communicator_prio_);
//    static time_msecs_t last_send_time;
//    static int last_transferred;

    /**
     * @brief send_angles
     */
    static uint8_t tx_angles[13];

    /**
     * @brief TxRxThread for CDC
     */
    class CommunicatorThd : public BaseStaticThread<512> {
        void main() final;
    };

    static CommunicatorThd communicator_thd;
};


#endif //META_INFANTRY_COMMUNICATOR_H
