//
// Created by Shen Zheng on 2022.
//

#ifndef FIX_ROUTE_DECISION_H
#define FIX_ROUTE_DECISION_H

#include "VirtualCOMPort.h"
#include "chassis_scheduler.h"
#include "user_infantry.h"

using namespace chibios_rt;
class decision_maker {
public:

    /**
     * @brief Initiate the decision maker.
     */
    static void init(tprio_t decision_maker_prio_);

    /**
     * @brief send results of fixed route decision
     */
    static float cur_vx, cur_vy, angle;
    static uint8_t mode;
    static float target_theta, target_vx, target_vy;


    /**
     * @brief Basic function for velocity solve.
     */
    static void compute_v(float FR, float FL, float BL, float BR);

    /**
     * @brief dont know it's what
     */
    class DecisionThd : public BaseStaticThread<512> {
        void main() final;
    };

    static DecisionThd decision_thd;
};


#endif 
