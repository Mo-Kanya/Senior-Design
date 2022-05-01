//
// Created by liuzikai on 2019-06-25.
//

#ifndef META_INFANTRY_USER_INFANTRY_H
#define META_INFANTRY_USER_INFANTRY_H

#include "ch.hpp"

#include "remote_interpreter.h"

#include "chassis_logic.h"
#include "vision_scheduler.h"

#include "inspector_infantry.h"

#include "VirtualCOMPort.h"

class UserI {

public:

    static void start(tprio_t user_thd_prio, tprio_t user_action_thd_prio);
    static int get_mode();
    static int get_command();

private:

    /// Chassis Config
    static float base_power;             // [w]
    static float base_v_forward;        // [mm/s]
    static float chassis_v_left_right;  // [mm/s]
    static float Base_left_right_power; // [w]
    static  float Base_left_right;      // [mm/s]
    static float chassis_v_forward;     // [mm/s]
    static float chassis_v_backward;    // [mm/s]

    static int control_mode; // 0=vision; 1=rc; 2=programming
    static int motion_mode; // 0=undef; 1=translation; 2=turning;
    static float target_angle_;
    static float target_vx_;
    static float target_vy_;
    static int sector_;

    static bool mag_status;

    /// Helpers

    /// User Thread
    static constexpr unsigned USER_THREAD_INTERVAL = 7;  // [ms]
    class UserThread : public chibios_rt::BaseStaticThread<512> {
        void main() override;
    };

    static UserThread userThread;

    /// Friend Configure Functions
    friend void chassis_get_config(BaseSequentialStream *chp, int argc, char *argv[]);
    friend void chassis_set_config(BaseSequentialStream *chp, int argc, char *argv[]);

};


#endif //META_INFANTRY_USER_INFANTRY_H
