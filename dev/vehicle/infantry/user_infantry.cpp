//
// Created by liuzikai on 2019-06-25.
//

#include "user_infantry.h"

// FIXME: change as input parameter from main()

/// Vehicle Specific Configurations
#if defined(INFANTRY_THREE)                                                 /** Infantry #3 **/
#include "vehicle_infantry_three.h"
#elif defined(INFANTRY_FOUR)                                                /** Infantry #4 **/
#include "vehicle_infantry_four.h"
#elif defined(INFANTRY_FIVE)                                                /** Infantry #5 **/
#include "vehicle_infantry_five.h"
#else
#error "File main_infantry.cpp should only be used for Infantry #3, #4, #5."
#endif

/// Chassis Config
float UserI::base_power = 40.0f;
float UserI::base_v_forward = 1500.0f;
float UserI::chassis_v_left_right = 1500.0f;   // [mm/s]
float UserI::chassis_v_forward = 1500.0f;     // [mm/s]
float UserI::chassis_v_backward = 1500.0f;    // [mm/s]
int UserI::control_mode = 1;
float UserI::target_angle_ = 0.0f;
float UserI::target_vx_ = 0.0f;
float UserI::target_vy_ = 0.0f;
int UserI::motion_mode = 0;
int UserI::sector_ = 0;

/// Variables

UserI::UserThread UserI::userThread;

void UserI::start(tprio_t user_thd_prio, tprio_t user_action_thd_prio) {
    userThread.start(user_thd_prio);
}

int UserI::get_mode() {
    return control_mode;
}

int UserI::get_command() {
    return motion_mode;
}

void UserI::UserThread::main() {
    setName("UserI");
    static constexpr PWMConfig FRICTION_WHEELS_PWM_CFG = {
            50000,   // frequency
            1000,    // period
            nullptr, // callback
            {
                    {PWM_OUTPUT_ACTIVE_HIGH, nullptr}, // CH0
                    {PWM_OUTPUT_ACTIVE_HIGH, nullptr}, // CH1
                    {PWM_OUTPUT_DISABLED, nullptr},    // CH2
                    {PWM_OUTPUT_DISABLED, nullptr}     // CH3
            },
            0,
            0
    };
    static constexpr PWMDriver *FRICTION_WHEEL_PWM_DRIVER = &PWMD8;

    pwmStart(FRICTION_WHEEL_PWM_DRIVER, &FRICTION_WHEELS_PWM_CFG);
    pwmEnableChannel(FRICTION_WHEEL_PWM_DRIVER, 1,
                     PWM_PERCENTAGE_TO_WIDTH(FRICTION_WHEEL_PWM_DRIVER, 0));

    while (!shouldTerminate()) {

        /*** ---------------------------------- Chassis --------------------------------- ***/
        if (!InspectorI::remote_failure() && !InspectorI::chassis_failure()) {
            if (Remote::rc.s1 == Remote::S_UP && Remote::rc.s2 == Remote::S_UP) {
                if (VirtualCOMPort::rxmode == 1) {
                    if ( ((int) VirtualCOMPort::sector) != sector_ && VirtualCOMPort::sector < 3 ) {
                        sector_ = (int) VirtualCOMPort::sector;
                        pwmEnableChannel(FRICTION_WHEEL_PWM_DRIVER, 0,
                                         PWM_PERCENTAGE_TO_WIDTH(FRICTION_WHEEL_PWM_DRIVER, 250+ sector_ * 500 ));
                    }
                }
            } else if (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_UP) {
                control_mode = 1;
                if (motion_mode != 0) motion_mode = 0;

                // rc; not programming
                ChassisLG::set_action(ChassisLG::FOLLOW_MODE);
                ChassisLG::set_target(Remote::rc.ch2 * chassis_v_left_right,  // Both use right as positive direction
                                      (Remote::rc.ch3 > 0 ?
                                       Remote::rc.ch3 * 1000 :
                                       Remote::rc.ch3 * 800) ,  // Both use up as positive direction
                                      -Remote::rc.ch0 * 45
                );

            } else if (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_MIDDLE) {
                // programming
                control_mode = 2;
                ChassisLG::set_action(ChassisLG::FOLLOW_MODE);
                if (ABS(-Remote::rc.ch0 * 45) > 2.0f) {
                    motion_mode = 2;
                    ChassisLG::set_target(0,  // Both use right as positive direction
                                          0,   // Both use up as positive direction
                                          -Remote::rc.ch0 * 45);
                } else if (ABS(Remote::rc.ch3)>0) {
                    motion_mode = 1;
                    ChassisLG::set_target(0,  // Both use right as positive direction
                                          (Remote::rc.ch3 > 0 ?
                                           Remote::rc.ch3 * 1000 :
                                           Remote::rc.ch3 * 800),   // Both use up as positive direction
                                          0);
                } else {
                    motion_mode = 0;
                    ChassisLG::set_target(0,  // Both use right as positive direction
                                          0,   // Both use up as positive direction
                                          0);
                }

            } else if (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_DOWN) {

                // Vision
                if (control_mode != 0) {
                    control_mode = 0;
                    target_angle_ = 0.0f;
                    target_vx_ = 0.0f;
                    target_vy_ = 0.0f;
                }
                motion_mode= 0;

                if ( SYSTIME-VirtualCOMPort::last_update_time <= 2000 ) {
                    if (VirtualCOMPort::rxmode == 0) {
                        target_vx_ = (float) VirtualCOMPort::target_vx - 3000.0f;
                        target_vy_ = (float) VirtualCOMPort::target_vy - 3000.0f;
                        target_angle_ = ((float) VirtualCOMPort::target_theta)*360.0f/8192.0f - 180.0f;
                        if ( ((int) VirtualCOMPort::sector) != sector_ && VirtualCOMPort::sector < 3 ) {
                            sector_ = (int) VirtualCOMPort::sector;
                            pwmEnableChannel(FRICTION_WHEEL_PWM_DRIVER, 0,
                                             PWM_PERCENTAGE_TO_WIDTH(FRICTION_WHEEL_PWM_DRIVER, 250+ sector_ * 500 ));
                        }
                    }
                } else {
                    target_angle_ = 0.0f;
                    target_vx_ = 0.0f;
                    target_vy_ = 0.0f;
                }

                // ChassisLG::set_action(GimbalLG::VISION_MODE);
                ChassisLG::set_action(ChassisLG::FOLLOW_MODE);
                ChassisLG::set_target(target_vx_,target_vy_,target_angle_); // theta_
                ////

            } else {
                control_mode = 1;
                motion_mode = 0;
                /// Safe Mode
                ChassisLG::set_action(ChassisLG::FORCED_RELAX_MODE);
            }

        } else {  // InspectorI::remote_failure() || InspectorI::chassis_failure() || InspectorI::gimbal_failure()

            /// Safe Mode
            ChassisLG::set_action(ChassisLG::FORCED_RELAX_MODE);
        }


        /// Final
        sleep(TIME_MS2I(USER_THREAD_INTERVAL));
    }
}
