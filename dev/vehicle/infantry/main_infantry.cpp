//
// Created by liuzikai on 2019-01-27.
//

/// Headers
#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "buzzer_scheduler.h"
#include "common_macro.h"

#include "shell.h"
#include "can_interface.h"
#include "ahrs.h"
#include "remote_interpreter.h"
#include "sd_card_interface.h"

#include "chassis_interface.h"
#include "chassis_scheduler.h"
#include "chassis_logic.h"

#include "vision_interface.h"
#include "vision_scheduler.h"

#include "inspector_infantry.h"
#include "user_infantry.h"
#include "thread_priorities.h"

#include "VirtualCOMPort.h"
#include "Communicator.h"

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

/// Board Guard
#if defined(BOARD_RM_2018_A)
#else
#error "Infantry supports only RM Board 2018 A currently"
#endif

/// Instances
CANInterface can1(&CAND1);
SerialUSBDriver SDU1;
// CANInterface can2(&CAND2);
AHRSOnBoard ahrs;

/// Local Constants
static const Matrix33 ON_BOARD_AHRS_MATRIX_ = ON_BOARD_AHRS_MATRIX;
static const Matrix33 GIMBAL_ANGLE_INSTALLATION_MATRIX_ = GIMBAL_ANGLE_INSTALLATION_MATRIX;
static const Matrix33 GIMBAL_GYRO_INSTALLATION_MATRIX_ = GIMBAL_GYRO_INSTALLATION_MATRIX;

static ChassisIF::motor_can_config_t CHASSIS_MOTOR_CONFIG_[ChassisIF::MOTOR_COUNT] = CHASSIS_MOTOR_CONFIG;



int main() {

    /*** --------------------------- Period 0. Fundamental Setup --------------------------- ***/

    halInit();
    chibios_rt::System::init();

    ////
    VirtualCOMPort::init(&SDU1, THREAD_VIRTUAL_COM_PRIO);

    // Enable power of bullet loader motor
//    palSetPadMode(GPIOH, GPIOH_POWER1_CTRL, PAL_MODE_OUTPUT_PUSHPULL);
//    palSetPad(GPIOH, GPIOH_POWER1_CTRL);

    // Enable power of ultraviolet lights
//    palSetPadMode(GPIOH, GPIOH_POWER2_CTRL, PAL_MODE_OUTPUT_PUSHPULL);
//    palSetPad(GPIOH, GPIOH_POWER2_CTRL);

    /*** ---------------------- Period 1. Modules Setup and Self-Check ---------------------- ***/

    /// Preparation of Period 1
    InspectorI::init(&can1, &ahrs);
    LED::all_off();

    /// Setup Shell
    Shell::start(THREAD_SHELL_PRIO);
    chThdSleepMilliseconds(50);  // wait for logo to print :)

    BuzzerSKD::init(THREAD_BUZZER_SKD_PRIO);

    /// Setup SDCard
    if (SDCard::init()) {
        SDCard::read_all();
        LED::led_on(DEV_BOARD_LED_SD_CARD);  // LED 8 on if SD card inserted
    }

    LED::led_on(DEV_BOARD_LED_SYSTEM_INIT);  // LED 1 on now

    /// Setup CAN1
    can1.start(THREAD_CAN1_RX_PRIO, THREAD_CAN1_TX_PRIO);
    chThdSleepMilliseconds(5);
    InspectorI::startup_check_can();  // check no persistent CAN Error. Block for 100 ms
    LED::led_on(DEV_BOARD_LED_CAN);  // LED 2 on now

    /// Setup SuperCapacitor Port
//    SuperCapacitor::init(&can2, THREAD_SUPERCAP_INIT_PRIO);

    /// Setup Referee
//    Referee::init();

    /// Complete Period 1
    LED::green_on();  // LED Green on now

    /// Setup On-Board AHRS
    Vector3D ahrs_bias;
    if (SDCard::get_data(MPU6500_BIAS_DATA_ID, &ahrs_bias, sizeof(ahrs_bias)) == SDCard::OK) {
        ahrs.load_calibration_data(ahrs_bias);
        LOG("Use AHRS bias in SD Card");
    } else {
        ahrs.load_calibration_data(MPU6500_STORED_GYRO_BIAS);
        LOG_WARN("Use default AHRS bias");
    }
    ahrs.start(ON_BOARD_AHRS_MATRIX_, THREAD_AHRS_PRIO);
    while(!ahrs.ready()) {
        chThdSleepMilliseconds(5);
    }
    InspectorI::startup_check_mpu();  // check MPU6500 has signal. Block for 20 ms
    InspectorI::startup_check_ist();  // check IST8310 has signal. Block for 20 ms
    Shell::addCommands(ahrs.shellCommands);
    Shell::addFeedbackCallback(AHRSOnBoard::cmdFeedback, &ahrs);
    LED::led_on(DEV_BOARD_LED_AHRS);  // LED 3 on now

    /// Setup Remote
    Remote::start();
    InspectorI::startup_check_remote();  // check Remote has signal. Block for 50 ms
    LED::led_on(DEV_BOARD_LED_REMOTE);  // LED 4 on now


    LED::led_on(DEV_BOARD_LED_GIMBAL);  // LED 5 on now


    /// Setup ChassisIF
    ChassisIF::init(&can1, CHASSIS_MOTOR_CONFIG_);
    chThdSleepMilliseconds(10);
    InspectorI::startup_check_chassis_feedback();  // check chassis motors has continuous feedback. Block for 20 ms
    LED::led_on(DEV_BOARD_LED_CHASSIS);  // LED 6 on now


    Communicator::init(THREAD_VISION_SKD_PRIO);


    /// Setup Red Spot Laser
    palSetPad(GPIOG, GPIOG_RED_SPOT_LASER);  // enable the red spot laser

    /*** ------------ Period 2. Calibration and Start Logic Control Thread ----------- ***/

    ChassisSKD::start(&ahrs,  GIMBAL_ANGLE_INSTALLATION_MATRIX_,
                      CHASSIS_WHEEL_BASE, CHASSIS_WHEEL_TREAD, CHASSIS_WHEEL_CIRCUMFERENCE, ChassisSKD::POSITIVE,
                      GIMBAL_YAW_INSTALL_DIRECTION, 0, THREAD_CHASSIS_SKD_PRIO);
    ChassisSKD::load_pid_params(CHASSIS_FOLLOW_PID_THETA2V_PARAMS, CHASSIS_PID_V2I_PARAMS);
    Shell::addCommands(ChassisSKD::shellCommands);
    Shell::addFeedbackCallback(ChassisSKD::cmdFeedback);

    /// Start LGs
    ChassisLG::init();

    /// Setup Vision
//    VisionIF::init();
//    Vision::start(VISION_BASIC_CONTROL_DELAY, THREAD_VISION_SKD_PRIO);
//    Shell::addFeedbackCallback(Vision::cmd_feedback);
//    Shell::addCommands(Vision::shell_commands);

    /// Start Inspector and User Threads
    InspectorI::start_inspection(THREAD_INSPECTOR_PRIO);
    UserI::start(THREAD_USER_PRIO, THREAD_USER_ACTION_PRIO);


    /// Complete Period 2
    BuzzerSKD::play_sound(BuzzerSKD::sound_startup);  // Now play the startup sound


    /*** ------------------------ Period 3. End of main thread ----------------------- ***/

    // Entering empty loop with low priority
#if CH_CFG_NO_IDLE_THREAD  // See chconf.h for what this #define means.
    // ChibiOS idle thread has been disabled, main() should implement infinite loop
    while (true) {}
#else
    // When vehicle() quits, the vehicle thread will somehow enter an infinite loop, so we set the
    // priority to lowest before quitting, to let other threads run normally
    chibios_rt::BaseThread::setPriority(IDLEPRIO);
#endif
    return 0;
}
