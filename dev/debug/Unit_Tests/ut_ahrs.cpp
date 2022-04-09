//
// Created by liuzikai on 2019-05-13.
//

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "shell.h"

#include "ahrs.h"
#include "buzzer_scheduler.h"

#include "sd_card_interface.h"

using namespace chibios_rt;

static constexpr Matrix33 AHRS_MATRIX = {{0.0f, -1.0f, 0.0f}, \
                              {1.0f, 0.0f, 0.0f}, \
                              {0.0f, 0.0f, 1.0f}};
static float momentum = 0.0f;


//static constexpr Matrix33 ANGLE_INSTALLATION_MATRIX = {{1.0f, 0.0f, 0.0f},
//                                                              {0.0f, 1.0f, 0.0f},
//                                                              {0.0f, 0.0f, -1.0f}};
//
//
//static constexpr Matrix33 GYRO_INSTALLATION_MATRIX = {{0.0f,  -1.0f, 0.0f},
//                                                             {0.0f,  0.0f,  1.0f},
//                                                             {-1.0f, 0.0f,  0.0f}};

static constexpr Matrix33 ANGLE_INSTALLATION_MATRIX = {{1.0f, 0.0f, 0.0f},
                                                       {0.0f, 1.0f, 0.0f},
                                                       {0.0f, 0.0f, -1.0f}};


static constexpr Matrix33 GYRO_INSTALLATION_MATRIX = {{0.0f,  1.0f, 0.0f},
                                                             {0.0f,  0.0f,  -1.0f},
                                                             {-1.0f, 0.0f,  0.0f}};
AHRSOnBoard ahrs;

class AHRSFeedbackThread : public BaseStaticThread<1024> {
protected:
    void main() final {
        sleep(TIME_MS2I(3000));
        setName("AHRS");
        Vector3D ahrs_bias;
        if (SDCard::get_data(0x0001, &ahrs_bias, sizeof(ahrs_bias)) == SDCard::OK) {
            ahrs.load_calibration_data(ahrs_bias);
            Shell::printf("Use AHRS bias in SD Card");
            Shell::printf("%.4f,%.4f,%.4f" SHELL_NEWLINE_STR, ahrs_bias.x, ahrs_bias.y, ahrs_bias.z);
        } else {
            ahrs.load_calibration_data({0.135951459, -1.207195401, 0.213372603}); //{0.136242627, -1.2080826357, 0.1428485357}
            Shell::printf("Use default AHRS bias");
        }
        sleep(TIME_MS2I(2000));
        ahrs.start(AHRS_MATRIX, HIGHPRIO - 2);
        BuzzerSKD::init(LOWPRIO);
        BuzzerSKD::play_sound(BuzzerSKD::sound_startup);
        while (!shouldTerminate()) {
            Vector3D angle =  ANGLE_INSTALLATION_MATRIX * ahrs.get_angle();
            Shell::printf("!a,%.4f,%.4f,%.4f" SHELL_NEWLINE_STR,
                          angle.x,
                          angle.y,
                          angle.z);
//            Vector3D accs = ANGLE_INSTALLATION_MATRIX *ahrs.get_accel();
//            Vector3D gyro = GYRO_INSTALLATION_MATRIX*ahrs.get_gyro();
//            momentum = momentum*0.75 + gyro.x*0.25;
//            Shell::printf("acc ,%.4f,%.4f,%.4f,%.4f" SHELL_NEWLINE_STR,
//                          accs.x, // R fw
//                          accs.y, // R lr
//                          accs.z, momentum); // R up down

//            Shell::printf("gyro ,%.4f,%.4f,%.4f" SHELL_NEWLINE_STR,
//                          gyro.x,
//                          gyro.y,
//                          gyro.z);
//            Shell::printf("gyro.x = %.2f, gyro.z = %.2f, angle.y = %.2f ,ans = %.2f" SHELL_NEWLINE_STR,
//                          gyro.x,
//                          gyro.z,
//                          angle.y,
//                          gyro.x * cosf(angle.y / 180.0f * M_PI) + gyro.z * sinf(angle.y / 180.0f * M_PI));
            sleep(TIME_MS2I(250));
        }
    }
} feedbackThread;


//void cmd_echo_gyro_bias(BaseSequentialStream *chp, int argc, char *argv[]) {
//    (void) argv;
//    if (argc != 0) {
//        shellUsage(chp, "echo_bias");
//        return;
//    }
//
//    chprintf(chp, "gyro_bias.x = %f" SHELL_NEWLINE_STR, ahrs.gyro_bias.x);
//    chprintf(chp, "gyro_bias.y = %f" SHELL_NEWLINE_STR, ahrs.gyro_bias.y);
//    chprintf(chp, "gyro_bias.z = %f" SHELL_NEWLINE_STR, ahrs.gyro_bias.z);
//    chprintf(chp, "temp = %f" SHELL_NEWLINE_STR, ahrs.temperature);
//}
//
//
//ShellCommand ahrsShellCommands[] = {
//        {"echo_bias", cmd_echo_gyro_bias},
//        {nullptr,     nullptr}
//};

int main(void) {

    halInit();
    System::init();

    if (SDCard::init()) {
        SDCard::read_all();
    }

    Shell::start(NORMALPRIO - 10);
    // Shell::addCommands(ahrsShellCommands);
    LED::all_off();

    feedbackThread.start(NORMALPRIO);

    // See chconf.h for what this #define means.
#if CH_CFG_NO_IDLE_THREAD
    // ChibiOS idle thread has been disabled, main() should implement infinite loop
    while (true) {}
#else
    // When main() quits, the main thread will somehow enter an infinite loop, so we set the priority to lowest
    // before quitting, to let other threads run normally
    BaseThread::setPriority(1);
#endif
    return 0;
}