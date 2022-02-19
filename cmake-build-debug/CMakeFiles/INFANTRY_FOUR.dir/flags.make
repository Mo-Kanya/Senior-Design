# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.19

# compile ASM with /usr/local/bin/arm-none-eabi-gcc
# compile C with /usr/local/bin/arm-none-eabi-gcc
# compile CXX with /usr/local/bin/arm-none-eabi-g++
ASM_DEFINES = -DARM_MATH_CM4 -DARM_MATH_MATRIX_CHECK -DARM_MATH_ROUNDING -DBUILD_TARGET_NAME=\"INFANTRY_FOUR\" -DINFANTRY -DINFANTRY_CHASSIS_ENABLE=1 -DINFANTRY_FOUR -DINFANTRY_GIMBAL_ENABLE=1 -DINFANTRY_SUPER_CAPACITOR_ENABLE=1 -DINFANTRY_VISION_ENABLE=1 -D__FPU_PRESENT=1U

ASM_INCLUDES = -I/Users/kanya/Projects/Senior-Design/./os/license -I/Users/kanya/Projects/Senior-Design/./os/common/portability/GCC -I/Users/kanya/Projects/Senior-Design/./os/common/startup/ARMCMx/compilers/GCC -I/Users/kanya/Projects/Senior-Design/./os/common/startup/ARMCMx/devices/STM32F4xx -I/Users/kanya/Projects/Senior-Design/./os/common/ext/ARM/CMSIS/Core/Include -I/Users/kanya/Projects/Senior-Design/./os/common/ext/ST/STM32F4xx -I/Users/kanya/Projects/Senior-Design/./os/hal/include -I/Users/kanya/Projects/Senior-Design/./os/hal/ports/common/ARMCMx -I/Users/kanya/Projects/Senior-Design/./os/hal/ports/STM32/STM32F4xx -I/Users/kanya/Projects/Senior-Design/./os/hal/ports/STM32/LLD/ADCv2 -I/Users/kanya/Projects/Senior-Design/./os/hal/ports/STM32/LLD/CANv1 -I/Users/kanya/Projects/Senior-Design/./os/hal/ports/STM32/LLD/DACv1 -I/Users/kanya/Projects/Senior-Design/./os/hal/ports/STM32/LLD/DMAv2 -I/Users/kanya/Projects/Senior-Design/./os/hal/ports/STM32/LLD/EXTIv1 -I/Users/kanya/Projects/Senior-Design/./os/hal/ports/STM32/LLD/GPIOv2 -I/Users/kanya/Projects/Senior-Design/./os/hal/ports/STM32/LLD/I2Cv1 -I/Users/kanya/Projects/Senior-Design/./os/hal/ports/STM32/LLD/MACv1 -I/Users/kanya/Projects/Senior-Design/./os/hal/ports/STM32/LLD/OTGv1 -I/Users/kanya/Projects/Senior-Design/./os/hal/ports/STM32/LLD/QUADSPIv1 -I/Users/kanya/Projects/Senior-Design/./os/hal/ports/STM32/LLD/RTCv2 -I/Users/kanya/Projects/Senior-Design/./os/hal/ports/STM32/LLD/SPIv1 -I/Users/kanya/Projects/Senior-Design/./os/hal/ports/STM32/LLD/SDIOv1 -I/Users/kanya/Projects/Senior-Design/./os/hal/ports/STM32/LLD/TIMv1 -I/Users/kanya/Projects/Senior-Design/./os/hal/ports/STM32/LLD/USARTv1 -I/Users/kanya/Projects/Senior-Design/./os/hal/ports/STM32/LLD/xWDGv1 -I/Users/kanya/Projects/Senior-Design/./dev/board -I/Users/kanya/Projects/Senior-Design/./os/hal/osal/rt -I/Users/kanya/Projects/Senior-Design/./os/rt/include -I/Users/kanya/Projects/Senior-Design/./os/common/oslib/include -I/Users/kanya/Projects/Senior-Design/./os/common/ports/ARMCMx -I/Users/kanya/Projects/Senior-Design/./os/common/ports/ARMCMx/compilers/GCC -I/Users/kanya/Projects/Senior-Design/./os/various/cpp_wrappers -I/Users/kanya/Projects/Senior-Design/./os/hal/lib/streams -I/Users/kanya/Projects/Senior-Design/dev -I/Users/kanya/Projects/Senior-Design/dev/common -I/Users/kanya/Projects/Senior-Design/dev/debug -I/Users/kanya/Projects/Senior-Design/dev/debug/shell -I/Users/kanya/Projects/Senior-Design/dev/interface -I/Users/kanya/Projects/Senior-Design/dev/interface/ahrs -I/Users/kanya/Projects/Senior-Design/dev/module -I/Users/kanya/Projects/Senior-Design/dev/scheduler -I/Users/kanya/Projects/Senior-Design/dev/logic -I/Users/kanya/Projects/Senior-Design/dev/vehicle -I/Users/kanya/Projects/Senior-Design/dev/board/rm_board_2018_a -I/Users/kanya/Projects/Senior-Design/dev/vehicle/infantry -I/Users/kanya/Projects/Senior-Design/cmsis/dsp/include

ASM_FLAGS = -x assembler-with-cpp -mcpu=cortex-m4 -fomit-frame-pointer -falign-functions=16 -ffunction-sections -fdata-sections -fno-common -flto -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DCORTEX_USE_FPU=TRUE -mthumb -DTHUMB -DTHUMB_PRESENT -mno-thumb-interwork -DTHUMB_NO_INTERWORKING -g

C_DEFINES = -DARM_MATH_CM4 -DARM_MATH_MATRIX_CHECK -DARM_MATH_ROUNDING -DBUILD_TARGET_NAME=\"INFANTRY_FOUR\" -DINFANTRY -DINFANTRY_CHASSIS_ENABLE=1 -DINFANTRY_FOUR -DINFANTRY_GIMBAL_ENABLE=1 -DINFANTRY_SUPER_CAPACITOR_ENABLE=1 -DINFANTRY_VISION_ENABLE=1 -D__FPU_PRESENT=1U

C_INCLUDES = -I/Users/kanya/Projects/Senior-Design/./os/license -I/Users/kanya/Projects/Senior-Design/./os/common/portability/GCC -I/Users/kanya/Projects/Senior-Design/./os/common/startup/ARMCMx/compilers/GCC -I/Users/kanya/Projects/Senior-Design/./os/common/startup/ARMCMx/devices/STM32F4xx -I/Users/kanya/Projects/Senior-Design/./os/common/ext/ARM/CMSIS/Core/Include -I/Users/kanya/Projects/Senior-Design/./os/common/ext/ST/STM32F4xx -I/Users/kanya/Projects/Senior-Design/./os/hal/include -I/Users/kanya/Projects/Senior-Design/./os/hal/ports/common/ARMCMx -I/Users/kanya/Projects/Senior-Design/./os/hal/ports/STM32/STM32F4xx -I/Users/kanya/Projects/Senior-Design/./os/hal/ports/STM32/LLD/ADCv2 -I/Users/kanya/Projects/Senior-Design/./os/hal/ports/STM32/LLD/CANv1 -I/Users/kanya/Projects/Senior-Design/./os/hal/ports/STM32/LLD/DACv1 -I/Users/kanya/Projects/Senior-Design/./os/hal/ports/STM32/LLD/DMAv2 -I/Users/kanya/Projects/Senior-Design/./os/hal/ports/STM32/LLD/EXTIv1 -I/Users/kanya/Projects/Senior-Design/./os/hal/ports/STM32/LLD/GPIOv2 -I/Users/kanya/Projects/Senior-Design/./os/hal/ports/STM32/LLD/I2Cv1 -I/Users/kanya/Projects/Senior-Design/./os/hal/ports/STM32/LLD/MACv1 -I/Users/kanya/Projects/Senior-Design/./os/hal/ports/STM32/LLD/OTGv1 -I/Users/kanya/Projects/Senior-Design/./os/hal/ports/STM32/LLD/QUADSPIv1 -I/Users/kanya/Projects/Senior-Design/./os/hal/ports/STM32/LLD/RTCv2 -I/Users/kanya/Projects/Senior-Design/./os/hal/ports/STM32/LLD/SPIv1 -I/Users/kanya/Projects/Senior-Design/./os/hal/ports/STM32/LLD/SDIOv1 -I/Users/kanya/Projects/Senior-Design/./os/hal/ports/STM32/LLD/TIMv1 -I/Users/kanya/Projects/Senior-Design/./os/hal/ports/STM32/LLD/USARTv1 -I/Users/kanya/Projects/Senior-Design/./os/hal/ports/STM32/LLD/xWDGv1 -I/Users/kanya/Projects/Senior-Design/./dev/board -I/Users/kanya/Projects/Senior-Design/./os/hal/osal/rt -I/Users/kanya/Projects/Senior-Design/./os/rt/include -I/Users/kanya/Projects/Senior-Design/./os/common/oslib/include -I/Users/kanya/Projects/Senior-Design/./os/common/ports/ARMCMx -I/Users/kanya/Projects/Senior-Design/./os/common/ports/ARMCMx/compilers/GCC -I/Users/kanya/Projects/Senior-Design/./os/various/cpp_wrappers -I/Users/kanya/Projects/Senior-Design/./os/hal/lib/streams -I/Users/kanya/Projects/Senior-Design/dev -I/Users/kanya/Projects/Senior-Design/dev/common -I/Users/kanya/Projects/Senior-Design/dev/debug -I/Users/kanya/Projects/Senior-Design/dev/debug/shell -I/Users/kanya/Projects/Senior-Design/dev/interface -I/Users/kanya/Projects/Senior-Design/dev/interface/ahrs -I/Users/kanya/Projects/Senior-Design/dev/module -I/Users/kanya/Projects/Senior-Design/dev/scheduler -I/Users/kanya/Projects/Senior-Design/dev/logic -I/Users/kanya/Projects/Senior-Design/dev/vehicle -I/Users/kanya/Projects/Senior-Design/dev/board/rm_board_2018_a -I/Users/kanya/Projects/Senior-Design/dev/vehicle/infantry -I/Users/kanya/Projects/Senior-Design/cmsis/dsp/include

C_FLAGS = -mcpu=cortex-m4 -fomit-frame-pointer -falign-functions=16 -ffunction-sections -fdata-sections -fno-common -flto -mfloat-abi=hard -mfpu=fpv4-sp-d16  -Wall -Wextra -Wundef -Wstrict-prototypes -DCORTEX_USE_FPU=TRUE -DSHELL_CONFIG_FILE -mthumb -DTHUMB -DTHUMB_PRESENT -mno-thumb-interwork -DTHUMB_NO_INTERWORKING -g

CXX_DEFINES = -DARM_MATH_CM4 -DARM_MATH_MATRIX_CHECK -DARM_MATH_ROUNDING -DBUILD_TARGET_NAME=\"INFANTRY_FOUR\" -DINFANTRY -DINFANTRY_CHASSIS_ENABLE=1 -DINFANTRY_FOUR -DINFANTRY_GIMBAL_ENABLE=1 -DINFANTRY_SUPER_CAPACITOR_ENABLE=1 -DINFANTRY_VISION_ENABLE=1 -D__FPU_PRESENT=1U

CXX_INCLUDES = -I/Users/kanya/Projects/Senior-Design/./os/license -I/Users/kanya/Projects/Senior-Design/./os/common/portability/GCC -I/Users/kanya/Projects/Senior-Design/./os/common/startup/ARMCMx/compilers/GCC -I/Users/kanya/Projects/Senior-Design/./os/common/startup/ARMCMx/devices/STM32F4xx -I/Users/kanya/Projects/Senior-Design/./os/common/ext/ARM/CMSIS/Core/Include -I/Users/kanya/Projects/Senior-Design/./os/common/ext/ST/STM32F4xx -I/Users/kanya/Projects/Senior-Design/./os/hal/include -I/Users/kanya/Projects/Senior-Design/./os/hal/ports/common/ARMCMx -I/Users/kanya/Projects/Senior-Design/./os/hal/ports/STM32/STM32F4xx -I/Users/kanya/Projects/Senior-Design/./os/hal/ports/STM32/LLD/ADCv2 -I/Users/kanya/Projects/Senior-Design/./os/hal/ports/STM32/LLD/CANv1 -I/Users/kanya/Projects/Senior-Design/./os/hal/ports/STM32/LLD/DACv1 -I/Users/kanya/Projects/Senior-Design/./os/hal/ports/STM32/LLD/DMAv2 -I/Users/kanya/Projects/Senior-Design/./os/hal/ports/STM32/LLD/EXTIv1 -I/Users/kanya/Projects/Senior-Design/./os/hal/ports/STM32/LLD/GPIOv2 -I/Users/kanya/Projects/Senior-Design/./os/hal/ports/STM32/LLD/I2Cv1 -I/Users/kanya/Projects/Senior-Design/./os/hal/ports/STM32/LLD/MACv1 -I/Users/kanya/Projects/Senior-Design/./os/hal/ports/STM32/LLD/OTGv1 -I/Users/kanya/Projects/Senior-Design/./os/hal/ports/STM32/LLD/QUADSPIv1 -I/Users/kanya/Projects/Senior-Design/./os/hal/ports/STM32/LLD/RTCv2 -I/Users/kanya/Projects/Senior-Design/./os/hal/ports/STM32/LLD/SPIv1 -I/Users/kanya/Projects/Senior-Design/./os/hal/ports/STM32/LLD/SDIOv1 -I/Users/kanya/Projects/Senior-Design/./os/hal/ports/STM32/LLD/TIMv1 -I/Users/kanya/Projects/Senior-Design/./os/hal/ports/STM32/LLD/USARTv1 -I/Users/kanya/Projects/Senior-Design/./os/hal/ports/STM32/LLD/xWDGv1 -I/Users/kanya/Projects/Senior-Design/./dev/board -I/Users/kanya/Projects/Senior-Design/./os/hal/osal/rt -I/Users/kanya/Projects/Senior-Design/./os/rt/include -I/Users/kanya/Projects/Senior-Design/./os/common/oslib/include -I/Users/kanya/Projects/Senior-Design/./os/common/ports/ARMCMx -I/Users/kanya/Projects/Senior-Design/./os/common/ports/ARMCMx/compilers/GCC -I/Users/kanya/Projects/Senior-Design/./os/various/cpp_wrappers -I/Users/kanya/Projects/Senior-Design/./os/hal/lib/streams -I/Users/kanya/Projects/Senior-Design/dev -I/Users/kanya/Projects/Senior-Design/dev/common -I/Users/kanya/Projects/Senior-Design/dev/debug -I/Users/kanya/Projects/Senior-Design/dev/debug/shell -I/Users/kanya/Projects/Senior-Design/dev/interface -I/Users/kanya/Projects/Senior-Design/dev/interface/ahrs -I/Users/kanya/Projects/Senior-Design/dev/module -I/Users/kanya/Projects/Senior-Design/dev/scheduler -I/Users/kanya/Projects/Senior-Design/dev/logic -I/Users/kanya/Projects/Senior-Design/dev/vehicle -I/Users/kanya/Projects/Senior-Design/dev/board/rm_board_2018_a -I/Users/kanya/Projects/Senior-Design/dev/vehicle/infantry -I/Users/kanya/Projects/Senior-Design/cmsis/dsp/include

CXX_FLAGS = -mcpu=cortex-m4 -fomit-frame-pointer -falign-functions=16 -ffunction-sections -fdata-sections -fno-common -flto -mfloat-abi=hard -mfpu=fpv4-sp-d16 -std=gnu++11 -fno-exceptions -fno-rtti -Wall -Wextra -Wundef -DCORTEX_USE_FPU=TRUE -DSHELL_CONFIG_FILE -mthumb -DTHUMB -DTHUMB_PRESENT -mno-thumb-interwork -DTHUMB_NO_INTERWORKING -g -std=gnu++11

