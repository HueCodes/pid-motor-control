cmake_minimum_required(VERSION 3.20)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_SIZE arm-none-eabi-size)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

project(pid-motor-control C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Cortex-M4 with hardware FPU
set(MCU_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")
set(CMAKE_C_FLAGS "${MCU_FLAGS} -Wall -Wextra -Werror -ffunction-sections -fdata-sections -Os")
set(CMAKE_C_FLAGS_DEBUG "-g3 -DDEBUG")
set(CMAKE_C_FLAGS_RELEASE "-DNDEBUG")
set(CMAKE_EXE_LINKER_FLAGS "${MCU_FLAGS} -T${CMAKE_SOURCE_DIR}/STM32F411RETx_FLASH.ld -Wl,--gc-sections -specs=nosys.specs -specs=nano.specs -lm")

# STM32 CMSIS and HAL paths (adjust to your local STM32CubeF4 install)
set(CUBE_DIR "$ENV{HOME}/STM32Cube/Repository/STM32Cube_FW_F4" CACHE PATH "STM32CubeF4 firmware package")
set(CMSIS_DIR "${CUBE_DIR}/Drivers/CMSIS")
set(HAL_DIR "${CUBE_DIR}/Drivers/STM32F4xx_HAL_Driver")

add_executable(pid-motor-control
    src/main.c
    src/startup.c
    src/syscalls.c
    src/rcc.c
    src/gpio.c
    src/timer.c
    src/i2c.c
    src/uart.c
    src/mpu6050.c
    src/pid.c
    src/motor.c
    src/cli.c
    src/ssd1306.c
)

target_include_directories(pid-motor-control PRIVATE
    include
    ${CMSIS_DIR}/Include
    ${CMSIS_DIR}/Device/ST/STM32F4xx/Include
)

target_compile_definitions(pid-motor-control PRIVATE
    STM32F411xE
    HSE_VALUE=8000000U
)

# Generate .bin and .hex
add_custom_command(TARGET pid-motor-control POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:pid-motor-control> pid-motor-control.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:pid-motor-control> pid-motor-control.hex
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:pid-motor-control>
)

# Flash target (ST-Link)
add_custom_target(flash
    COMMAND st-flash write pid-motor-control.bin 0x08000000
    DEPENDS pid-motor-control
)

# Host-side test build
option(BUILD_TESTS "Build host-side unit tests" OFF)
if(BUILD_TESTS)
    # Reset cross-compilation for host tests
    enable_testing()
    add_subdirectory(test)
endif()
