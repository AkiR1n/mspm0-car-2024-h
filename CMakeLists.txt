cmake_minimum_required(VERSION 3.30)
include(mspm0g350x_base.cmake)

project(mspm0_school_2026 C CXX ASM)

include_directories(
    ./Includes
    ./Sources
    ./Sources/tasks
    ./FreeRTOS/include
    ./FreeRTOS/portable/GCC/ARM_CM0/
    ./Drivers/MSPM0
    ./Drivers/MotionV2/include
    ./Drivers/MPU6050
    ./Drivers/OLED_Hardware_I2C
)

set(APP_SOURCES
    ./Sources/main.c
    ./Sources/sysmem.c
    ./Sources/uart_printf.c
    ./Sources/app_state.c
    ./Sources/tasks/motion_task.c
    ./Sources/tasks/imu_task.c
    ./Sources/tasks/oled_task.c
    ./Sources/tasks/log_task.c
)
file(GLOB MSPM0_SUPPORT_SOURCES "./Drivers/MSPM0/*.c")
file(GLOB MOTION_V2_SOURCES "./Drivers/MotionV2/src/*.c")
file(GLOB MPU6050_SOURCES "./Drivers/MPU6050/*.c")
set(OLED_SOURCES ./Drivers/OLED_Hardware_I2C/oled_hardware_i2c.c)
file(GLOB FREERTOS_C_SOURCES "./FreeRTOS/*.c")
file(GLOB FREERTOS_MEMMANG_C_SOURCES "./FreeRTOS/portable/MemMang/heap_4.c")
file(GLOB_RECURSE FREERTOS_PORTABLE_C_SOURCES "./FreeRTOS/portable/GCC/ARM_CM0/*.c")

add_executable(${CMAKE_PROJECT_NAME}.elf
    ${SYSCONFIG_SOURCES}
    ${MSPM0_SDK_SOURCES}
    ${MSPM0_STARTUP}
    ${FREERTOS_C_SOURCES}
    ${FREERTOS_PORTABLE_C_SOURCES}
    ${FREERTOS_MEMMANG_C_SOURCES}
    ${APP_SOURCES}
    ${MSPM0_SUPPORT_SOURCES}
    ${MOTION_V2_SOURCES}
    ${MPU6050_SOURCES}
    ${OLED_SOURCES}
)

# 库顺序很重要：libm 提供 atan2f/asinf；libnosys 提供 _sbrk/_close 等 syscall stub。
# 用 --start-group 容忍循环依赖。
target_link_libraries(${CMAKE_PROJECT_NAME}.elf PRIVATE
    -Wl,--start-group
    m
    c
    nosys
    gcc
    -Wl,--end-group
)

set(HEX_FILE ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex)
set(BIN_FILE ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin)

add_custom_command(
    OUTPUT ${HEX_FILE}
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}.elf> ${HEX_FILE}
    DEPENDS ${CMAKE_PROJECT_NAME}.elf
    VERBATIM
)

add_custom_command(
    OUTPUT ${BIN_FILE}
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}.elf> ${BIN_FILE}
    DEPENDS ${CMAKE_PROJECT_NAME}.elf
    VERBATIM
)

add_custom_target(hex DEPENDS ${HEX_FILE})
add_custom_target(bin DEPENDS ${BIN_FILE})
