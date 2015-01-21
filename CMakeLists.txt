cmake_minimum_required(VERSION 2.8)
option(HOST_TEST "do host based testing (no cross-compiling)" OFF)

if (NOT HOST_TEST)
    set(CMAKE_TOOLCHAIN_FILE cmake/Toolchain-arm-none-eabi.cmake)
endif()

project(px4flow C ASM)

set(JTAGCONFIG ${CMAKE_SOURCE_DIR}/olimex-jtag-tiny.cfg)
#set(JTAGCONFIG	${CMAKE_SOURCE_DIR}/olimex-arm-usb-tiny-h.cfg)
set(BOARDCONFIG ${CMAKE_SOURCE_DIR}/stm32f4x.cfg)

find_program(OPENOCD openocd HINT ../../sat/bin/)

enable_testing()

set(CPACK_PACKAGE_CONTACT "px4users@googlegroups.com")
set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY 0)
set(CPACK_PACKAGE_VERSION_MAJOR "0") 
set(CPACK_PACKAGE_VERSION_MINOR "0") 
set(CPACK_PACKAGE_VERSION_PATCH "0") 
include(CPack)

message(STATUS "building for: ${CMAKE_SYSTEM_NAME}")

if (${CMAKE_SYSTEM_NAME} STREQUAL "Arm")
    # and linker script if compiling for board
    foreach(ARG in EXE TEST)
        set(CMAKE_${ARG}_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T${CMAKE_SOURCE_DIR}/stm32f4.ld")
    endforeach()
    set(COMMON_FLAGS "-mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Wall -Wextra -MD")
else()
    set(COMMON_FLAGS "")
endif()
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${COMMON_FLAGS} --std=gnu99 -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COMMON_FLAGS}")
add_definitions(-DMAVLINK_SEND_UART_BYTES=mavlink_send_uart_bytes)

macro(generate_firmware NAME)
    add_custom_target(firmware_${NAME} ALL DEPENDS ${NAME}.px4)
    add_custom_command(OUTPUT ${NAME}.px4
        COMMAND ${OBJCOPY} --output-format=binary ${NAME} ${NAME}.bin
        COMMAND python -u ${CMAKE_SOURCE_DIR}/Tools/px_mkfw.py --board_id 6 > ${NAME}_prototype.px4
        COMMAND python -u ${CMAKE_SOURCE_DIR}/Tools/px_mkfw.py --prototype ${NAME}_prototype.px4 --image ${NAME}.bin > ${NAME}.px4
        DEPENDS ${NAME})
endmacro()

include_directories(
    inc lib
    lib/STM32F4xx_StdPeriph_Driver/inc
    lib/STM32_USB_Device_Library/Class/cdc/inc
    lib/STM32_USB_Device_Library/Core/inc
    lib/STM32_USB_HOST_Library/Core/inc
    lib/STM32_USB_OTG_Driver/inc
    mavlink/include/mavlink/v1.0
    mavlink/include/mavlink/v1.0/common
    )

# source
#-----------------------------------------------------
set(PX4FLOW_HDRS
    inc/communication.h
    inc/dcmi.h
    inc/debug.h
    inc/flow.h
    inc/gyro.h
    inc/i2c_frame.h
    inc/i2c.h
    inc/led.h
    inc/main.h
    inc/mavlink_bridge_header.h
    inc/mt9v034.h
    inc/settings.h
    inc/sonar.h
    inc/sonar_mode_filter.h
    inc/stm32f4xx_conf.h
    inc/stm32f4xx_it.h
    inc/usart.h
    inc/usb_conf.h
    inc/usbd_cdc_vcp.h
    inc/usbd_conf.h
    inc/usbd_desc.h
    inc/utils.h
    )
set(PX4FLOW_SRC
    lib/startup_stm32f4xx.s
    lib/STM32F4xx_StdPeriph_Driver/src/misc.c
    lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rcc.c
    lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dma.c
    lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dcmi.c
    lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_i2c.c
    lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_tim.c
    lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_spi.c
    lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_usart.c
    lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_adc.c
    lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_gpio.c
    lib/STM32_USB_OTG_Driver/src/usb_core.c
    lib/STM32_USB_OTG_Driver/src/usb_dcd_int.c
    lib/STM32_USB_OTG_Driver/src/usb_dcd.c
    lib/STM32_USB_Device_Library/Core/src/usbd_core.c
    lib/STM32_USB_Device_Library/Core/src/usbd_req.c
    lib/STM32_USB_Device_Library/Core/src/usbd_ioreq.c
    lib/STM32_USB_Device_Library/Class/cdc/src/usbd_cdc_core.c
    src/communication.c
    src/dcmi.c
    src/debug.c
    src/flow.c
    src/gyro.c
    src/i2c.c
    src/led.c
    src/main.c
    src/mt9v034.c
    src/reset.c
    src/settings.c
    src/sonar.c
    src/sonar_mode_filter.c
    src/stm32f4xx_it.c
    src/system_stm32f4xx.c
    src/usart.c
    src/usb_bsp.c
    src/usbd_cdc_vcp.c
    src/usbd_desc.c
    src/usbd_usr.c
    src/utils.c
    src/system_stm32f4xx.c
    src/stm32f4xx_it.c
    )

# building
#-----------------------------------------------------
if (${CMAKE_SYSTEM_NAME} STREQUAL "Arm")
    # board executable
    add_executable(px4flow ${PX4FLOW_SRC} ${PX4FLOW_HDRS})
    target_link_libraries(px4flow nosys m)
    generate_firmware(px4flow)
else()
    # host based testing
    include_directories(unittests)
    add_executable(sonar_mode_filter
        src/sonar_mode_filter.c
        inc/sonar_mode_filter.h
        unittests/tests.c
        )
    add_test(sonar_mode_filter sonar_mode_filter)
endif()

# flashing
#-----------------------------------------------------
add_custom_target(flash
    COMMAND ${OPENOCD} --search ../px4_flow -f${JTAGCONFIG} -f${BOARDCONFIG} -c"init; reset halt; flash write_image erase px4flow.elf; reset run; shutdown")

add_custom_target(flash-bootloader
    COMMAND ${OPENOCD} --search ../px4_flow -f${JTAGCONFIG} -f${BOARDCONFIG} -c"init; reset halt; flash write_image erase px4flow_bl.elf; reset run; shutdown")

add_custom_target(flash-both
    COMMAND ${OPENOCD} --search ../px4_flow -f${JTAGCONFIG} -f${BOARDCONFIG} -c"init; reset halt; flash write_image erase px4flow.elf; reset run" -c"init; reset halt; flash write_image erase px4flow_bl.elf; reset run; shutdown")

# for debugging with GDB
add_custom_target(flash-both-no-shutdown
    COMMAND ${OPENOCD} --search ../px4_flow -f${JTAGCONFIG} -f${BOARDCONFIG} -c"init; reset halt; flash write_image erase px4flow.elf; reset run" -c"init; reset halt; flash write_image erase px4flow_bl.elf; reset run")

if (${CMAKE_HOST_SYSTEM_NAME} STREQUAL "Linux")
    set(SERIAL_PORTS "/dev/ttyACM5,/dev/ttyACM4,/dev/ttyACM3,/dev/ttyACM2,/dev/ttyACM1,/dev/ttyACM0")
elseif (${CMAKE_HOST_APPLE})
    set(SERIAL_PORTS "/dev/tty.usbmodemPX1,/dev/tty.usbmodemPX2,/dev/tty.usbmodemPX3,/dev/tty.usbmodemPX4,/dev/tty.usbmodem1,/dev/tty.usbmodem2,/dev/tty.usbmodem3,/dev/tty.usbmodem4")
else()
    set(SERIAL_PORTS "\\\\.\\COM32,\\\\.\\COM31,\\\\.\\COM30,\\\\.\\COM29,\\\\.\\COM28,\\\\.\\COM27,\\\\.\\COM26,\\\\.\\COM25,\\\\.\\COM24,\\\\.\\COM23,\\\\.\\COM22,\\\\.\\COM21,\\\\.\\COM20,\\\\.\\COM19,\\\\.\\COM18,\\\\.\\COM17,\\\\.\\COM16,\\\\.\\COM15,\\\\.\\COM14,\\\\.\\COM13,\\\\.\\COM12,\\\\.\\COM11,\\\\.\\COM10,\\\\.\\COM9,\\\\.\\COM8,\\\\.\\COM7,\\\\.\\COM6,\\\\.\\COM5,\\\\.\\COM4,\\\\.\\COM3,\\\\.\\COM2,\\\\.\\COM1,\\\\.\\COM0")
endif()

add_custom_target(upload-usb
    COMMAND echo "Attempting to flash PX4FLOW board via USB ${SERIAL_PORTS}"
    COMMAND python -u ${CMAKE_SOURCE_DIR}/Tools/px_uploader.py px4flow.px4 --baud 921600 --port ${SERIAL_PORTS}
    DEPENDS px4flow.px4)

# vim: set et fenc= ff=unix sts=0 sw=4 ts=4 ft=cmake :
