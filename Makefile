#
# STM32F4 PX4FLOW board build rules
#

BINARY		 = px4flow.elf
BINARY_BIN	 = px4flow.bin

OPENOCD		?= ../../sat/bin/openocd
JTAGCONFIG 	?= olimex-jtag-tiny.cfg
#JTAGCONFIG	?= olimex-arm-usb-tiny-h.cfg

MAVLINKBASEDIR = mavlink/include/mavlink/v1.0
MAVLINKDIR = mavlink/include/mavlink/v1.0/pixhawk
MAVLINKUSERDIR = mavlink/include/mavlink/v1.0/user

AS =		arm-none-eabi-as
CC =		arm-none-eabi-gcc
OBJCOPY =	arm-none-eabi-objcopy

SRCS = 		src/main.c \
			src/utils.c \
			src/led.c \
			src/settings.c \
			src/communication.c \
			src/flow.c \
			src/dcmi.c \
			src/mt9v034.c \
			src/gyro.c \
			src/usart.c \
			src/sonar.c \
			src/debug.c \
			src/usb_bsp.c \
			src/usbd_cdc_vcp.c \
			src/usbd_desc.c \
			src/usbd_usr.c \
			src/i2c.c \
			src/reset.c
SRCS += 	src/system_stm32f4xx.c src/stm32f4xx_it.c lib/startup_stm32f4xx.s
SRCS += 	lib/STM32F4xx_StdPeriph_Driver/src/misc.c \
			lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rcc.c \
			lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dma.c \
			lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dcmi.c \
			lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_i2c.c \
			lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_tim.c \
			lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_spi.c \
			lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_usart.c \
			lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_adc.c \
			lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_gpio.c \
			lib/STM32_USB_OTG_Driver/src/usb_core.c \
			lib/STM32_USB_OTG_Driver/src/usb_dcd_int.c \
			lib/STM32_USB_OTG_Driver/src/usb_dcd.c \
			lib/STM32_USB_Device_Library/Core/src/usbd_core.c \
			lib/STM32_USB_Device_Library/Core/src/usbd_req.c \
			lib/STM32_USB_Device_Library/Core/src/usbd_ioreq.c \
			lib/STM32_USB_Device_Library/Class/cdc/src/usbd_cdc_core.c

.PHONY: clean upload-usb

CFLAGS		 = -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 \
			-O3 \
			-std=gnu99 \
			-Wall \
			-MD \
			-Iinc \
			-Ilib \
			-Ilib/STM32F4xx_StdPeriph_Driver/inc \
			-Ilib/STM32_USB_Device_Library/Class/cdc/inc \
			-Ilib/STM32_USB_Device_Library/Core/inc \
			-Ilib/STM32_USB_HOST_Library/Core/inc \
			-Ilib/STM32_USB_OTG_Driver/inc \
			-I$(MAVLINKBASEDIR) \
			-I$(MAVLINKDIR) \
			-DMAVLINK_SEND_UART_BYTES=mavlink_send_uart_bytes

LDFLAGS		 = -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 \
			-lnosys \
			-Tstm32f4.ld \
			-Wl,-gc-sections \
			-lm

all:		$(BINARY) objcopy

$(BINARY):	$(SRCS)
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)
	
objcopy:
	@$(OBJCOPY) -O binary px4flow.elf px4flow.bin
	@python -u Tools/px_mkfw.py --board_id 6 > px4flow_prototype.px4
	@python -u Tools/px_mkfw.py --prototype px4flow_prototype.px4 --image $(BINARY_BIN) > px4flow.px4

clean:
	rm -f *.o *.d $(BINARY) $(BINARY_BIN)

upload-jtag:		all flash-both

flash:
	$(OPENOCD) --search ../px4_flow -f $(JTAGCONFIG) -f stm32f4x.cfg  -c init -c "reset halt" -c "flash write_image erase px4flow.elf" -c "reset run" -c shutdown

flash-bootloader:
	$(OPENOCD) --search ../px4_flow -f $(JTAGCONFIG) -f stm32f4x.cfg  -c init -c "reset halt" -c "flash write_image erase px4flow_bl.elf" -c "reset run" -c shutdown

flash-both:
	$(OPENOCD) --search ../px4_flow -f $(JTAGCONFIG) -f stm32f4x.cfg  -c init -c "reset halt" -c "flash write_image erase px4flow.elf" -c "reset run" -c init -c "reset halt" -c "flash write_image erase px4flow_bl.elf" -c "reset run" -c shutdown

# FOR GDB
flash-both-no-shutdown:
	$(OPENOCD) --search ../px4_flow -f $(JTAGCONFIG) -f stm32f4x.cfg  -c init -c "reset halt" -c "flash write_image erase px4flow.elf" -c "reset run" -c init -c "reset halt" -c "flash write_image erase px4flow_bl.elf" -c "reset run"

# serial port defaults by operating system.
SYSTYPE			 = $(shell uname)
ifeq ($(SYSTYPE),Darwin)
SERIAL_PORTS		?= "/dev/tty.usbmodemPX1,/dev/tty.usbmodemPX2,/dev/tty.usbmodemPX3,/dev/tty.usbmodemPX4,/dev/tty.usbmodem1,/dev/tty.usbmodem2,/dev/tty.usbmodem3,/dev/tty.usbmodem4"
endif
ifeq ($(SYSTYPE),Linux)
SERIAL_PORTS		?= "/dev/ttyACM5,/dev/ttyACM4,/dev/ttyACM3,/dev/ttyACM2,/dev/ttyACM1,/dev/ttyACM0"
endif
ifeq ($(SERIAL_PORTS),)
SERIAL_PORTS		 = "\\\\.\\COM32,\\\\.\\COM31,\\\\.\\COM30,\\\\.\\COM29,\\\\.\\COM28,\\\\.\\COM27,\\\\.\\COM26,\\\\.\\COM25,\\\\.\\COM24,\\\\.\\COM23,\\\\.\\COM22,\\\\.\\COM21,\\\\.\\COM20,\\\\.\\COM19,\\\\.\\COM18,\\\\.\\COM17,\\\\.\\COM16,\\\\.\\COM15,\\\\.\\COM14,\\\\.\\COM13,\\\\.\\COM12,\\\\.\\COM11,\\\\.\\COM10,\\\\.\\COM9,\\\\.\\COM8,\\\\.\\COM7,\\\\.\\COM6,\\\\.\\COM5,\\\\.\\COM4,\\\\.\\COM3,\\\\.\\COM2,\\\\.\\COM1,\\\\.\\COM0"
endif

upload-usb:
	@echo Attempting to flash PX4FLOW board via USB
	@python -u Tools/px_mkfw.py --board_id 6 > px4flow_prototype.px4
	@python -u Tools/px_mkfw.py --prototype px4flow_prototype.px4 --image $(BINARY_BIN) > px4flow.px4
	@python -u Tools/px_uploader.py px4flow.px4 --baud 921600 --port $(SERIAL_PORTS)


#-include $(DEPS)
