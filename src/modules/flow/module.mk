#
# Flow Code as it
#
DEFAULT_VISIBILITY=default

ST_LIB_VERSION=v1.0.2

ST_LIB = $(ST_LIB_DIR)$(ST_LIB_VERSION)/

INCLUDE_DIRS	 += $(MAVLINK_SRC) \
									$(MAVLINK_SRC)common \
									$(ST_LIB)STM32F4xx_StdPeriph_Driver/inc \
									$(ST_LIB)STM32_USB_Device_Library/Class/cdc/inc \
									$(ST_LIB)STM32_USB_Device_Library/Core/inc \
									$(ST_LIB)STM32_USB_HOST_Library/Core/inc \
									$(ST_LIB)STM32_USB_OTG_Driver/inc \


SRCS = 	$(ST_LIB)startup_stm32f4xx.s \
					system_stm32f4xx.c \
					stm32f4xx_it.c 
					

SRCS += 		main.c \
          utils.c \
          led.c \
          settings.c \
          communication.c \
          flow.c \
          dcmi.c \
          mt9v034.c \
          gyro.c \
          usart.c \
          sonar.c \
          debug.c \
          usb_bsp.c \
          usbd_cdc_vcp.c \
          usbd_desc.c \
          usbd_usr.c \
          i2c.c \
          reset.c \
          sonar_mode_filter.c

SRCS += 	$(ST_LIB)STM32F4xx_StdPeriph_Driver/src/misc.c \
    			$(ST_LIB)STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rcc.c \
    			$(ST_LIB)STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dma.c \
    			$(ST_LIB)STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dcmi.c \
    			$(ST_LIB)STM32F4xx_StdPeriph_Driver/src/stm32f4xx_i2c.c \
    			$(ST_LIB)STM32F4xx_StdPeriph_Driver/src/stm32f4xx_tim.c \
    			$(ST_LIB)STM32F4xx_StdPeriph_Driver/src/stm32f4xx_spi.c \
    			$(ST_LIB)STM32F4xx_StdPeriph_Driver/src/stm32f4xx_usart.c \
    			$(ST_LIB)STM32F4xx_StdPeriph_Driver/src/stm32f4xx_adc.c \
    			$(ST_LIB)STM32F4xx_StdPeriph_Driver/src/stm32f4xx_gpio.c \
    			$(ST_LIB)STM32_USB_OTG_Driver/src/usb_core.c \
    			$(ST_LIB)STM32_USB_OTG_Driver/src/usb_dcd_int.c \
    			$(ST_LIB)STM32_USB_OTG_Driver/src/usb_dcd.c \
    			$(ST_LIB)STM32_USB_Device_Library/Core/src/usbd_core.c \
    			$(ST_LIB)STM32_USB_Device_Library/Core/src/usbd_req.c \
    			$(ST_LIB)STM32_USB_Device_Library/Core/src/usbd_ioreq.c \
    			$(ST_LIB)STM32_USB_Device_Library/Class/cdc/src/usbd_cdc_core.c

$(info SRCS=$(SRCS))
