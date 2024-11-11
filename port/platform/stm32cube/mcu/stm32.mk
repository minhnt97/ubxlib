# Include ubxlib src and inc
UBXLIB_BASE ?= $(realpath $(MAKEFILE_PATH)/../../../../../..)
UBXLIB_FEATURES ?= cell gnss short_range
$(info UBXLIB_FEATURES is "${UBXLIB_FEATURES}")

# ubxlib.mk will define the following for us:
# UBXLIB_INC
# UBXLIB_PRIVATE_INC
# UBXLIB_SRC
# UBXLIB_TEST_SRC
# UBXLIB_TEST_INC
include $(UBXLIB_BASE)/port/ubxlib.mk

# Assume STM32F4 for backwards compatibility
STM32_MCU_FAMILY ?= stm32f4

PLATFORM_PATH = $(UBXLIB_BASE)/platform/stm32cube/mcu/$(STM32_MCU_FAMILY)

# Ubxlib port
UBXLIB_SRC += \
	$(UBXLIB_BASE)/port/clib/u_port_clib_mktime64.c \
	$(UBXLIB_BASE)/port/u_port_timezone.c \
	$(PLATFORM_PATH)/src/u_port_debug.c \
	$(PLATFORM_PATH)/src/u_port_gpio.c \
	$(PLATFORM_PATH)/src/u_port_private.c \
	$(PLATFORM_PATH)/src/u_port_uart.c \
	$(PLATFORM_PATH)/src/u_port_i2c.c \
	$(PLATFORM_PATH)/src/u_port_spi.c \
	$(PLATFORM_PATH)/src/u_port.c

# Assume STM32F437VG for backwards compatibility
STARTUP_ASSEMBLER_FILE ?= startup_stm32f437vgtx.s

UBXLIB_ASM += \
	$(PLATFORM_PATH)/src/$(STARTUP_ASSEMBLER_FILE)

UBXLIB_INC += \
	$(UBXLIB_PRIVATE_INC) \
	$(UBXLIB_BASE)/port/clib \
	$(PLATFORM_PATH)/src \
	$(PLATFORM_PATH)

