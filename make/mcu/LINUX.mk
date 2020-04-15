
INCLUDE_DIRS    := $(INCLUDE_DIRS) 


MCU_COMMON_SRC  := 

#Flags
ARCH_FLAGS      =
DEVICE_FLAGS    =
LD_SCRIPT       = src/main/target/SITL/pg.ld
STARTUP_SRC     =

TARGET_FLAGS    = -D$(TARGET)
TARGET_FLASH   := 2048
MCU_FLASH_SIZE     := 2048

ifneq ($(CROSS_PREFIX),)
ARM_SDK_PREFIX =
else
ARM_SDK_PREFIX := $(CROSS_PREFIX)
endif



MCU_EXCLUDES = \
            drivers/adc.c \
            drivers/bus_spi.c \
            drivers/bus_spi_config.c \
            drivers/bus_spi_pinconfig.c \
            drivers/dma.c \
            drivers/pwm_output.c \
            drivers/timer.c \
            drivers/system.c \
            drivers/rcc.c \
            drivers/serial_escserial.c \
            drivers/serial_uart.c \
            drivers/serial_uart_init.c \
            drivers/serial_uart_pinconfig.c \
            drivers/rx/rx_xn297.c \
            drivers/display_ug2864hsweg01.c \
            telemetry/crsf.c \
            telemetry/srxl.c \
            io/displayport_oled.c \
            common/printf_serial.c \
            drivers/serial_tcp.c \
            drivers/serial_uart.c \
            drivers/exti.c \
            io/pidaudio.c \
            rx/srxl2.c \
            rx/rx_bind.c \
            drivers/rx/rx_pwm.c


TARGET_MAP  = $(OBJECT_DIR)/$(FORKNAME)_$(TARGET).map

LD_FLAGS    := \
              -lm \
              -lpthread \
              -lrt \
              -lftdi1 \
              -T$(LD_SCRIPT)


ifneq ($(DEBUG),GDB)
OPTIMISE_DEFAULT    := -Ofast
OPTIMISE_SPEED      := -Ofast
OPTIMISE_SIZE       := -Os

LTO_FLAGS           := $(OPTIMISATION_BASE) $(OPTIMISE_SPEED)
endif
