LINUX_TARGETS += $(TARGET)
FEATURES       += #SDCARD_SPI VCP

TARGET_SRC = \
            drivers/accgyro/accgyro_fake.c \
            drivers/barometer/barometer_fake.c \
            drivers/compass/compass_fake.c \
            drivers/serial_tcp.c \
            serial_linux.cxx target_linux.cxx \
            i2c_linux.cxx spi_linux.cxx \
            drivers/accgyro/accgyro_mpu6500.c \
            drivers/accgyro/accgyro_spi_mpu6500.c \
            drivers/accgyro/accgyro_spi_mpu9250.c \
            drivers/accgyro/accgyro_mpu.c \
            drivers/compass/compass_hmc5883l.c \
            fpga-linux/fpga-interface.cxx \
            pwm_rx.cxx


