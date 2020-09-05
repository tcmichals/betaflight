#include <stdbool.h>
#include <stdint.h>
#include <string.h>

extern "C"
{
#include "platform.h"
}

#if defined(USE_SPI)

extern "C"
{
#include "drivers/bus.h"
#include "drivers/bus_spi.h"
#include "drivers/bus_spi_impl.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/rcc.h"
#include "drivers/dma_reqmap.h"
#include "drivers/io.h"


}

#include "linux/limits.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <sched.h>

/*
 * 
 * 
#define SPI_MODE_0		(0|0)
#define SPI_MODE_1		(0|SPI_CPHA)
#define SPI_MODE_2		(SPI_CPOL|0)
#define SPI_MODE_3	(SPI_CPOL|SPI_CPHA)
 * 
 * */

#define _SPI_MODE SPI_MODE_0
#define _SPI_WORDSIZE 8
#define _SPI_SPEED 1000000
#define _BUFFER_SIZE 64

typedef struct
{
    int fd;
    int errorCount;
    const char *path;

} linuxSPIDevice_t;

static linuxSPIDevice_t _spiDevice[SPIDEV_COUNT] =
{
        {
            .fd = -1,
            .errorCount = 0,
            .path = "/dev/spidev0.0",
        },
        {
            .fd = -1,
            .errorCount = 0,
            .path = "/dev/spidev0.1",
        },
        {
            .fd = -1,
            .errorCount = 0,
            .path = "/dev/spidev0.0",
        },
        {
            .fd = -1,
            .errorCount = 0,
            .path = "/dev/spidev0.1",
        },
};


SPIDevice spiDeviceByInstance(SPI_TypeDef *instance)
{
#ifdef USE_SPI_DEVICE_1
    if (instance == SPI_DEV_1_INSTANCE)
        return SPIDEV_1;
#endif

#ifdef USE_SPI_DEVICE_2
    if (instance == SPI2)
        return SPIDEV_2;
#endif

#ifdef USE_SPI_DEVICE_3
    if (instance == SPI3)
        return SPIDEV_3;
#endif

#ifdef USE_SPI_DEVICE_4
    if (instance == SPI4)
        return SPIDEV_4;
#endif

    return SPIINVALID;
}

SPI_TypeDef *spiInstanceByDevice(SPIDevice device)
{
    SPI_TypeDef *rc = nullptr;
    if (device >= SPIDEV_COUNT)
    {
        return rc;
    }
    switch(device)
    {
        #ifdef USE_SPI_DEVICE_1
        case SPIDEV_1:
            rc = SPI_DEV_1_INSTANCE;
            break;
#endif
        default:
            rc = nullptr;
        break;

    }
    return rc;
}

void spiInitDevice(SPIDevice device)
{
    linuxSPIDevice_t *spi = &(_spiDevice[device]);

    if (spi->fd >= 0)
        return;

    spi->fd = open(spi->path, O_RDWR);

    if (spi->fd < 0)
        return;

    int wordsize = _SPI_WORDSIZE;
    int speed = _SPI_SPEED;
    int mode = _SPI_MODE;

    // Configure SPI transfer mode (clock polarity and phase)
    if (ioctl(spi->fd, SPI_IOC_WR_MODE, &mode) < 0)
    {
        close(spi->fd);
        spi->fd = -1;
        return;
    }

    if (ioctl(spi->fd, SPI_IOC_RD_MODE, &mode) < 0)
    {
        close(spi->fd);
        spi->fd = -1;
        return;
    }

    if (ioctl(spi->fd, SPI_IOC_WR_BITS_PER_WORD, &wordsize) < 0)
    {
        close(spi->fd);
        spi->fd = -1;
        return;
    }

    if (ioctl(spi->fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0)
    {
        close(spi->fd);
        spi->fd = -1;
        return;
    }

    return;
}

bool spiInit(SPIDevice device, bool leadingEdge)
{
    switch (device)
    {
    case SPIINVALID:
        return false;
    case SPIDEV_1:
#ifdef USE_SPI_DEVICE_1
        spiInitDevice(device);
        return true;
#else
        break;
#endif
    case SPIDEV_2:
#ifdef USE_SPI_DEVICE_2
        spiInitDevice(device);
        return true;
#else
        break;
#endif
    case SPIDEV_3:
#if defined(USE_SPI_DEVICE_3) && (defined(STM32F303xC) || defined(STM32F4))
        spiInitDevice(device);
        return true;
#else
        break;
#endif
    case SPIDEV_4:
#if defined(USE_SPI_DEVICE_4)
        spiInitDevice(device);
        return true;
#else
        break;
#endif
    }
    return false;
}

uint32_t spiTimeoutUserCallback(SPI_TypeDef *instance)
{
    SPIDevice device = spiDeviceByInstance(instance);
    if (device == SPIINVALID)
    {
        return -1;
    }

    linuxSPIDevice_t *spi = &(_spiDevice[device]);
    return ++spi->errorCount;
}

// return uint8_t value or -1 when failure
uint8_t spiTransferByte(SPI_TypeDef *instance, uint8_t txByte)
{
    SPIDevice device = spiDeviceByInstance(instance);
    linuxSPIDevice_t *spi = &(_spiDevice[device]);

    if (spi->fd < 0)
        return 0;

    struct spi_ioc_transfer spi_transfer;
    uint8_t rx = 0xFF;

    memset(&spi_transfer, 0, sizeof(spi_transfer));

    spi_transfer.tx_buf = (__u64)&txByte;
    spi_transfer.rx_buf = (__u64)&rx;
    spi_transfer.len = 1;
    spi_transfer.speed_hz = 0;
    spi_transfer.bits_per_word = _SPI_WORDSIZE;

    ioctl(spi->fd, SPI_IOC_MESSAGE(1), &spi_transfer);

    return rx;
}

/**
 * Return true if the bus is currently in the middle of a transmission.
 */
bool spiIsBusBusy(SPI_TypeDef *instance)
{
    UNUSED(instance);
    return false;
}

bool spiTransfer(SPI_TypeDef *instance,
                 const uint8_t *txData,
                 uint8_t *rxData,
                 int len)
{
    SPIDevice device = spiDeviceByInstance(instance);
    linuxSPIDevice_t *spi = &(_spiDevice[device]);

    if (spi->fd < 0)
        return false;

    struct spi_ioc_transfer spi_transfer;

    memset(&spi_transfer, 0, sizeof(spi_transfer));

    spi_transfer.tx_buf = (__u64)txData;
    spi_transfer.rx_buf = (__u64)rxData;
    spi_transfer.len = len;
    spi_transfer.delay_usecs = 0;
    spi_transfer.speed_hz = 0;
    spi_transfer.bits_per_word = _SPI_WORDSIZE;

    ioctl(spi->fd, SPI_IOC_MESSAGE(1), &spi_transfer);
    return true;
}

#include "build/debug.h"

bool spiBusTransfer(const busDevice_t *bus,
                    const uint8_t *txData,
                    uint8_t *rxData,
                    int length)
{
    return spiTransfer(bus->busdev_u.spi.instance, txData, rxData, length);
}

void spiSetDivisor(SPI_TypeDef *instance, uint16_t divisor)
{

    int ret;
    uint32_t speed;
    switch (divisor)
    {
    case SPI_CLOCK_SLOW:
    case SPI_CLOCK_STANDARD:
        speed = 1000000;
        break;
    default:
        speed = 25000000;
        break;
    }

    printf("%s spi_baud=%d\n", __FUNCTION__, speed);
    SPIDevice device = spiDeviceByInstance(instance);
    linuxSPIDevice_t *spi = &(_spiDevice[device]);

    ret = ioctl(spi->fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    if (ret == -1)
    {
        printf("ERROR: %s\n", strerror(errno));
    }

    ret = ioctl(spi->fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
    if (ret == -1)
    {
        printf("ERROR: %s\n", strerror(errno));
    }
    return;
}

uint16_t spiGetErrorCounter(SPI_TypeDef *instance)
{
    SPIDevice device = spiDeviceByInstance(instance);
    if (device == SPIINVALID)
        return 0;

    linuxSPIDevice_t *spi = &(_spiDevice[device]);

    return spi->errorCount;
}

void spiResetErrorCounter(SPI_TypeDef *instance)
{
    SPIDevice device = spiDeviceByInstance(instance);
    if (device != SPIINVALID)
    {
        linuxSPIDevice_t *spi = &(_spiDevice[device]);
        spi->errorCount = 0;
        ;
    }
}

bool spiBusWriteRegister(const busDevice_t *bus, uint8_t reg, uint8_t data)
{
    uint8_t tx[2] = {reg, data};
    uint8_t rx[2] = {0, 0};

    return spiTransfer(bus->busdev_u.spi.instance, tx, rx, sizeof(tx));
}
bool spiBusRawReadRegisterBuffer(const busDevice_t *bus, uint8_t reg, uint8_t *data, uint8_t length)
{
    uint8_t tx_reg[255] = {0};
    uint8_t rx_reg[255] = {0};
    tx_reg[0] = reg;
    int _spiLen = length + 1;

    bool rc = spiTransfer(bus->busdev_u.spi.instance, tx_reg, rx_reg, _spiLen);

    if (rc)
        memcpy(data, rx_reg + 1, length);

    return rc;
}


bool spiBusReadRegisterBuffer(const busDevice_t *bus,
                              uint8_t reg,
                              uint8_t *data,
                              uint8_t length)
{
#if 0
    uint8_t tx_reg[255] = {0};
    uint8_t rx_reg[255] = {0};
    tx_reg[0] = reg | 0x80u;
    int _spiLen = length + 1;

    bool rc = spiTransfer(bus->busdev_u.spi.instance, tx_reg, rx_reg, _spiLen);

    if (rc)
        memcpy(data, rx_reg + 1, length);

    return rc;
#else
    return spiBusRawReadRegisterBuffer(bus, reg | 0x80, data, length);
#endif
}

uint8_t spiBusReadRegister(const busDevice_t *bus, uint8_t reg)
{
    uint8_t data;
    spiBusReadRegisterBuffer(bus, reg, &data, 1);
    return data;
}

void spiBusSetInstance(busDevice_t *bus, SPI_TypeDef *instance)
{
    bus->busdev_u.spi.csnPin = 0;
    bus->busdev_u.spi.instance = instance;
}

void spiPinConfigure(const struct spiPinConfig_s *pConfig)
{
        UNUSED(pConfig);

}
void spiPreinitByIO(IO_t io)
{
        UNUSED(io);

}
void spiPreinit(void){}
//void spiPinConfigure(const spiPinConfig_t *pConfig){}
void spiPreinitByTag(ioTag_t tag)
{     
    UNUSED(tag);
}

void spiPreInitCs(ioTag_t iotag)
{
    UNUSED(iotag);
}


void spiPreinitRegister(ioTag_t iotag, uint8_t iocfg, uint8_t init)
{
    UNUSED(iotag);
    UNUSED(iocfg);
    UNUSED(init);
}
#endif
