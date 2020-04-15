

/*
 * Authors:
tcmichals
*/

#include <stdbool.h>
#include <stdint.h>

extern "C"
{
#include "platform.h"
}

#ifdef USE_UART

extern "C"
{
#include "build/build_config.h"
#include "common/utils.h"
#include "drivers/rcc.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"

#include "pg/serial_uart.h"
}

#include <vector>
#include <thread>
#include <mutex>
#include <memory.h>
#include <boost/asio.hpp>
#include "target_linux.h"
#include "port.h"
#include "linux_serial.h"
#include "linux_tcpServerSocket.h"


serialPort_t serialPorts[10];
std::unique_ptr<tcpServer> gTCPServer;
std::vector<std::unique_ptr<LinuxSerialPort> > m_linuxSerialPorts;



/**
 * @brief 
 * 
 * @param id 
 * @param rxCallback 
 * @param rxCallbackData 
 * @param baudRate 
 * @param mode 
 * @param options 
 * @return serialPort_t* 
 */
serialPort_t *uartOpen( UARTDevice_e id, 
                        serialReceiveCallbackPtr rxCallback, 
                        void *rxCallbackData, 
                        uint32_t baudRate, 
                        portMode_e mode, 
                        portOptions_e options)
{
    UNUSED(rxCallback);
    UNUSED(rxCallbackData);
    UNUSED(baudRate);
    UNUSED(mode);
    UNUSED(options);

    serialPort_t *s = nullptr;
    switch(id)
    {
        case UARTDEV_1:
            gTCPServer = std::make_unique<tcpServer>(serialPorts[0], getIOService(), 56001);
            s = &serialPorts[0];
        break;

        case UARTDEV_2:
        {
            auto serialPort = std::make_unique<LinuxSerialPort>(serialPorts[1], getIOService());

        }
        break;

        default:
        break;
    }



    if (!s)
        return nullptr;

    return (serialPort_t *)s;
}


#endif // USE_UART
