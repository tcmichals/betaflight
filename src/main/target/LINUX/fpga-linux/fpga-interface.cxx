#include <iostream>
#include <cstdint>
#include <thread>
#include <memory>

#include "serialTransport.h"
#include "avalonProtocol.h"
#include "loopback.h"
#include "timer.h"
#include "sysid.h"
#include "gpioled.h"
#include "pwmdecoder.h"
#include "pwmOut.h"
#include "apa102.h"



static ftdiTransPort transPort;
static sysIDAvalon sysid(transPort);
static avalonTimer timer(transPort);
static gpioAvalon gpioLED(transPort);
static pwmDecoderAvalon pwmDecoder(transPort);
static pwmOutAvalon pwmOut(transPort);
static apa102LED ledBar(transPort);

extern bool pwmCallback(   uint32_t channel_1,
                    uint32_t channel_2,
                    uint32_t channel_3,
                    uint32_t channel_4,
                    uint32_t channel_5,
                    uint32_t channel_6);

bool startFPGA()
{
    if (false == transPort.start())
    {
        std::cerr << "Error: cannot open()" << std::endl;
        return false;
    }

        pwmDecoder.updateCalllback( [](uint32_t channel_1,
                    uint32_t channel_2,
                    uint32_t channel_3,
                    uint32_t channel_4,
                    uint32_t channel_5,
                    uint32_t channel_6) -> bool
                    {
                     return pwmCallback(channel_1,
                                channel_2,
                                channel_3,
                                channel_4,
                                channel_5,
                                channel_6);
                                
                                

                    });

    return true;
}

extern "C" void linuxLedSet(int led, bool val)
{
    switch(led)
    {
        case 0:
            led = 1;
        break;
        case 1:
            led = 8;
        break;
        case 2:
            led = 0x80;
        break;
    }

    if (val)
    {
        gpioLED.setLED(led);
    }
    else
    {

        gpioLED.clearLED(led);
    }
    
}
extern "C" void linuxLedToggle(int led)
{
    switch(led)
    {
        case 0:
            led = 1;
        break;
        case 1:
            led = 8;
        break;
        case 2:
            led = 0x80;
        break;
    }

        gpioLED.toggleLED(led);
}



bool postReadPWM()
{
    return pwmDecoder.postRead();
}

