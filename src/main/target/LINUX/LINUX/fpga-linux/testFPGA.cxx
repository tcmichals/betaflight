#include <iostream>
#include <cstdint>
#include <thread>
#include "serialTransport.h"
#include "avalonProtocol.h"
#include "loopback.h"
#include "timer.h"
#include "sysid.h"
#include "gpioled.h"
#include "pwmdecoder.h"
#include "pwmOut.h"
#include "apa102.h"
//#include "dshot.h"


int main(int argc, char **argv)
{
    ftdiTransPort transPort;

    if (false == transPort.start())
    {
        std::cerr << "Error: cannot open()" << std::endl;
        return -1;
    }


    sysIDAvalon sysid(transPort);
    avalonTimer timer(transPort);
    gpioAvalon gpioLED(transPort);
    pwmDecoderAvalon pwmDecoder(transPort);
    pwmOutAvalon pwmOut(transPort);
    apa102LED   apaLED(transPort);
    // dshotAvalon   pwmOut(transPort);

    uint8_t ledValue = 0;
    int timeout = 0;
    uint16_t pwmValue = MIN_ON;
    #define ARM_COUNT 1000
    #define SPEED_INC  100
    typedef enum
    {
        NOT_ARMED = 0,
        ARM_ARMED =1,
    }state_t;
    int state = NOT_ARMED;
    #if 0

    do 
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        gpioLED.postLED(ledValue++);
      //  pwmOut.postPWMOut(pwmValue);

        timeout++;
        switch(state)
        {

            case NOT_ARMED:
                pwmValue = MIN_ON;
                if (timeout > ARM_COUNT)
                {
                    timeout = 0;
                    state = ARM_ARMED;
                    std::cout << "ARMED" << std::endl;
                }
            break;

            case ARM_ARMED:
                if ( timeout > SPEED_INC )
                {
                    timeout = 0;
                    pwmValue++;
                    if (pwmValue > 150)
                    {
                        timeout =0;
                        state = NOT_ARMED;
                          std::cout << "DISARMED" << std::endl;
                    }
                }



            break;
        }
        #if 0
        if ( timeout++ > COUNT)
        {
            if (pwmValue == MIN_ON)
            {
                pwmValue = MID_ON;
                std::cout << " MID_ON:" << pwmValue << std::endl;
            }
            else
            {
                pwmValue = MIN_ON;
                std::cout << "MIN_ON:" << pwmValue << std::endl;

            }
            timeout = 0;
        }
        #endif
        
    }while(true);
#else
auto pwmFunc = std::bind([](uint32_t val, const std::string str) -> void
{
    if (val & PWM_OFF_MASK)
    {
        std::cout << str << " OFF" << std::endl;
    }
    else
    {
        std::cout << str << " "<< (val & 0xFFFF) << std::endl;
    }

}, std::placeholders::_1, std::placeholders::_2);

pwmDecoder.updateCalllback( [&pwmFunc](uint32_t pwm1,
                                        uint32_t pwm2,
                                        uint32_t pwm3,
                                        uint32_t pwm4,
                                        uint32_t pwm5,
                                        uint32_t pwm6)-> void
                                    {
                                        pwmFunc(pwm1, "PWM1");
                                        pwmFunc(pwm2, "PWM2");
                                        pwmFunc(pwm3, "PWM3");
                                        pwmFunc(pwm4, "PWM4");
                                        pwmFunc(pwm5, "PWM5");
                                        pwmFunc(pwm6, "PWM6");
                                    }
                                    );
ledValue = 1;
gpioLED.setLED(ledValue);
std::this_thread::sleep_for(std::chrono::seconds(1));
ledValue = 2;
gpioLED.setLED(ledValue);
std::this_thread::sleep_for(std::chrono::seconds(1));
ledValue = 0x80;
gpioLED.setLED(ledValue);

apaLED.set_led(apa102LED::whichLED::LED_1, 0, 0, 0,0);
apaLED.set_led(apa102LED::whichLED::LED_2, 0, 0, 0,0);
apaLED.set_led(apa102LED::whichLED::LED_3, 0, 0, 0,0);
apaLED.set_led(apa102LED::whichLED::LED_4, 0, 0, 0,0);
apaLED.set_led(apa102LED::whichLED::LED_5, 0, 0, 0,0);
apaLED.set_led(apa102LED::whichLED::LED_6, 0, 0, 0,0);
apaLED.set_led(apa102LED::whichLED::LED_7, 0, 0, 0,0);
apaLED.set_led(apa102LED::whichLED::LED_8, 0, 0, 0,0);
apaLED.update();    
uint8_t red =0;
do {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    gpioLED.toggleLED(ledValue);

    pwmDecoder.postRead();
    if (red & 1)
        apaLED.set_led(apa102LED::whichLED::LED_1, 0xFF, 0, 0,0xFF);
    else
        apaLED.set_led(apa102LED::whichLED::LED_1, 0xFF, 0, 0xFF,0);
    red++;
        
    apaLED.update();
    

}while(true);


#endif


}


//eof
