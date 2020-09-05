/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */
#include <iostream>
#include <stdbool.h>
#include <stdint.h>
#include <atomic>

extern "C"
{
#include "platform.h"
}

#if defined(USE_PWM) || defined(USE_PPM)

extern "C"
{
#include "build/build_config.h"
#include "build/debug.h"

#include "common/utils.h"

#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/pwm_output.h"
#include "drivers/timer.h"

#include "pg/rx_pwm.h"
}

#define PPM_CAPTURE_COUNT 6

#if PPM_CAPTURE_COUNT > PWM_INPUT_PORT_COUNT
#define PWM_PORTS_OR_PPM_CAPTURE_COUNT PPM_CAPTURE_COUNT
#else
#define PWM_PORTS_OR_PPM_CAPTURE_COUNT PWM_INPUT_PORT_COUNT
#endif

// TODO - change to timer clocks ticks
#define INPUT_FILTER_TO_HELP_WITH_NOISE_FROM_OPENLRS_TELEMETRY_RX 0x03

static inputFilteringMode_e inputFilteringMode;

void pwmICConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t polarity);

typedef enum {
    INPUT_MODE_PPM,
    INPUT_MODE_PWM
} pwmInputMode_e;

typedef struct {
    pwmInputMode_e mode;
    uint8_t channel; // only used for pwm, ignored by ppm

    uint8_t state;
    uint8_t missedEvents;

    captureCompare_t rise;
    captureCompare_t fall;
    captureCompare_t capture;

    const timerHardware_t *timerHardware;
    timerCCHandlerRec_t edgeCb;
    timerOvrHandlerRec_t overflowCb;
} pwmInputPort_t;

static pwmInputPort_t pwmInputPorts[PWM_INPUT_PORT_COUNT];

static uint16_t captures[PWM_PORTS_OR_PPM_CAPTURE_COUNT];

#define PPM_TIMER_PERIOD 0x10000
#define PWM_TIMER_PERIOD 0x10000

static uint8_t ppmFrameCount = 0;
static uint8_t lastPPMFrameCount = 0;
static uint8_t ppmCountDivisor = 1;

#define PWM_FAILED_READ 3
static std::atomic_int pwmWasRead = 0;

typedef struct ppmDevice_s {
    //uint32_t previousTime;
    uint32_t currentCapture;
    uint32_t currentTime;
    uint32_t deltaTime;
    uint32_t captures[PWM_PORTS_OR_PPM_CAPTURE_COUNT];
    uint32_t largeCounter;
    uint8_t  pulseIndex;
    int8_t   numChannels;
    int8_t   numChannelsPrevFrame;
    uint8_t  stableFramesSeenCount;

    bool     tracking;
    bool     overflowed;
} ppmDevice_t;

static ppmDevice_t ppmDev;

#define PPM_IN_MIN_SYNC_PULSE_US    2700    // microseconds
#define PPM_IN_MIN_CHANNEL_PULSE_US 750     // microseconds
#define PPM_IN_MAX_CHANNEL_PULSE_US 2250    // microseconds
#define PPM_STABLE_FRAMES_REQUIRED_COUNT    25
#define PPM_IN_MIN_NUM_CHANNELS     4
#define PPM_IN_MAX_NUM_CHANNELS     PWM_PORTS_OR_PPM_CAPTURE_COUNT

bool isPPMDataBeingReceived(void)
{
    return (ppmFrameCount != lastPPMFrameCount);
}

void resetPPMDataReceivedState(void)
{
    lastPPMFrameCount = ppmFrameCount;
}

#define MIN_CHANNELS_BEFORE_PPM_FRAME_CONSIDERED_VALID 4


typedef enum {
    SOURCE_OVERFLOW = 0,
    SOURCE_EDGE = 1
} eventSource_e;

typedef struct ppmISREvent_s {
    uint32_t capture;
    eventSource_e source;
} ppmISREvent_t;

static uint8_t ppmEventIndex = 0;


#define MAX_MISSED_PWM_EVENTS 10
extern bool postReadPWM();
bool isPWMDataBeingReceived(void)
{
    int channel;
    postReadPWM();  //Post a read to the FPGA 

    for (channel = 0; channel < PWM_PORTS_OR_PPM_CAPTURE_COUNT; channel++) 
    {
        if (captures[channel] != PPM_RCVR_TIMEOUT || pwmWasRead < PWM_FAILED_READ) 
        {
            return true;
        }
    }
    return false;
}


/**
 * @brief 
 * 
 * @param pwmConfig 
 */
void pwmRxInit(const pwmConfig_t *pwmConfig)
{
  
}

#define FIRST_PWM_PORT 0
/**
 * @brief 
 * 
 * @param ppmConfig 
 */
void ppmRxInit(const ppmConfig_t *ppmConfig)
{

}
/**
 * @brief 
 * 
 * @param channel 
 * @return uint16_t 
 */
uint16_t ppmRead(uint8_t channel)
{
    return captures[channel];
}

/**
 * @brief 
 * 
 * @param channel 
 * @return uint16_t 
 */
uint16_t pwmRead(uint8_t channel)
{
    // update the count the pwmCallback should clear it.
    pwmWasRead++;
    return captures[channel];
}

/**
 * @brief 
 * 
 * @param channel_1 
 * @param channel_2 
 * @param channel_3 
 * @param channel_4 
 * @param channel_5 
 * @param channel_6 
 * @return true 
 * @return false 
 */
bool pwmCallback(   uint32_t channel_1,
                    uint32_t channel_2,
                    uint32_t channel_3,
                    uint32_t channel_4,
                    uint32_t channel_5,
                    uint32_t channel_6)
{
    pwmWasRead =  0;
    captures[0] = channel_1;
    captures[1] = channel_2;
    captures[2] = channel_3;
    captures[3] = channel_4;
    captures[4] = channel_5;
    captures[5] = channel_6;
    return true;
}


#endif 