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

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include <errno.h>
#include <time.h>

#include "common/maths.h"

#include "drivers/io.h"
#include "drivers/dma.h"
#include "drivers/motor.h"
#include "drivers/serial.h"
//#include "drivers/serial_tcp.h"
#include "drivers/system.h"
#include "drivers/pwm_output.h"
#include "drivers/light_led.h"

#include "drivers/timer.h"
#include "drivers/timer_def.h"
const timerHardware_t timerHardware[1]; // unused

#include "drivers/accgyro/accgyro_fake.h"
#include "flight/imu.h"

#include "config/feature.h"
#include "config/config.h"
#include "scheduler/scheduler.h"

#include "pg/rx.h"
#include "pg/motor.h"

#include "rx/rx.h"


//#include "dyad.h"
//#include "target/SITL/udplink.h"

uint32_t SystemCoreClock;

static fdm_packet fdmPkt;
static servo_packet pwmPkt;

static struct timespec start_time;
static double simRate = 1.0;

static bool workerRunning = true;


int timeval_sub(struct timespec *result, struct timespec *x, struct timespec *y);
extern bool startWorkerThread(void);


#define RAD2DEG (180.0 / M_PI)
#define ACC_SCALE (256 / 9.80665)
#define GYRO_SCALE (16.4)

// system
void systemInit(void) {
    int ret;

    clock_gettime(CLOCK_MONOTONIC, &start_time);
    SystemCoreClock = 500 * 1e6; // fake 500MHz

    // serial can't been slow down
    rescheduleTask(TASK_SERIAL, 1);
    startWorkerThread();
}

void systemReset(void)
{

    workerRunning = false;
    exit(0);
}

void systemResetToBootloader(bootloaderRequestType_e requestType) 
{
    UNUSED(requestType);

    workerRunning = false;
    exit(0);
}

void timerInit(void) 
{
}

void timerStart(void) 
{
    
}

void failureMode(failureMode_e mode) {
    printf("[failureMode]!!! %d\n", mode);
    while (1);
}

void indicateFailure(failureMode_e mode, int repeatCount)
{
    UNUSED(repeatCount);
    char *errStr = NULL;
    switch(mode)
    {
        case  FAILURE_DEVELOPER:
            errStr = "FAILURE_DEVELOPER";
        break;
        case FAILURE_MISSING_ACC:
            errStr = "FAILURE_MISSING_ACC";
        break;
        case FAILURE_ACC_INIT:
            errStr = "FAILURE_ACC_INIT";
        break;        
        case FAILURE_ACC_INCOMPATIBLE:
            errStr = "FAILURE_ACC_INCOMPATIBLE";
        break;        
        case FAILURE_INVALID_EEPROM_CONTENTS:
            errStr = "FAILURE_INVALID_EEPROM_CONTENTS";
        break;        
        case FAILURE_CONFIG_STORE_FAILURE:
            errStr = "FAILURE_CONFIG_STORE_FAILURE";
        break;        
        case FAILURE_GYRO_INIT_FAILED:
            errStr = "FAILURE_GYRO_INIT_FAILED";
        break;        
        case FAILURE_FLASH_READ_FAILED:
            errStr = "FAILURE_FLASH_READ_FAILED";
        break;        
        case FAILURE_FLASH_WRITE_FAILED:
            errStr = "FAILURE_FLASH_WRITE_FAILED";
        break;        
        case FAILURE_FLASH_INIT_FAILED:
            errStr = "FAILURE_FLASH_INIT_FAILED";
        break;        
        case FAILURE_EXTERNAL_FLASH_READ_FAILED:
            errStr = "FAILURE_EXTERNAL_FLASH_READ_FAILED";
        break;        
        case FAILURE_EXTERNAL_FLASH_WRITE_FAILED:
            errStr = "FAILURE_EXTERNAL_FLASH_WRITE_FAILED";
        break;        
        case  FAILURE_EXTERNAL_FLASH_INIT_FAILED:
            errStr = "FAILURE_EXTERNAL_FLASH_INIT_FAILED";
        break;        
        case FAILURE_SDCARD_READ_FAILED:
            errStr = "FAILURE_SDCARD_READ_FAILED";
        break;        
        case FAILURE_SDCARD_WRITE_FAILED:
            errStr = "FAILURE_SDCARD_WRITE_FAILED";
        break;        
        case FAILURE_SDCARD_INITIALISATION_FAILED:
            errStr = "FAILURE_SDCARD_INITIALISATION_FAILED";
        break;
        default:
            errStr = "Not known";
        break;        
    }
    printf("Failure LED flash for: %s %d\n", errStr, mode);
}

// Time part
// Thanks ArduPilot
uint64_t nanos64_real() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (ts.tv_sec*1e9 + ts.tv_nsec) - (start_time.tv_sec*1e9 + start_time.tv_nsec);
}

uint64_t micros64_real() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return 1.0e6*((ts.tv_sec + (ts.tv_nsec*1.0e-9)) - (start_time.tv_sec + (start_time.tv_nsec*1.0e-9)));
}

uint64_t millis64_real() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return 1.0e3*((ts.tv_sec + (ts.tv_nsec*1.0e-9)) - (start_time.tv_sec + (start_time.tv_nsec*1.0e-9)));
}

uint64_t micros64() {
    static uint64_t last = 0;
    static uint64_t out = 0;
    uint64_t now = nanos64_real();

    out += (now - last) * simRate;
    last = now;

    return out*1e-3;
//    return micros64_real();
}

uint64_t millis64() {
    static uint64_t last = 0;
    static uint64_t out = 0;
    uint64_t now = nanos64_real();

    out += (now - last) * simRate;
    last = now;

    return out*1e-6;
//    return millis64_real();
}

uint32_t micros(void) {
    return micros64() & 0xFFFFFFFF;
}

uint32_t millis(void) {
    return millis64() & 0xFFFFFFFF;
}

void microsleep(uint32_t usec) {
    struct timespec ts;
    ts.tv_sec = 0;
    ts.tv_nsec = usec*1000UL;
    while (nanosleep(&ts, &ts) == -1 && errno == EINTR) ;
}

void delayMicroseconds(uint32_t us) {
    microsleep(us / simRate);
}

void delayMicroseconds_real(uint32_t us) {
    microsleep(us);
}

void delay(uint32_t ms) {
    uint64_t start = millis64();

    while ((millis64() - start) < ms) {
        microsleep(1000);
    }
}

// Subtract the ‘struct timespec’ values X and Y,  storing the result in RESULT.
// Return 1 if the difference is negative, otherwise 0.
// result = x - y
// from: http://www.gnu.org/software/libc/manual/html_node/Elapsed-Time.html
int timeval_sub(struct timespec *result, struct timespec *x, struct timespec *y) {
    unsigned int s_carry = 0;
    unsigned int ns_carry = 0;
    // Perform the carry for the later subtraction by updating y.
    if (x->tv_nsec < y->tv_nsec) {
        int nsec = (y->tv_nsec - x->tv_nsec) / 1000000000 + 1;
        ns_carry += 1000000000 * nsec;
        s_carry += nsec;
    }

    // Compute the time remaining to wait. tv_usec is certainly positive.
    result->tv_sec = x->tv_sec - y->tv_sec - s_carry;
    result->tv_nsec = x->tv_nsec - y->tv_nsec + ns_carry;

    // Return 1 if result is negative.
    return x->tv_sec < y->tv_sec;
}


// PWM part
pwmOutputPort_t motors[MAX_SUPPORTED_MOTORS];
static pwmOutputPort_t servos[MAX_SUPPORTED_SERVOS];

// real value to send
static int16_t motorsPwm[MAX_SUPPORTED_MOTORS];
static int16_t servosPwm[MAX_SUPPORTED_SERVOS];
static int16_t idlePulse;
static bool pwmIsMotorEnabled(uint8_t index);

void servoDevInit(const servoDevConfig_t *servoConfig) {
    UNUSED(servoConfig);
    for (uint8_t servoIndex = 0; servoIndex < MAX_SUPPORTED_SERVOS; servoIndex++) {
        servos[servoIndex].enabled = true;
    }
}

static motorDevice_t motorPwmDevice; // Forward

pwmOutputPort_t *pwmGetMotors(void) 
{
    return motors;
}

static float pwmConvertFromExternal(uint16_t externalValue)
{
    return (float)externalValue;
}

static uint16_t pwmConvertToExternal(float motorValue)
{
    return (uint16_t)motorValue;
}

static void pwmDisableMotors(void)
{
    motorPwmDevice.enabled = false;
}

static bool pwmEnableMotors(void)
{
  //TCM   return (motorPwmDevice.write != &pwmWriteUnused);

    return true;
}

static void pwmWriteMotor(uint8_t index, float value)
{
    motorsPwm[index] = value - idlePulse;
}

static void pwmWriteMotorInt(uint8_t index, uint16_t value)
{
    pwmWriteMotor(index, (float)value);
}

static void pwmShutdownPulsesForAllMotors(void)
{
    motorPwmDevice.enabled = false;
}

static void pwmCompleteMotorUpdate(void)
{
    // send to simulator
    // for gazebo8 ArduCopterPlugin remap, normal range = [0.0, 1.0], 3D rang = [-1.0, 1.0]

    double outScale = 1000.0;
    if (featureIsEnabled(FEATURE_3D)) {
        outScale = 500.0;
    }

    pwmPkt.motor_speed[3] = motorsPwm[0] / outScale;
    pwmPkt.motor_speed[0] = motorsPwm[1] / outScale;
    pwmPkt.motor_speed[1] = motorsPwm[2] / outScale;
    pwmPkt.motor_speed[2] = motorsPwm[3] / outScale;

    // get one "fdm_packet" can only send one "servo_packet"!!

}

void pwmWriteServo(uint8_t index, float value) {
    servosPwm[index] = value;
}

static motorDevice_t motorPwmDevice = {
    .vTable = {
          .postInit = motorPostInitNull,
        .convertExternalToMotor = pwmConvertFromExternal,
        .convertMotorToExternal = pwmConvertToExternal,
        .enable = pwmEnableMotors,
        .disable = pwmDisableMotors,
        .updateStart = motorUpdateStartNull,
        .write = pwmWriteMotor,
        .writeInt = pwmWriteMotorInt,
        .updateComplete = pwmCompleteMotorUpdate,
        .shutdown = pwmShutdownPulsesForAllMotors,
        .isMotorEnabled = pwmIsMotorEnabled,

    }
};

motorDevice_t *motorPwmDevInit(const motorDevConfig_t *motorConfig, uint16_t _idlePulse, uint8_t motorCount, bool useUnsyncedPwm)
{
    UNUSED(motorConfig);
    UNUSED(useUnsyncedPwm);

    if (motorCount > 4) {
        return NULL;
    }

    idlePulse = _idlePulse;

    for (int motorIndex = 0; motorIndex < MAX_SUPPORTED_MOTORS && motorIndex < motorCount; motorIndex++) {
        motors[motorIndex].enabled = true;
    }
    motorPwmDevice.count = motorCount; // Never used, but seemingly a right thing to set it anyways.
    motorPwmDevice.initialized = true;
    motorPwmDevice.enabled = false;

    return &motorPwmDevice;
}

bool pwmIsMotorEnabled(uint8_t index)
{
    return motors[index].enabled;
}


// ADC part
uint16_t adcGetChannel(uint8_t channel) {
    UNUSED(channel);
    return 0;
}

// stack part
char _estack;
char _Min_Stack_Size;

// fake EEPROM
static FILE *eepromFd = NULL;

void FLASH_Unlock(void) {
    if (eepromFd != NULL) {
        fprintf(stderr, "[FLASH_Unlock] eepromFd != NULL\n");
        return;
    }

    // open or create
    eepromFd = fopen(EEPROM_FILENAME,"r+");
    if (eepromFd != NULL) {
        // obtain file size:
        fseek(eepromFd , 0 , SEEK_END);
        size_t lSize = ftell(eepromFd);
        rewind(eepromFd);

        size_t n = fread(eepromData, 1, sizeof(eepromData), eepromFd);
        if (n == lSize) {
            printf("[FLASH_Unlock] loaded '%s', size = %ld / %ld\n", EEPROM_FILENAME, lSize, sizeof(eepromData));
        } else {
            fprintf(stderr, "[FLASH_Unlock] failed to load '%s'\n", EEPROM_FILENAME);
            return;
        }
    } else {
        printf("[FLASH_Unlock] created '%s', size = %ld\n", EEPROM_FILENAME, sizeof(eepromData));
        if ((eepromFd = fopen(EEPROM_FILENAME, "w+")) == NULL) {
            fprintf(stderr, "[FLASH_Unlock] failed to create '%s'\n", EEPROM_FILENAME);
            return;
        }
        if (fwrite(eepromData, sizeof(eepromData), 1, eepromFd) != 1) {
            fprintf(stderr, "[FLASH_Unlock] write failed: %s\n", strerror(errno));
        }
    }
}

void FLASH_Lock(void) {
    // flush & close
    if (eepromFd != NULL) {
        fseek(eepromFd, 0, SEEK_SET);
        fwrite(eepromData, 1, sizeof(eepromData), eepromFd);
        fclose(eepromFd);
        eepromFd = NULL;
        printf("[FLASH_Lock] saved '%s'\n", EEPROM_FILENAME);
    } else {
        fprintf(stderr, "[FLASH_Lock] eeprom is not unlocked\n");
    }
}

FLASH_Status FLASH_ErasePage(uintptr_t Page_Address) {
    UNUSED(Page_Address);
//    printf("[FLASH_ErasePage]%x\n", Page_Address);
    return FLASH_COMPLETE;
}

FLASH_Status FLASH_ProgramWord(uintptr_t addr, uint32_t value) {
    if ((addr >= (uintptr_t)eepromData) && (addr < (uintptr_t)ARRAYEND(eepromData))) {
        *((uint32_t*)addr) = value;
        printf("[FLASH_ProgramWord]%p = %08x\n", (void*)addr, *((uint32_t*)addr));
    } else {
            printf("[FLASH_ProgramWord]%p out of range!\n", (void*)addr);
    }
    return FLASH_COMPLETE;
}

void uartPinConfigure(const serialPinConfig_t *pSerialPinConfig)
{
    UNUSED(pSerialPinConfig);
    printf("uartPinConfigure");
}

void spektrumBind(rxConfig_t *rxConfig)
{
    UNUSED(rxConfig);
    printf("spektrumBind");
}


void IOConfigGPIO(IO_t io, ioConfig_t cfg)
{
    UNUSED(io);
    UNUSED(cfg);

}

void printfSerialInit(void)
{

}

void setPrintfSerialPort(serialPort_t *serialPort)
{
    UNUSED(serialPort);
}
void unusedPinsInit(void)
{
    printf("unusedPinsInit\n");
}

 #include "drivers/bus.h"
 #if 0
uint8_t mpuGyroReadRegister(const busDevice_t *bus, uint8_t reg)
{
     UNUSED(bus);
      UNUSED(reg);
    return 0;
}
#endif 