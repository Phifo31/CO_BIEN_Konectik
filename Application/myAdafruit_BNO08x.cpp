/*
 * Adafruit_BNO085.cpp
 *
 *  Created on: May 26, 2025
 *      Author: phifo
 *
 *  reecriture (à minima) pour passer la compilation sur STM32
 *
 */

#include <stdio.h>
#include <string.h> // for memset function
#include <math.h>

#include "main.h"
#include "application.h"

#include "GPIO_Pin.h"
#include "myAdafruit_BNO08x.h"

static bool _reset_occurred = false;
static sh2_SensorValue_t *_sensor_value = NULL;
extern Adafruit_BNO08x bno08x;

#define SPI_READ_TIMEOUT	0x1000
#define SPI_WRITE_TIMEOUT	0x1000

/**
 *
 */
static void debug_println(char *str) {
#ifdef PRINT_DEBUG_IMU
    printf("BNO08x : %s \n\r", str);
#endif
}

/**
 * Function to calculate the diff between 2 quaternions
 */

float quaternionDiff(sh2_Quaternion q1, sh2_Quaternion q2) {
    double dr = q1.x - q2.x;
    double di = q1.y - q2.y;
    double dj = q1.z - q2.z;
    double dk = q1.w - q2.w;
    return ((float) sqrt(dr * dr + di * di + dj * dj + dk * dk));
}

//Function to calculate the diff between 2 quaternions, if one is > threshold, we have motion

bool quaternionHasMotion(sh2_Quaternion q1, sh2_Quaternion q2, float threshold) {
    float dx = fabsf(q1.x - q2.x);
    float dy = fabsf(q1.y - q2.y);
    float dz = fabsf(q1.z - q2.z);
    float dw = fabsf(q1.w - q2.w);

    // Debug optionnel :
    // printf("dx=%.5f dy=%.5f dz=%.5f dw=%.5f\n\r", dx, dy, dz, dw);

    return (dx > threshold || dy > threshold || dz > threshold || dw > threshold);
}

/**
 *
 */
static uint32_t hal_getTimeUs(sh2_Hal_t *self) {
    uint32_t t = HAL_GetTick() * 1000;
    // Serial.printf("I2C HAL get time: %d\n", t);
    debug_println((char*) " hal_getTimeUs Non, non & non");
    return t;
}

/**
 *
 */
static void hal_callback(void *cookie, sh2_AsyncEvent_t *pEvent) {
    // If we see a reset, set a flag so that sensors will be reconfigured.
    if (pEvent->eventId == SH2_RESET) {
        debug_println((char*) "Reset callback !\n\r");
        _reset_occurred = true;
    }
}

/**
 * Handle sensor events.
 */
static void sensorHandler(void *cookie, sh2_SensorEvent_t *event) {
    int rc;
    debug_println((char*) "Got an event !\n\r");

    rc = sh2_decodeSensorEvent(_sensor_value, event);
    if (rc != SH2_OK) {
        debug_println((char*) "Error decoding sensor event \r\n");
        _sensor_value->timestamp = 0;
        return;
    }
}

/**
 *
 */
void Adafruit_BNO08x::hal_hardwareReset(void) {
    if (reset_pin_.pin() != -1) {
        debug_println((char*) "Hardware reset");
        //cs_pin_.reset();
        reset_pin_.set();
        HAL_Delay(10);
        reset_pin_.reset();
        HAL_Delay(10);
        reset_pin_.set();
        HAL_Delay(200);
    }
}

/**
 *
 */
void Adafruit_BNO08x::setpin_CS(void) {
    cs_pin_.set();
}

/**
 *
 */
void Adafruit_BNO08x::resetpin_CS(void) {
    cs_pin_.reset();
}

/**
 *
 */
GPIO_PinState Adafruit_BNO08x::readpin_IRQ(void) {
    return irq_pin_.read();
}

/**
 * @brief Construct a new Adafruit_BNO08x::Adafruit_BNO08x object
 *
 * @param reset_pin The arduino pin # connected to the BNO Reset pin
 */
Adafruit_BNO08x::Adafruit_BNO08x(SPI_HandleTypeDef *hspi, const GPIO_Pin &reset_pin, const GPIO_Pin &cs_pin,
        const GPIO_Pin &irq_pin) {

    reset_pin_ = reset_pin;
    cs_pin_ = cs_pin;
    irq_pin_ = irq_pin;
    hspi_ = hspi;
}

/**
 * @brief Destroy the Adafruit_BNO08x::Adafruit_BNO08x object
 *
 */
Adafruit_BNO08x::~Adafruit_BNO08x(void) {
    // if (temp_sensor)
    //   delete temp_sensor;
}

/*!  @brief Initializer for post i2c/spi init
 *   @param sensor_id Optional unique ID for the sensor set
 *   @returns True if chip identified and initialized
 */
bool Adafruit_BNO08x::_init(int32_t sensor_id) {
    int status;

    //
    hal_hardwareReset();

    // Open SH2 interface (also registers non-sensor event handler.)
    status = sh2_open(&_HAL, hal_callback, NULL);
    if (status != SH2_OK) {
        return false;
    }

    // Check connection partially by getting the product id's
    memset(&prodIds, 0, sizeof(prodIds));
    status = sh2_getProdIds(&prodIds);
    if (status != SH2_OK) {
        return false;
    }
    // Register sensor listener
    sh2_setSensorCallback(sensorHandler, NULL);

    return true;
}

/**************************************** interface  *********************************************/
/**
 *
 */
bool spihal_wait_for_int(void) {
    for (uint32_t i = 0; i < 500; i++) {
        //if (!digitalRead(_int_pin))
        //if (irq_pin_.read() == GPIO_PIN_RESET)
        if (bno08x.readpin_IRQ() == GPIO_PIN_RESET)
            //if (HAL_GPIO_ReadPin(irq_pin_.port_, irq_pin_.pin_) == GPIO_PIN_RESET)
            return true;
        debug_println((char*) ".");
        HAL_Delay(1);
    }

    debug_println((char*) "Timed out !");
    bno08x.hal_hardwareReset();

    return false;
}

/**
 *
 */
int spihal_open(sh2_Hal_t *self) {
    debug_println((char*) "SPI HAL open");
    spihal_wait_for_int();
    return 0;
}

/**
 *
 */
void spihal_close(sh2_Hal_t *self) {
    debug_println((char*) "SPI HAL close");
}

/**
 *
 */
int spihal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us) {
    uint16_t packet_size = 0;
    HAL_StatusTypeDef status;

    debug_println((char*) "SPI HAL read");
    if (!spihal_wait_for_int()) {
        return 0;
    }

    //if (!spi_dev->read(pBuffer, 4, 0x00)) {
    bno08x.resetpin_CS();

    if ((status = HAL_SPI_Receive(&hspi1, pBuffer, 4, SPI_READ_TIMEOUT)) != HAL_OK) {
        bno08x.setpin_CS();
        return 0;
    }

    bno08x.setpin_CS();

    // Determine amount to read
    packet_size = (uint16_t) pBuffer[0] | (uint16_t) pBuffer[1] << 8;
    // Unset the "continue" bit
    packet_size &= ~0x8000;

    char lBuffer[100];
    sprintf(lBuffer, "Read SHTP header - packet size : %d, buffer size : %d", packet_size, len);
    debug_println(lBuffer);

    if (packet_size > len) {
        return 0;
    }

    if (!spihal_wait_for_int()) {
        return 0;
    }

    //if (!spi_dev->read(pBuffer, packet_size, 0x00)) {
    bno08x.resetpin_CS();
    if ((status = HAL_SPI_Receive(&hspi1, pBuffer, packet_size, SPI_READ_TIMEOUT)) != HAL_OK) {
        bno08x.setpin_CS();
        return 0;
    }
    bno08x.setpin_CS();

    return packet_size;
}

int spihal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len) {
    debug_println((char*) "SPI HAL write packet size : ");
    //, len);

    if (!spihal_wait_for_int()) {
        return 0;
    }

    //spi_dev->write(pBuffer, len);
    bno08x.resetpin_CS();
    HAL_SPI_Transmit(&hspi1, pBuffer, len, SPI_WRITE_TIMEOUT);
    bno08x.setpin_CS();

    return len;
}

bool Adafruit_BNO08x::begin_SPI(int32_t sensor_id) {

//	  pinMode(_int_pin, INPUT_PULLUP);

//	  if (spi_dev) {
//	    delete spi_dev; // remove old interface
//	  }
//	  spi_dev = new Adafruit_SPIDevice(cs_pin,
//	                                   1000000,               // frequency
//	                                   SPI_BITORDER_MSBFIRST, // bit order
//	                                   SPI_MODE3,             // data mode
//	                                   theSPI);
//	  if (!spi_dev->begin()) {
//	    return false;
//	  }

    //HAL_SPI_Init(hspi)
    cs_pin_.set();
    reset_pin_.reset();

    _HAL.open = spihal_open;
    _HAL.close = spihal_close;
    _HAL.read = spihal_read;
    _HAL.write = spihal_write;
    _HAL.getTimeUs = hal_getTimeUs;

    return _init(sensor_id);
}

/**
 * @brief Fill the given sensor value object with a new report
 *
 * @param value Pointer to an sh2_SensorValue_t struct to fil
 * @return true: The report object was filled with a new report
 * @return false: No new report available to fill
 */
bool Adafruit_BNO08x::getSensorEvent(sh2_SensorValue_t *value) {
    _sensor_value = value;

    value->timestamp = 0;

    sh2_service();

    if (value->timestamp == 0 && value->sensorId != SH2_GYRO_INTEGRATED_RV) {
        // no new events
        return false;
    }

    return true;
}

/**
 * @brief Enable the given report type
 *
 * @param sensorId The report ID to enable
 * @param interval_us The update interval for reports to be generated, in
 * microseconds
 * @return true: success false: failure
 */
bool Adafruit_BNO08x::enableReport(sh2_SensorId_t sensorId, uint32_t interval_us) {
    static sh2_SensorConfig_t config;

    // These sensor options are disabled or not used in most cases
    config.changeSensitivityEnabled = false;
    config.wakeupEnabled = false;
    config.changeSensitivityRelative = false;
    config.alwaysOnEnabled = false;
    config.changeSensitivity = 0;
    config.batchInterval_us = 0;
    config.sensorSpecific = 0;

    config.reportInterval_us = interval_us;
    int status = sh2_setSensorConfig(sensorId, &config);

    if (status != SH2_OK) {
        return false;
    }

    return true;
}

void Adafruit_BNO08x::setup(void) {

    // Try to initialize!
    if (!begin_SPI()) {
        printf("BNO085 : Failed to find BNO08x chip ! \n\r");
        Error_Handler();
    }

    printf("BNO08x : Found ! \n\r");

    for (int n = 0; n < prodIds.numEntries; n++) {
        printf("Part %lu ", prodIds.entry[n].swPartNumber);
        printf(": Version : %d.%d.%d \n\r", prodIds.entry[n].swVersionMajor, prodIds.entry[n].swVersionMinor,
                prodIds.entry[n].swVersionPatch);
        printf(" Build %lu \n\r", prodIds.entry[n].swBuildNumber);
    }

    // Here is where you define the sensor outputs you want to receive
    printf("Setting desired reports \n\r");
    if (!enableReport(SH2_GAME_ROTATION_VECTOR)) {
        printf("Could not enable game vector\n\r");
    }
}

/**
 *
 */
imu_state_t IMU_change_state_detection(sh2_SensorValue_t *values) {
    static bool firstRead = true;
    static uint32_t lastMotionTime;
    static bool isImmobile = false;
    static sh2_Quaternion lastQuat = { 0 };

    imu_state_t state = NO_CHANGE;
    sh2_Quaternion q;

    if (values->sensorId == SH2_GAME_ROTATION_VECTOR) {
        q.x = values->un.gameRotationVector.i;
        q.y = values->un.gameRotationVector.j;
        q.z = values->un.gameRotationVector.k;
        q.w = values->un.gameRotationVector.real;

        if (firstRead) {
            firstRead = false;
            lastMotionTime = HAL_GetTick();
        } else {
            //float diff = quaternionDiff(q, lastQuat);
            bool motionDetected = quaternionHasMotion(q, lastQuat, IMU_MOTION_THRESHOLD);
            lastQuat.x = q.x;
            lastQuat.y = q.y;
            lastQuat.z = q.z;
            lastQuat.w = q.w;

            if (motionDetected) {
                lastMotionTime = HAL_GetTick();
                if (isImmobile) {
                    printf("MOUVEMENT détecté !\n\r");
                    isImmobile = false;
                    state = MOVEMENT_DETECTED;
                }
            } else {
                if (!isImmobile && ((HAL_GetTick() - lastMotionTime) > IMU_IMMOBILE_TIME_MS)) {
                    isImmobile = true;
                    printf("IMMOBILE depuis %u ms\n\r", IMU_IMMOBILE_TIME_MS);
                    state = IMMOBILE_DETECTED;
                }
            }
        }
    }
    return state;
}

// End of file

