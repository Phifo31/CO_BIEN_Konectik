/*
 * Adafruit_BNO085.h
 *
 *  Created on: May 26, 2025
 *      Author: phifo
 */

#ifndef MYADAFRUIT_BNO08X_H_
#define MYADAFRUIT_BNO08X_H_


/*!
 *  @file Adafruit_BNO08x.h
 *
 * 	I2C Driver for the Adafruit BNO08x 9-DOF Orientation IMU Fusion Breakout
 *
 * 	This is a library for the Adafruit BNO08x breakout:
 * 	https://www.adafruit.com/products/4754
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *
 *	BSD license (see license.txt)
 */

//#include "Arduino.h"
#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"
//#include <Adafruit_BusIO_Register.h>
//#include <Adafruit_I2CDevice.h>
//#include <Adafruit_Sensor.h>
//#include <Wire.h>

#include "GPIO_Pin.h"

#define BNO08x_I2CADDR_DEFAULT 0x4A ///< The default I2C address

/* Additional Activities not listed in SH-2 lib */
#define PAC_ON_STAIRS 8 ///< Activity code for being on stairs
#define PAC_OPTION_COUNT 9 ///< The number of current options for the activity classifier

/*!
 *    @brief  Class that stores state and functions for interacting with
 *            the BNO08x 9-DOF Orientation IMU Fusion Breakout
 */
class Adafruit_BNO08x {
public:
  Adafruit_BNO08x(SPI_HandleTypeDef * hspi, GPIO_Pin reset_pin, GPIO_Pin cs_pin, GPIO_Pin irq_pin);
  ~Adafruit_BNO08x();

//  bool begin_I2C(uint8_t i2c_addr = BNO08x_I2CADDR_DEFAULT,
//                 TwoWire *wire = &Wire, int32_t sensor_id = 0);
//  bool begin_UART(HardwareSerial *serial, int32_t sensor_id = 0);
//
  bool begin_SPI(int32_t sensor_id = 0);

  void hardwareReset(void);
  bool wasReset(void);

  bool enableReport(sh2_SensorId_t sensor, uint32_t interval_us = 10000);
  bool getSensorEvent(sh2_SensorValue_t *value);

  sh2_ProductIds_t prodIds; ///< The product IDs returned by the sensor

protected:
  virtual bool _init(int32_t sensor_id);

  sh2_Hal_t
      _HAL; ///< The struct representing the SH2 Hardware Abstraction Layer
};


#endif /* MYADAFRUIT_BNO08X_H_ */
