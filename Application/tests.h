/*
 * tests.h
 *
 *  Created on: Apr 18, 2025
 *      Author: phifo
 */

#ifndef INC_TESTS_H_
#define INC_TESTS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "../../Common/common_data.h"
#include "touch_button.h"
#include "can_bus.h"
#include "GPIO_Pin.h"
#include "myAdafruit_BNO08x.h"
#include "RFID_MFRC522.h"


// -----------------------------------------------------------------------------
// Variables globales (déclarées dans tests.cpp)
// -----------------------------------------------------------------------------
extern LEDS_color_t COLOR_RED;
extern LEDS_color_t COLOR_GREEN;
extern LEDS_color_t COLOR_BLUE;
extern LEDS_color_t COLOR_PINK;
extern LEDS_color_t COLOR_WHITE;

// -----------------------------------------------------------------------------
// Fonctions utilitaires
// -----------------------------------------------------------------------------
uint16_t msec2sec(uint32_t n, uint16_t *reste);

// -----------------------------------------------------------------------------
// Fonctions de test I2C / LEDs / Buttons
// -----------------------------------------------------------------------------
void testHardware_i2cSlave(void);
void testDriver_i2cSlave(void);
void testDriver_debug_led_state(void);
void testDriver_RGB_LEDS(void);
void testDriver_scan_button(void);
void testDriver_button_and_leds(void);
void TestIntegration_button_and_leds(void);

// -----------------------------------------------------------------------------
// Fonctions de test IMU / RFID
// -----------------------------------------------------------------------------
bool quaternionHasMotion(sh2_Quaternion q1, sh2_Quaternion q2, float threshold);
void testIMU_connection(void);
void test_RFID_connection(void);
void test_IMU_and_RFID_communication(void);

// -----------------------------------------------------------------------------
// Fonctions de test CAN BUS
// -----------------------------------------------------------------------------
uint16_t can_bus_callback_led(uint16_t sender, uint8_t len, uint8_t data[5]);
uint16_t can_bus_callback_uart_tx(uint16_t sender, uint8_t len, uint8_t data[5]);
void test_CAN_BUS_send_only(void);
void test_CAN_BUS_send_receive(void);

// -----------------------------------------------------------------------------
// Lancement global des tests
// -----------------------------------------------------------------------------
void tests_unitaires(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_TESTS_H_ */
