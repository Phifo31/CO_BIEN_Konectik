/*
 * touch_button.c
 *
 *  Created on: May 14, 2025
 *      Author: phifo
 */

#include "main.h"

#include "application.h"
#include "../../Common/common_data.h"

#define MY_I2C_TIMEOUT	100

/**
 *
 */
void TOUCH_BUTTON_debug_led_set_state(uint8_t address, LEDS_mode_t state) {
    uint8_t data[2] = { DEBUG_LED_STATE, 0x00 };

    data[1] = state;
    HAL_I2C_Master_Transmit(&hi2c1, address, data, 2, MY_I2C_TIMEOUT);
}

/**
 *
 */
void TOUCH_BUTTON_RGB_leds_set_mode(uint8_t address, LEDS_mode_t mode) {
    uint8_t data[2] = { RGB_LED_MODE, 0x00 };

    data[1] = mode;
    HAL_I2C_Master_Transmit(&hi2c1, address, data, 2, MY_I2C_TIMEOUT);
}

/**
 *
 */
void TOUCH_BUTTON_RGB_leds_set_color(uint8_t address, LEDS_color_t color) {
    uint8_t data[4] = { RGB_LED_COLOR, 0x00, 0x00, 0x00 };

    data[1] = color.red;
    data[2] = color.green;
    data[3] = color.blue;
    HAL_I2C_Master_Transmit(&hi2c1, address, data, 4, MY_I2C_TIMEOUT);
}

/**
 *
 */
void TOUCH_BUTTON_RGB_leds_set_intensity(uint8_t address, uint8_t intensity) {
    uint8_t data[2] = { RGB_LED_INTENSITY, 0x00 };

    data[1] = intensity;
    HAL_I2C_Master_Transmit(&hi2c1, address, data, 2, 100);
}

/**
 *
 */
void TOUCH_BUTTON_RGB_leds_set_blink_tempo(uint8_t address, int tempo) {

}

/**
 *
 */
bool TOUCH_BUTTON_verif_communication(uint8_t address) {

    return false;
}

/**
 *
 */
bool TOUCH_BUTTON_get_button_state(uint8_t address) {
    HAL_StatusTypeDef status;
    uint8_t data[2] = { TOUCH_BUTTON_STATE, 0x00 };

    if ((status = HAL_I2C_Master_Transmit(&hi2c1, address, data, 1, MY_I2C_TIMEOUT)) != HAL_ERROR) {
        if ((status = HAL_I2C_Master_Receive(&hi2c1, address, data, 1, MY_I2C_TIMEOUT)) != HAL_ERROR) {
            if (data[0] == TOUCH_BUTTON_PRESSED)
                return true;
            if (data[0] == TOUCH_BUTTON_RELEASED)
                return false;
        }
    }
    return false;
}

/**
 *
 */
bool TOUCH_BUTTON_get_button_raw_values(uint8_t address) {

    return false;
}

// End of file
