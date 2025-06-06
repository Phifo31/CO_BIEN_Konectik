/*
 * touch_button.h
 *
 *  Created on: May 14, 2025
 *      Author: phifo
 */

#ifndef TOUCH_BUTTON_H_
#define TOUCH_BUTTON_H_


void TOUCH_BUTTON_debug_led_set_state (uint8_t address, LEDS_mode_t state);

void TOUCH_BUTTON_RGB_leds_set_color (uint8_t address, LEDS_color_t color);
void TOUCH_BUTTON_RGB_leds_set_mode (uint8_t address, LEDS_mode_t mode);
void TOUCH_BUTTON_RGB_leds_set_intensity (uint8_t address, uint8_t intensity);

bool TOUCH_BUTTON_get_button_state(uint8_t address);




#endif /* TOUCH_BUTTON_H_ */
