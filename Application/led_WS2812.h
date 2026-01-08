/*
 * led_WS2812.h
 *
 *  Created on: Oct 14, 2025
 *      Author: phifo
 */

#ifndef LED_WS2812_H_
#define LED_WS2812_H_

#ifdef __cplusplus
extern "C" {
#endif


#define WS2812_PWM_DATA_ONE 60 // 2/3 of 90
#define WS2812_PWM_DATA_ZERO 30  // 1/3 of 90

#define PI 3.14159265

class LED_WS2812 {

public:

    LED_WS2812  (uint32_t channel, uint16_t nb_leds);

    /**
     *
     */
    void set_color(uint8_t led_num, uint8_t red, uint8_t green, uint8_t blue);

    /**
     *
     */
    void set_brightness(uint8_t brightness);

    /**
     *
     */
    void send(void);

    /**
     *
     */
    void reset_flag(void);

    /**
     *
     */
    uint16_t nb_leds (void);

private:
    uint8_t LED_data_[MAX_LED][4];

    bool datasentflag_;

#if USE_BRIGHTNESS
        uint8_t LED_mod_[MAX_LED][4];  // for brightness
#endif

uint32_t timer_channel_;
uint16_t nb_leds_;

void leds_strip_set_color (LEDS_color_t color);
void leds_strip_set_mode (LEDS_mode_t mode);
void leds_strip_set_brightness (uint8_t brightness);
};

#ifdef __cplusplus
} // extern "C"
#endif

#endif /* LED_WS2812_H_ */
