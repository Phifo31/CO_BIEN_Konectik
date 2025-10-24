/**
 * Contrôle des leds de type WS2812
 *
 * Utilise le PWM d'un timer et son DMA associé
 *
 * Les initialisations sont faites par cubeMX, par de modification ici.
 *
 * La base vient de ControllersTech :
 *      https://controllerstech.com/interface-ws2812-with-stm32/
 *
 */

#include <math.h>

#include "main.h"
#include "application.h"

#include "../../Common/common_data.h"

#include "led_WS2812.h"

/*
 * table de données à transmettre - utilisé par le DMA du PWM
 */
static uint16_t pwm_data[(24 * MAX_LED) + 50];

/**
 *
 */
LED_WS2812::LED_WS2812() {

    datasentflag_ = true;

}

/**
 *
 */
void LED_WS2812::set_color(uint8_t led_num, uint8_t red, uint8_t green, uint8_t blue) {
    if (led_num < MAX_LED) {

        LED_data_[led_num][0] = led_num;
        LED_data_[led_num][1] = green;
        LED_data_[led_num][2] = red;
        LED_data_[led_num][3] = blue;
    }
}

/**
 *
 *
 *  0 <= brightness <= 45
 */
void LED_WS2812::set_brightness(uint8_t brightness) {
#if USE_BRIGHTNESS

    if (brightness > 45)
        brightness = 45;

    float angle = 90 - brightness;  // in degrees
    angle = angle * PI / 180;  // in rad
    float facteur_d = tan(angle);

    for (int i = 0; i < MAX_LED; i++) {
        LED_mod_[i][0] = LED_data_[i][0];
        for (int j = 1; j < 4; j++) {
            LED_mod_[i][j] = (uint8_t) (((float) LED_data_[i][j]) / facteur_d);
        }
    }
#endif
}

/**
 *
 */
void LED_WS2812::send(void) {
    uint32_t indx = 0;
    uint32_t color;

    if (datasentflag_ == false)
        return;

    for (int i = 0; i < MAX_LED; i++) {
#if USE_BRIGHTNESS
        color = ((LED_mod_[i][1] << 16) | (LED_mod_[i][2] << 8) | (LED_mod_[i][3]));
#else
        color = ((LED_data_[i][1]<<16) | (LED_data_[i][2]<<8) | (LED_data_[i][3]));
#endif

        for (int i = 23; i >= 0; i--) {
            if (color & (1 << i)) {
                pwm_data[indx] = WS2812_PWM_DATA_ONE;
            }

            else
                pwm_data[indx] = WS2812_PWM_DATA_ZERO;

            indx++;
        }
    }

    // release link
    for (int i = 0; i < 50; i++) {
        pwm_data[indx] = 0;
        indx++;
    }

    HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t*) pwm_data, indx);
//    while (!datasentflag) {};
    datasentflag_ = false;
}

void LED_WS2812::reset_flag(void) {
    datasentflag_ = true;
}



/**
 *
 */
/*
void notification_leds_set_mode (LEDS_mode_t mode) {

}
*/




// end of file

