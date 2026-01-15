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

#include "Led_WS2812.h"

bool data_sent_flag_ ;  // variable de classe (bug du 13/01/2026)


/*
 * table de données à transmettre - utilisé par le DMA du PWM
 */
static uint16_t pwm_data[(24 * MAX_LED) + 60];

/**
 *
 */
LED_WS2812::LED_WS2812(uint32_t channel, uint16_t nb_leds) {
    reset_flag ();
    timer_channel_ = channel;
    nb_leds_ = nb_leds;
    fadelevel_ = 45;
    direction_ = -1;
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
 */
void LED_WS2812::set_color(uint8_t red, uint8_t green, uint8_t blue) {

    for (int i = 0; i < nb_leds_; i++) {
        LED_data_[i][0] = i;
        LED_data_[i][1] = green;
        LED_data_[i][2] = red;
        LED_data_[i][3] = blue;
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
    angle = angle * PI / 180.0;  // in rad
    float facteur_d = tan(angle);

    for (int i = 0; i < MAX_LED; i++) {
        LED_mod_[i][0] = LED_data_[i][0]; // numero de led
        for (int j = 1; j < 4; j++) { // couleurs
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

    while (data_sent_flag () == false);

    // release link before real value
    for (int i = 0; i < 50; i++) {
        pwm_data[indx++] = 0;
    }

    for (int i = 0; i < nb_leds_; i++) {
#if USE_BRIGHTNESS
        color = ((LED_mod_[i][1] << 16) | (LED_mod_[i][2] << 8) | (LED_mod_[i][3]));
#else
        color = ((LED_data_[i][1]<<16) | (LED_data_[i][2]<<8) | (LED_data_[i][3]));
#endif

        for (int i = 23; i >= 0; i--) {
            if (color & (1 << i)) {
                pwm_data[indx++] = WS2812_PWM_DATA_ONE;
            } else {
                pwm_data[indx++] = WS2812_PWM_DATA_ZERO;
            }
        }
    }

    // release link between two leds
    for (int i = 0; i < 50; i++) {
        pwm_data[indx++] = 0;
    }

    switch (timer_channel_) {
    case TIM_CHANNEL_1:
        set_data_sent_flag ();
        HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t*) pwm_data, indx - 1);
        break;

    case TIM_CHANNEL_2:
        set_data_sent_flag ();
        HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_2, (uint32_t*) pwm_data, indx - 1);
        break;

    case TIM_CHANNEL_3:
        set_data_sent_flag ();
        HAL_TIMEx_PWMN_Start_DMA(&htim1, TIM_CHANNEL_3, (uint32_t*) pwm_data, indx - 1);
        break;

    default:
        break;
    }
}

/**
 *
 */
void LED_WS2812::set_off(void) {
    uint32_t indx = 0;

    while (data_sent_flag () == false);

    // release link before real value
    for (int i = 0; i < 50; i++) {
        pwm_data[indx++] = 0;
    }

    for (int i = 0; i < nb_leds_; i++) {
        for (int i = 23; i >= 0; i--) {
            pwm_data[indx++] = WS2812_PWM_DATA_ZERO;
        }
    }

    // release link between two leds
    for (int i = 0; i < 50; i++) {
        pwm_data[indx++] = 0;
    }

    switch (timer_channel_) {
    case TIM_CHANNEL_1:
        set_data_sent_flag ();
        HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t*) pwm_data, indx - 1);
        break;

    case TIM_CHANNEL_2:
        set_data_sent_flag ();
        HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_2, (uint32_t*) pwm_data, indx - 1);
        break;

    case TIM_CHANNEL_3:
        set_data_sent_flag ();
        HAL_TIMEx_PWMN_Start_DMA(&htim1, TIM_CHANNEL_3, (uint32_t*) pwm_data, indx - 1);
        break;

    default:
        break;
    }
}

/**
 *
 */
void LED_WS2812::do_fading(uint8_t step) {
    int16_t tmp = (int16_t) fadelevel_;

    tmp = tmp + (((int16_t) (direction_)) * (int16_t) step); // adjust fading step
    if (tmp >= 45 || tmp <= 0) {
        direction_ *= -1; // reverse fading direction
    }
    if (tmp > 45)
        tmp = 45; // saturation des valeurs entre [0..45]
    if (tmp < 0)
        tmp = 0;

    fadelevel_ = (uint8_t) tmp;
    set_brightness(fadelevel_);
}


void LED_WS2812::reset_flag(void) {data_sent_flag_ = true;};
void LED_WS2812::set_data_sent_flag (void) { data_sent_flag_ = false;};
bool LED_WS2812::data_sent_flag (void) { return data_sent_flag_; };


/**
 *
 */
/*
 void notification_leds_set_mode (LEDS_mode_t mode) {

 }
 */

// end of file
