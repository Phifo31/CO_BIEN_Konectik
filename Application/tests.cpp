/*
 * tests.cpp
 *
 *  Created on: Apr 18, 2025
 *      Author: phifo
 */

#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "main.h"
#include "../../Common/common_data.h"
#include "../../Common/can_ids.h"

#include "application.h"
#include "touch_button.h"
#include "RFID_MFRC522.h"
#include "led_WS2812.h"
#include "can_bus.h"

#include "GPIO_Pin.h"
#include "myAdafruit_BNO08x.h"

extern Adafruit_BNO08x bno08x;

#define ENDLESS_LOOP 1

//#define MOTION_THRESHOLD  0.01f   // Variation threshold(à ajuster)
//#define IMMOBILE_TIME_MS 2000    // motionless time before validation (ms)

extern LED_WS2812 leds_strip_J5;
//LED_WS2812 leds_strip_J6;
extern LED_WS2812 leds_strip_J7;

extern CAN_BUS can_bus;

//                         Red, Green, Blue
LEDS_color_t COLOR_RED = { 0xFF, 0x00, 0x00 };
LEDS_color_t COLOR_GREEN = { 0x00, 0xFF, 0x00 };
LEDS_color_t COLOR_BLUE = { 0x00, 0x00, 0xFF };
LEDS_color_t COLOR_PINK = { 0xFF, 0x00, 0xFF };
LEDS_color_t COLOR_WHITE = { 0xFF, 0xFF, 0xFF };

FDCAN_TxHeaderTypeDef txHeader;
FDCAN_RxHeaderTypeDef rxHeader;

uint8_t txData[8];
uint8_t rxData[8];

uint32_t txMailBox;

uint16_t msec2sec(uint32_t n, uint16_t *reste) {

    // Récupéré là mais je ne comprend rien
    // https://stackoverflow.com/questions/1294885/convert-milliseconds-to-seconds-in-c
    //  uint32_t q, r, t;
    //n = n + 500; pourquoi ?
    //t = (n >> 7) + (n >> 8) + (n >> 12);
    //q = (n >> 1) + t + (n >> 15) + (t >> 11) + (t >> 14);
    //q = q >> 9;
    //r = n - q*1000;
    //return q + ((r + 24) >> 10);

    // Donc voici ma version
    uint32_t q = n / 1000;
    *reste = n - 1000 * q;
    return (uint16_t) q;
}

/**
 * Test de la led de la carte nucleo et du print redirigé sur la liaison série et l'USB de la carte Nucleo
 *
 * Comportement attendu : la led clignote 2 fois par secondes et la liaison série affiche un compteur
 */
void testDriver_debug_led_state(void) {
    int16_t i = 0;

    while (1) {
        printf("Test : %d \n\r", i++);

        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
        HAL_Delay(50);
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
        HAL_Delay(450);
    }
}

/**
 * Les leds de la carte STM32 et la led de debug du bouton 1 doivent clignoter
 */
void testHardware_i2cSlave(void) {
    bool status = false;

    uint8_t data_on[2] = { DEBUG_LED_STATE, 0x01 };
    uint8_t data_off[2] = { DEBUG_LED_STATE, 0x00 };
    int16_t i = 0;

    while (1) {
        printf("Test : %d \n\r", i++);

        if (status) {
            HAL_I2C_Master_Transmit(&hi2c1, TOUCH_BUTTON_ADDRESS_1, data_on, 2, 100);
            status = false;
        } else {
            HAL_I2C_Master_Transmit(&hi2c1, TOUCH_BUTTON_ADDRESS_1, data_off, 2, 100);
            status = true;
        }

        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
        HAL_Delay(50);
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
        HAL_Delay(450);
    }
}

/**
 * Les leds de la carte STM32 et la led de debug du bouton 1 doivent clignoter
 */
void test_driver_i2cSlave(void) {
    bool status = false;
    int16_t i = 0;

    while (1) {
        printf("Test : %d \n\r", i++);

        if (status) {
            TOUCH_BUTTON_debug_led_set_state(TOUCH_BUTTON_ADDRESS_1, ON);
            status = false;
        } else {
            i = 0;
            TOUCH_BUTTON_debug_led_set_state(TOUCH_BUTTON_ADDRESS_1, OFF);
            status = true;
        }

        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
        HAL_Delay(50);
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
        HAL_Delay(450);
    }
}

/**
 * Utilisation des différents mode de la led de debug : tests des modes
 */
void test_driver_touch_button_led_state(void) {
    int16_t i = 0;

    while (1) {
        printf("Test : %d \n\r", i++);

        TOUCH_BUTTON_debug_led_set_state(TOUCH_BUTTON_ADDRESS_1, ON);
        HAL_Delay(250);
        TOUCH_BUTTON_debug_led_set_state(TOUCH_BUTTON_ADDRESS_1, OFF);
        HAL_Delay(250);

        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
        HAL_Delay(50);
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
        HAL_Delay(450);

        TOUCH_BUTTON_debug_led_set_state(TOUCH_BUTTON_ADDRESS_1, BLINK);
        HAL_Delay(5000);

        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
        HAL_Delay(50);
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
        HAL_Delay(450);

        TOUCH_BUTTON_debug_led_set_state(TOUCH_BUTTON_ADDRESS_1, FADING_BLINK);
        HAL_Delay(10000);
    }
}

/**
 *
 */
void test_driver_touch_button_RGB_LEDS(void) {
    bool status = false;
    int16_t i = 0;

    while (1) {
        printf("Test : %d \n\r", i++);

        if (status) {
            TOUCH_BUTTON_debug_led_set_state(TOUCH_BUTTON_ADDRESS_1, ON);
            TOUCH_BUTTON_RGB_leds_set_mode(TOUCH_BUTTON_ADDRESS_1, ON);
            TOUCH_BUTTON_RGB_leds_set_intensity(TOUCH_BUTTON_ADDRESS_1, 100);
            status = false;
        } else {
            TOUCH_BUTTON_debug_led_set_state(TOUCH_BUTTON_ADDRESS_1, OFF);
            TOUCH_BUTTON_RGB_leds_set_mode(TOUCH_BUTTON_ADDRESS_1, OFF);
            status = true;
        }

        TOUCH_BUTTON_RGB_leds_set_color(TOUCH_BUTTON_ADDRESS_1, COLOR_RED);
        TOUCH_BUTTON_RGB_leds_set_intensity(TOUCH_BUTTON_ADDRESS_1, 50);
        HAL_Delay(250);
        TOUCH_BUTTON_RGB_leds_set_intensity(TOUCH_BUTTON_ADDRESS_1, 100);
        HAL_Delay(250);
        TOUCH_BUTTON_RGB_leds_set_intensity(TOUCH_BUTTON_ADDRESS_1, 150);
        HAL_Delay(250);
        TOUCH_BUTTON_RGB_leds_set_intensity(TOUCH_BUTTON_ADDRESS_1, 200);
        HAL_Delay(250);

        TOUCH_BUTTON_RGB_leds_set_color(TOUCH_BUTTON_ADDRESS_1, COLOR_GREEN);
        HAL_Delay(500);

        TOUCH_BUTTON_RGB_leds_set_color(TOUCH_BUTTON_ADDRESS_1, COLOR_BLUE);
        TOUCH_BUTTON_RGB_leds_set_intensity(TOUCH_BUTTON_ADDRESS_1, 200);
        HAL_Delay(250);
        TOUCH_BUTTON_RGB_leds_set_intensity(TOUCH_BUTTON_ADDRESS_1, 150);
        HAL_Delay(250);
        TOUCH_BUTTON_RGB_leds_set_intensity(TOUCH_BUTTON_ADDRESS_1, 100);
        HAL_Delay(250);
        TOUCH_BUTTON_RGB_leds_set_intensity(TOUCH_BUTTON_ADDRESS_1, 50);
        HAL_Delay(250);
    }
}

/**
 * Test des communications avec le touch button : modifie l'etat de la led en fonction de l'appui
 */
void test_driver_scan_touch_button(void) {
    int16_t i = 0;

    while (1) {
        printf("Test : %d : ", i++);

        if (TOUCH_BUTTON_get_button_state(TOUCH_BUTTON_ADDRESS_1) == true) {
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
            printf("Push button state : On \n\r");
        } else {
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
            printf("Push button state : Off \n\r");
        }
        HAL_Delay(250);
    }
}

/**
 * teste deux boutons, l'appui sur l'un modifie la couleur de l'autre
 */
void test_driver_touch_button_and_RGB_leds(void) {

    int16_t i = 0;

    TOUCH_BUTTON_debug_led_set_state(TOUCH_BUTTON_ADDRESS_1, ON);
    TOUCH_BUTTON_RGB_leds_set_mode(TOUCH_BUTTON_ADDRESS_1, ON);
    TOUCH_BUTTON_RGB_leds_set_intensity(TOUCH_BUTTON_ADDRESS_1, 100);
    TOUCH_BUTTON_RGB_leds_set_color(TOUCH_BUTTON_ADDRESS_1, COLOR_BLUE);

    TOUCH_BUTTON_debug_led_set_state(TOUCH_BUTTON_ADDRESS_2, ON);
    TOUCH_BUTTON_RGB_leds_set_mode(TOUCH_BUTTON_ADDRESS_2, ON);
    TOUCH_BUTTON_RGB_leds_set_intensity(TOUCH_BUTTON_ADDRESS_2, 100);
    TOUCH_BUTTON_RGB_leds_set_color(TOUCH_BUTTON_ADDRESS_2, COLOR_BLUE);

    while (1) {
        printf("Test : %d \n\r", i++);

        if (TOUCH_BUTTON_get_button_state(TOUCH_BUTTON_ADDRESS_1) == true) {
            TOUCH_BUTTON_debug_led_set_state(TOUCH_BUTTON_ADDRESS_1, ON);
            TOUCH_BUTTON_RGB_leds_set_color(TOUCH_BUTTON_ADDRESS_2, COLOR_RED);
            printf("Push button n°1 state : On \n\r");
        } else {
            TOUCH_BUTTON_debug_led_set_state(TOUCH_BUTTON_ADDRESS_1, OFF);
            TOUCH_BUTTON_RGB_leds_set_color(TOUCH_BUTTON_ADDRESS_2, COLOR_BLUE);
            printf("Push button n°1 state : Off \n\r");
        }

        if (TOUCH_BUTTON_get_button_state(TOUCH_BUTTON_ADDRESS_2) == true) {
            TOUCH_BUTTON_debug_led_set_state(TOUCH_BUTTON_ADDRESS_2, ON);
            TOUCH_BUTTON_RGB_leds_set_color(TOUCH_BUTTON_ADDRESS_1, COLOR_RED);
            printf("Push button n°2 state : On \n\r");
        } else {
            TOUCH_BUTTON_debug_led_set_state(TOUCH_BUTTON_ADDRESS_2, OFF);
            TOUCH_BUTTON_RGB_leds_set_color(TOUCH_BUTTON_ADDRESS_1, COLOR_BLUE);
            printf("Push button n°2 state : Off \n\r");
        }
        HAL_Delay(250);
    }
}

/**
 * Affiche l'etat du bouton et modifie les couleurs si on envoi le char c
 */
void test_integration_one_touch_button_and_RGB_leds(void) {
    uint16_t seconds, reste = 0;
    bool state_button1 = false;
    uint8_t color_state = 0;

    TOUCH_BUTTON_debug_led_set_state(TOUCH_BUTTON_ADDRESS_1, ON);
    TOUCH_BUTTON_RGB_leds_set_mode(TOUCH_BUTTON_ADDRESS_1, ON);
    TOUCH_BUTTON_RGB_leds_set_intensity(TOUCH_BUTTON_ADDRESS_1, 100);
    TOUCH_BUTTON_RGB_leds_set_color(TOUCH_BUTTON_ADDRESS_1, COLOR_PINK);

    printf("\n\rTest un bouton tactile et leds RGB - version 1.0 : 22/10/2025\r\n");

    while (1) {
        if ((huart2.Instance->ISR & USART_ISR_RXNE_RXFNE) != 0) {
            uint8_t receive_char = (uint8_t) (huart2.Instance->RDR);    // & (uint8_t) huart2.Mask);
            seconds = msec2sec(HAL_GetTick(), &reste);
            printf("%05u:%03u - Caractère reçu : %c\r\n", seconds, reste, receive_char);
            if (receive_char == 'c') {
                TOUCH_BUTTON_RGB_leds_set_mode(TOUCH_BUTTON_ADDRESS_1, ON);
                TOUCH_BUTTON_RGB_leds_set_intensity(TOUCH_BUTTON_ADDRESS_1, 100);
                switch (color_state) {
                case 0:
                    TOUCH_BUTTON_RGB_leds_set_color(TOUCH_BUTTON_ADDRESS_1, COLOR_PINK);
                    color_state++;
                    break;
                case 1:
                    TOUCH_BUTTON_RGB_leds_set_color(TOUCH_BUTTON_ADDRESS_1, COLOR_RED);
                    color_state++;
                    break;
                case 2:
                    TOUCH_BUTTON_RGB_leds_set_color(TOUCH_BUTTON_ADDRESS_1, COLOR_BLUE);
                    color_state++;
                    break;
                case 3:
                    TOUCH_BUTTON_RGB_leds_set_color(TOUCH_BUTTON_ADDRESS_1, COLOR_GREEN);
                    color_state++;
                    break;
                case 4:
                default:
                    TOUCH_BUTTON_RGB_leds_set_color(TOUCH_BUTTON_ADDRESS_1, COLOR_WHITE);
                    color_state = 0;
                    break;
                }
            }
        }

        if (state_button1 == false) {
            if (TOUCH_BUTTON_get_button_state(TOUCH_BUTTON_ADDRESS_1) == true) {
                state_button1 = true;
                seconds = msec2sec(HAL_GetTick(), &reste);
                printf("%05u:%03u - button 1 = true\r\n", seconds, reste);
            }
        } else if (TOUCH_BUTTON_get_button_state(TOUCH_BUTTON_ADDRESS_1) == false) {
            state_button1 = false;
            seconds = msec2sec(HAL_GetTick(), &reste);
            printf("%05u:%03u - button 1 = false\r\n", seconds, reste);
        }

        HAL_Delay(250);
    }
}

/**
 *
 */
void test_integration_two_touch_buttons_and_RGB_leds(void) {
    uint16_t seconds, reste = 0;
    bool state_button1 = false;
    bool state_button2 = false;
    uint8_t color_state = 0;

    TOUCH_BUTTON_debug_led_set_state(TOUCH_BUTTON_ADDRESS_1, ON);
    TOUCH_BUTTON_RGB_leds_set_mode(TOUCH_BUTTON_ADDRESS_1, ON);
    TOUCH_BUTTON_RGB_leds_set_intensity(TOUCH_BUTTON_ADDRESS_1, 100);
    TOUCH_BUTTON_RGB_leds_set_color(TOUCH_BUTTON_ADDRESS_1, COLOR_PINK);

    TOUCH_BUTTON_debug_led_set_state(TOUCH_BUTTON_ADDRESS_2, ON);
    TOUCH_BUTTON_RGB_leds_set_mode(TOUCH_BUTTON_ADDRESS_2, ON);
    TOUCH_BUTTON_RGB_leds_set_intensity(TOUCH_BUTTON_ADDRESS_2, 100);
    TOUCH_BUTTON_RGB_leds_set_color(TOUCH_BUTTON_ADDRESS_2, COLOR_PINK);

    printf("\n\nTest boutons tactiles et leds - version 1.0 : 23/09/2025\r\n");

    while (1) {
        if ((huart2.Instance->ISR & USART_ISR_RXNE_RXFNE) != 0) {
            uint8_t receive_char = (uint8_t) (huart2.Instance->RDR);    // & (uint8_t) huart2.Mask);
            seconds = msec2sec(HAL_GetTick(), &reste);
            printf("%05u:%03u - Caractère reçu : %c\r\n", seconds, reste, receive_char);
            if (receive_char == 'c') {
                TOUCH_BUTTON_RGB_leds_set_mode(TOUCH_BUTTON_ADDRESS_1, ON);
                TOUCH_BUTTON_RGB_leds_set_mode(TOUCH_BUTTON_ADDRESS_2, ON);
                TOUCH_BUTTON_RGB_leds_set_intensity(TOUCH_BUTTON_ADDRESS_1, 100);
                TOUCH_BUTTON_RGB_leds_set_intensity(TOUCH_BUTTON_ADDRESS_2, 100);
                switch (color_state) {
                case 0:
                    TOUCH_BUTTON_RGB_leds_set_color(TOUCH_BUTTON_ADDRESS_1, COLOR_PINK);
                    TOUCH_BUTTON_RGB_leds_set_color(TOUCH_BUTTON_ADDRESS_2, COLOR_PINK);
                    color_state++;
                    break;
                case 1:
                    TOUCH_BUTTON_RGB_leds_set_color(TOUCH_BUTTON_ADDRESS_1, COLOR_RED);
                    TOUCH_BUTTON_RGB_leds_set_color(TOUCH_BUTTON_ADDRESS_2, COLOR_RED);
                    color_state++;
                    break;
                case 2:
                    TOUCH_BUTTON_RGB_leds_set_color(TOUCH_BUTTON_ADDRESS_1, COLOR_BLUE);
                    TOUCH_BUTTON_RGB_leds_set_color(TOUCH_BUTTON_ADDRESS_2, COLOR_BLUE);
                    color_state++;
                    break;
                case 3:
                    TOUCH_BUTTON_RGB_leds_set_color(TOUCH_BUTTON_ADDRESS_1, COLOR_GREEN);
                    TOUCH_BUTTON_RGB_leds_set_color(TOUCH_BUTTON_ADDRESS_2, COLOR_GREEN);
                    color_state++;
                    break;
                case 4:
                default:
                    TOUCH_BUTTON_RGB_leds_set_color(TOUCH_BUTTON_ADDRESS_1, COLOR_WHITE);
                    TOUCH_BUTTON_RGB_leds_set_color(TOUCH_BUTTON_ADDRESS_2, COLOR_WHITE);
                    color_state = 0;
                    break;
                }
            }
        }

        if (state_button1 == false) {
            if (TOUCH_BUTTON_get_button_state(TOUCH_BUTTON_ADDRESS_1) == true) {
                state_button1 = true;
                seconds = msec2sec(HAL_GetTick(), &reste);
                printf("%05u:%03u - button 1 = pressed\r\n", seconds, reste);
            }
        } else if (TOUCH_BUTTON_get_button_state(TOUCH_BUTTON_ADDRESS_1) == false) {
            state_button1 = false;
            seconds = msec2sec(HAL_GetTick(), &reste);
            printf("%05u:%03u - button 1 = released\r\n", seconds, reste);
        }
        HAL_Delay(1);
        if (state_button2 == false) {
            if (TOUCH_BUTTON_get_button_state(TOUCH_BUTTON_ADDRESS_2) == true) {
                state_button2 = true;
                seconds = msec2sec(HAL_GetTick(), &reste);
                printf("%05u:%03u - button 2 = pressed\r\n", seconds, reste);
            }
        } else if (TOUCH_BUTTON_get_button_state(TOUCH_BUTTON_ADDRESS_2) == false) {
            state_button2 = false;
            seconds = msec2sec(HAL_GetTick(), &reste);
            printf("%05u:%03u - button 2 = released\r\n", seconds, reste);
        }

        HAL_Delay(250);
    }
}

/**
 *
 */
void test_integration_three_touch_buttons_and_RGB_leds(void) {
    uint16_t seconds, reste = 0;
    bool state_button1 = false;
    bool state_button2 = false;
    bool state_button3 = false;
    uint8_t color_state = 0;

    TOUCH_BUTTON_debug_led_set_state(TOUCH_BUTTON_ADDRESS_1, ON);
    TOUCH_BUTTON_RGB_leds_set_mode(TOUCH_BUTTON_ADDRESS_1, ON);
    TOUCH_BUTTON_RGB_leds_set_intensity(TOUCH_BUTTON_ADDRESS_1, 100);
    TOUCH_BUTTON_RGB_leds_set_color(TOUCH_BUTTON_ADDRESS_1, COLOR_PINK);

    TOUCH_BUTTON_debug_led_set_state(TOUCH_BUTTON_ADDRESS_2, ON);
    TOUCH_BUTTON_RGB_leds_set_mode(TOUCH_BUTTON_ADDRESS_2, ON);
    TOUCH_BUTTON_RGB_leds_set_intensity(TOUCH_BUTTON_ADDRESS_2, 100);
    TOUCH_BUTTON_RGB_leds_set_color(TOUCH_BUTTON_ADDRESS_2, COLOR_PINK);

    TOUCH_BUTTON_debug_led_set_state(TOUCH_BUTTON_ADDRESS_3, ON);
    TOUCH_BUTTON_RGB_leds_set_mode(TOUCH_BUTTON_ADDRESS_3, ON);
    TOUCH_BUTTON_RGB_leds_set_intensity(TOUCH_BUTTON_ADDRESS_3, 100);
    TOUCH_BUTTON_RGB_leds_set_color(TOUCH_BUTTON_ADDRESS_3, COLOR_PINK);

    printf("\n\nTest boutons tactiles et leds - version 1.0 : 23/09/2025\r\n");

    while (1) {
        if ((huart2.Instance->ISR & USART_ISR_RXNE_RXFNE) != 0) {
            uint8_t receive_char = (uint8_t) (huart2.Instance->RDR);    // & (uint8_t) huart2.Mask);
            seconds = msec2sec(HAL_GetTick(), &reste);
            printf("%05u:%03u - Caractère reçu : %c\r\n", seconds, reste, receive_char);
            if (receive_char == 'c') {
                TOUCH_BUTTON_RGB_leds_set_mode(TOUCH_BUTTON_ADDRESS_1, ON);
                TOUCH_BUTTON_RGB_leds_set_mode(TOUCH_BUTTON_ADDRESS_2, ON);
                TOUCH_BUTTON_RGB_leds_set_mode(TOUCH_BUTTON_ADDRESS_3, ON);
                TOUCH_BUTTON_RGB_leds_set_intensity(TOUCH_BUTTON_ADDRESS_1, 100);
                TOUCH_BUTTON_RGB_leds_set_intensity(TOUCH_BUTTON_ADDRESS_2, 100);
                TOUCH_BUTTON_RGB_leds_set_intensity(TOUCH_BUTTON_ADDRESS_3, 100);
                switch (color_state) {
                case 0:
                    TOUCH_BUTTON_RGB_leds_set_color(TOUCH_BUTTON_ADDRESS_1, COLOR_PINK);
                    TOUCH_BUTTON_RGB_leds_set_color(TOUCH_BUTTON_ADDRESS_2, COLOR_PINK);
                    TOUCH_BUTTON_RGB_leds_set_color(TOUCH_BUTTON_ADDRESS_3, COLOR_PINK);
                    color_state++;
                    break;
                case 1:
                    TOUCH_BUTTON_RGB_leds_set_color(TOUCH_BUTTON_ADDRESS_1, COLOR_RED);
                    TOUCH_BUTTON_RGB_leds_set_color(TOUCH_BUTTON_ADDRESS_2, COLOR_RED);
                    TOUCH_BUTTON_RGB_leds_set_color(TOUCH_BUTTON_ADDRESS_3, COLOR_RED);
                    color_state++;
                    break;
                case 2:
                    TOUCH_BUTTON_RGB_leds_set_color(TOUCH_BUTTON_ADDRESS_1, COLOR_BLUE);
                    TOUCH_BUTTON_RGB_leds_set_color(TOUCH_BUTTON_ADDRESS_2, COLOR_BLUE);
                    TOUCH_BUTTON_RGB_leds_set_color(TOUCH_BUTTON_ADDRESS_3, COLOR_BLUE);
                    color_state++;
                    break;
                case 3:
                    TOUCH_BUTTON_RGB_leds_set_color(TOUCH_BUTTON_ADDRESS_1, COLOR_GREEN);
                    TOUCH_BUTTON_RGB_leds_set_color(TOUCH_BUTTON_ADDRESS_2, COLOR_GREEN);
                    TOUCH_BUTTON_RGB_leds_set_color(TOUCH_BUTTON_ADDRESS_3, COLOR_GREEN);
                    color_state++;
                    break;
                case 4:
                default:
                    TOUCH_BUTTON_RGB_leds_set_color(TOUCH_BUTTON_ADDRESS_1, COLOR_WHITE);
                    TOUCH_BUTTON_RGB_leds_set_color(TOUCH_BUTTON_ADDRESS_2, COLOR_WHITE);
                    TOUCH_BUTTON_RGB_leds_set_color(TOUCH_BUTTON_ADDRESS_3, COLOR_WHITE);
                    color_state = 0;
                    break;
                }
            }
        }

        if (state_button1 == false) {
            if (TOUCH_BUTTON_get_button_state(TOUCH_BUTTON_ADDRESS_1) == true) {
                state_button1 = true;
                seconds = msec2sec(HAL_GetTick(), &reste);
                printf("%05u:%03u - button 1 = pressed\r\n", seconds, reste);
            }
        } else if (TOUCH_BUTTON_get_button_state(TOUCH_BUTTON_ADDRESS_1) == false) {
            state_button1 = false;
            seconds = msec2sec(HAL_GetTick(), &reste);
            printf("%05u:%03u - button 1 = released\r\n", seconds, reste);
        }
        HAL_Delay(1);
        if (state_button2 == false) {
            if (TOUCH_BUTTON_get_button_state(TOUCH_BUTTON_ADDRESS_2) == true) {
                state_button2 = true;
                seconds = msec2sec(HAL_GetTick(), &reste);
                printf("%05u:%03u - button 2 = pressed\r\n", seconds, reste);
            }
        } else if (TOUCH_BUTTON_get_button_state(TOUCH_BUTTON_ADDRESS_2) == false) {
            state_button2 = false;
            seconds = msec2sec(HAL_GetTick(), &reste);
            printf("%05u:%03u - button 2 = released\r\n", seconds, reste);
        }
        HAL_Delay(1);
        if (state_button3 == false) {
            if (TOUCH_BUTTON_get_button_state(TOUCH_BUTTON_ADDRESS_3) == true) {
                state_button3 = true;
                seconds = msec2sec(HAL_GetTick(), &reste);
                printf("%05u:%03u - button 3 = pressed\r\n", seconds, reste);
            }
        } else if (TOUCH_BUTTON_get_button_state(TOUCH_BUTTON_ADDRESS_3) == false) {
            state_button3 = false;
            seconds = msec2sec(HAL_GetTick(), &reste);
            printf("%05u:%03u - button 3 = released\r\n", seconds, reste);
        }

        HAL_Delay(250);
    }
}

/**
 * Test d'un bandeau de led de notification
 */
void test_unitaires_strip_leds(void) {

    leds_strip_J5.set_color(0, 255, 0, 0);
    leds_strip_J5.set_color(1, 0, 255, 0);
    leds_strip_J5.set_color(2, 0, 0, 255);
    leds_strip_J5.set_color(3, 46, 89, 128);
    leds_strip_J5.set_color(4, 156, 233, 100);
    leds_strip_J5.set_color(5, 102, 0, 235);
    leds_strip_J5.set_color(6, 47, 38, 77);
    leds_strip_J5.set_color(7, 255, 200, 0);
    leds_strip_J5.set_color(8, 0, 255, 0);
    leds_strip_J5.set_color(9, 255, 0, 0);

    HAL_TIM_Base_Start(&htim1);

    while (1) {

        for (int i = 0; i < 46; i++) {
            leds_strip_J5.set_brightness(i);
            leds_strip_J5.send();
            HAL_Delay(50);
        }

        for (int i = 45; i >= 0; i--) {
            leds_strip_J5.set_brightness(i);
            leds_strip_J5.send();
            HAL_Delay(50);
        }
    }
}

/**
 * Test de deux canaux seulement, la première Konnectik n'est pas complètement fonctionnelle sur le cannal J6
 */
void test_driver_strips_leds(void) {

    //while (ENDLESS_LOOP) {
    //HAL_GPIO_TogglePin(debug_GPIO_Port, debug_Pin);
    //HAL_Delay(50);
    //}



    for (int i = 0; i < leds_strip_J5.nb_leds(); i++) {
        leds_strip_J5.set_color(i, 102, 0, 235);
    }

    for (int i = 0; i < leds_strip_J7.nb_leds(); i++) {
        leds_strip_J7.set_color(i, 102, 0, 235);
    }

    HAL_TIM_Base_Start(&htim1);

    //leds_strip_J5.send();
    HAL_Delay(10);
    leds_strip_J7.send();

    while (ENDLESS_LOOP) {

        for (int i = 0; i < 46; i++) {
            leds_strip_J5.set_brightness(i);
            leds_strip_J7.set_brightness(i);
            leds_strip_J5.send();
            HAL_Delay(10);
            leds_strip_J7.send();
            HAL_Delay(50);
        }

        for (int i = 45; i >= 0; i--) {
            leds_strip_J5.set_brightness(i);
            leds_strip_J7.set_brightness(i);
            leds_strip_J5.send();
            HAL_Delay(10);
            leds_strip_J7.send();
            HAL_Delay(50);
        }
    }
}

/**
 * Test de communication avec l'IMU et detection de mouvement
 */
void testIMU_connection_and_detection(void) {
    sh2_SensorValue_t sensorValue;

    printf("%cBNO08x test \n\r", 0x0C); // form feed + texte
    // Try to initialize!
    if (!bno08x.begin_SPI()) {
        // if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte
        // UART buffer! if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
        printf("Test IMU : Failed to find BNO08x chip ! \n\r");
        while (1) {
            HAL_Delay(10);
        }
    }
    printf("BNO08x Found ! \n\r");

    for (int n = 0; n < bno08x.prodIds.numEntries; n++) {
        printf("Part %lu ", bno08x.prodIds.entry[n].swPartNumber);
        printf(": Version : %d.%d.%d \n\r", bno08x.prodIds.entry[n].swVersionMajor,
                bno08x.prodIds.entry[n].swVersionMinor, bno08x.prodIds.entry[n].swVersionPatch);
        printf(" Build %lu \n\r", bno08x.prodIds.entry[n].swBuildNumber);
    }

    // Here is where you define the sensor outputs you want to receive
    printf("Setting desired reports \n\r");
    if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR)) {
        printf("Could not enable game vector\n\r");
    }

    sh2_Quaternion lastQuat = { 0 };
    bool firstRead = true;
    uint32_t lastMotionTime = HAL_GetTick();
    bool isImmobile = false;

    while (1) {
        if (bno08x.getSensorEvent(&sensorValue)) {
            if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
                sh2_Quaternion q;
                q.x = sensorValue.un.gameRotationVector.i;
                q.y = sensorValue.un.gameRotationVector.j;
                q.z = sensorValue.un.gameRotationVector.k;
                q.w = sensorValue.un.gameRotationVector.real;

                if (!firstRead) {
                    float diff = quaternionDiff(q, lastQuat);

                    if (diff > IMU_MOTION_THRESHOLD) {
                        lastMotionTime = HAL_GetTick();
                        if (isImmobile) {
                            printf("Mouvement détecté !\n\r");
                            isImmobile = false;
                        }
                    } else {
                        if (!isImmobile && (HAL_GetTick() - lastMotionTime > IMU_IMMOBILE_TIME_MS)) {
                            isImmobile = true;
                            printf("Immobile depuis %u ms\n\r", IMU_IMMOBILE_TIME_MS);
                        }
                    }
                } else {
                    firstRead = false;
                    lastMotionTime = HAL_GetTick();
                }

                lastQuat = q;
            }
        }
        HAL_Delay(250);
    }
}

/**
 * Test des modules RFID
 *
 * Réalisé en mai 2025
 * Repris en octobre 2025 avec konectik v1.0
 *
 */
void test_RFID_connection(void) {
    RFID_MFRC522_Status_t status;

    printf("\n\r--- MRFC 522 test ---\n\r");
    // Try to initialize! _MFRC522_Reset(); // inutile, le init fait le reset
    config_SPI_before_RFID();

    //while (1) {
    RFID_MFRC522_init();
    HAL_Delay(50);
    //}

    RFID_MFRC522_dumpVersionToSerial();
    HAL_Delay(50);

    while (1) {
        uint8_t id[10];
        uint8_t type[10];

        status = RFID_MFRC522_check(id, type);
        if (status == MI_OK) {
            printf("MRFC522 : Carte lue num %02X %02X %02X %02X %02X \n\r", id[0], id[1], id[2], id[3], id[4]);
        }
        HAL_Delay(250);
    }
}

/**
 *
 */
void test_IMU_and_RFID_communication(void) {
    RFID_MFRC522_Status_t status;
    sh2_SensorValue_t sensorValue;

    printf("MRFC 522 test \n\r");
    // Try to initialize! _MFRC522_Reset(); // inutile, le init fait le reset
    config_SPI_before_RFID();
    RFID_MFRC522_init();
    HAL_Delay(50);
    RFID_MFRC522_dumpVersionToSerial();
    HAL_Delay(50);

    printf("%cBNO08x test \n\r", 0x0C); // form feed + texte
    config_SPI_before_IMU();
    // Try to initialize!
    if (!bno08x.begin_SPI()) {
        printf("Test IMU : Failed to find BNO08x chip ! \n\r");
        while (1) {
            HAL_Delay(10);
        }
    }
    printf("BNO08x Found ! \n\r");

    for (int n = 0; n < bno08x.prodIds.numEntries; n++) {
        printf("Part %lu ", bno08x.prodIds.entry[n].swPartNumber);
        printf(": Version : %d.%d.%d \n\r", bno08x.prodIds.entry[n].swVersionMajor,
                bno08x.prodIds.entry[n].swVersionMinor, bno08x.prodIds.entry[n].swVersionPatch);
        printf(" Build %lu \n\r", bno08x.prodIds.entry[n].swBuildNumber);
    }

    // Here is where you define the sensor outputs you want to receive
    printf("Setting desired reports \n\r");
    if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR)) {
        printf("Could not enable game vector\n\r");
    }

    sh2_Quaternion lastQuat = { 0 };
    bool firstRead = true;
    uint32_t lastMotionTime = HAL_GetTick();
    bool isImmobile = false;

    while (1) {
        //Reading RFID
        uint8_t id[10];
        uint8_t type[10];

        config_SPI_before_RFID();
        status = RFID_MFRC522_check(id, type);
        if (status == MI_OK) {
            printf("MRFC522 : Carte lue num %02X %02X %02X %02X %02X \n\r", id[0], id[1], id[2], id[3], id[4]);
        }

        //Reading IMU
        config_SPI_before_IMU();
        if (bno08x.getSensorEvent(&sensorValue)) {
            if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
                sh2_Quaternion q;
                q.x = sensorValue.un.gameRotationVector.i;
                q.y = sensorValue.un.gameRotationVector.j;
                q.z = sensorValue.un.gameRotationVector.k;
                q.w = sensorValue.un.gameRotationVector.real;

                if (!firstRead) {
                    float diff = quaternionDiff(q, lastQuat);

                    if (diff > IMU_MOTION_THRESHOLD) {
                        lastMotionTime = HAL_GetTick();
                        if (isImmobile) {
                            printf("Mouvement détecté !\n\r");
                            isImmobile = false;
                        }
                    } else {
                        if (!isImmobile && (HAL_GetTick() - lastMotionTime > IMU_IMMOBILE_TIME_MS)) {
                            isImmobile = true;
                            printf("Immobile depuis %u ms\n\r", IMU_IMMOBILE_TIME_MS);
                        }
                    }
                } else {
                    firstRead = false;
                    lastMotionTime = HAL_GetTick();
                }

                lastQuat = q;
            }
        }

        HAL_Delay(250);
    }
}

/**
 *
 */
// Tests CAN BUS
//void HAL_GPIO_EXTI_Callback (uint16_t gpio_pin) {
//    if (gpio_pin == GPIO_PIN_??) {
//        txData [0] = 100; // ms delay
//        txData [1] = 10;  // nb loop
//
//        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, &txData);
//    }
//}
/**
 *
 */
void test_CAN_BUS_send_only(void) {
    can_bus.begin();

    uint8_t toSend[5];
    toSend[0] = 0x12;
    toSend[1] = 0x34;

    uint8_t count = 0;
    while (1) {
        toSend[4] = toSend[3] = toSend[2] = count++;
        can_bus.send(toSend, 5);
        HAL_Delay(500);
    }
}

/**
 *
 */
void test_CAN_BUS_send_receive(void) {
    can_bus.begin();

    uint8_t toSend[5];
    toSend[0] = 0x12;
    toSend[1] = 0x34;

    can_bus.register_callback_function((arbitrationId_t) 0x1234, can_bus_callback_led);
    can_bus.register_callback_function((arbitrationId_t) 0x5678, can_bus_callback_uart_tx);

    uint8_t count = 0;
    while (1) {
        printf("count = %d \n\r", count);
        toSend[4] = toSend[3] = toSend[2] = count++;
        can_bus.send(toSend, 5);
        HAL_Delay(2500);
    }
}

/**
 *:
 */
void tests_unitaires(void) {
    //test_driver_debug_led_state(); // Clignotement de la led de debug
    //testHardware_i2cSlave (); // Clignotement de user led par ecriture lecture de base
    //test_driver_i2cSlave (); // Clignotement de user led par selection de mode
    //test_driver_touch_button_led_state ();
    //test_driver_touch_button_RGB_LEDS();	// Modification de l'intensité et des couleurs des leds RGB
    //test_driver_scan_touch_button ();
    //test_driver_touch_button_and_RGB_leds ();

    // test_unitaires_strip_leds();
    test_driver_strips_leds ();

    //testIMU_connection_and_detection();
    //test_RFID_connection();
    //test_IMU_and_RFID_communication();

    //test_CAN_BUS_send_only();
    //test_CAN_BUS_send_receive ();

    //Attention bien penser a changer de mode SPI pour les 2 interfaces IMU et RFID
    //test_integration_one_touch_button_and_RGB_leds();
    //test_integration_two_touch_buttons_and_RGB_leds();
    //test_integration_three_touch_buttons_and_RGB_leds ();
}

