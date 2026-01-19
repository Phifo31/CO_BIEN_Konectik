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
#include "Led_WS2812.h"
#include "can_bus.h"

#include "GPIO_Pin.h"
#include "myAdafruit_BNO08x.h"

// Liste des objets globaux de l'application

extern Adafruit_BNO08x bno08x;

extern LED_WS2812 leds_strip_J5;
extern LED_WS2812 leds_strip_J6;
extern LED_WS2812 leds_strip_J7;

extern CAN_BUS can_bus;
extern MFRC522_t myMFRC522;

#define TOUCH_BUTTON_ADDRESS_3    0x68

//                         Red, Green, Blue
LEDS_color_t COLOR_RED = { 0xFF, 0x00, 0x00 };
LEDS_color_t COLOR_GREEN = { 0x00, 0xFF, 0x00 };
LEDS_color_t COLOR_BLUE = { 0x00, 0x00, 0xFF };
LEDS_color_t COLOR_PINK = { 0xFF, 0x00, 0xFF };
LEDS_color_t COLOR_WHITE = { 0xFF, 0xFF, 0xFF };

FDCAN_TxHeaderTypeDef txHeader;
FDCAN_RxHeaderTypeDef rxHeader;

void config_SPI_before_IMU(void);
void config_SPI_before_RFID(void);

extern bool leds_strip_J5_change_flag;
extern bool leds_strip_J6_change_flag;
extern bool leds_strip_J7_change_flag;
extern bool rgbled_touch_button_1_change_flag;
extern bool rgbled_touch_button_2_change_flag;
extern bool rgbled_touch_button_3_change_flag;
extern uint32_t time_for_stop_vibrating_motor;

// Liste des fonctions de l'application utilisées dans les tests
uint16_t can_bus_callback_led(uint16_t sender, uint8_t data[6]);
uint16_t can_bus_callback_leds_strips(uint16_t sender, uint8_t data[6]);
uint16_t can_bus_callback_uart_tx(uint16_t sender, uint8_t data[6]);
uint16_t can_bus_callback_vibrating_motor(uint16_t sender, uint8_t data[6]);
uint16_t can_bus_callback_ledRGB_touch_button(uint16_t sender, uint8_t data[6]);
void start_vibrating_motor(uint8_t dc);
void stop_vibrating_motor(void);
uint8_t read_RFID(void);
void read_IMU(void);
void read_touch_buttons(void);
void change_ledRGB_touch_button_1(void);
void change_ledRGB_touch_button_2(void);
void change_ledRGB_touch_button_3(void);
uint32_t change_state_user_led(void);
uint32_t change_leds_strips_J5(uint32_t last);
uint32_t change_leds_strips_J6(uint32_t last);
uint32_t change_leds_strips_J7(uint32_t last);

/**
 * Test de la led de la carte nucleo et du print redirigé sur la liaison série et l'USB de la carte Nucleo
 *
 * Comportement attendu : la led clignote 2 fois par secondes et la liaison série affiche un compteur
 */
void testDriver_debug_led_state(void) {
    int16_t i = 0;

    while (ENDLESS_LOOP) {
        USER_LOG("Test : %d ", i++);

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

    while (ENDLESS_LOOP) {
        USER_LOG("Test : %d ", i++);

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
 *
 * NB : pour des raisons de fixation mécanique, la led de débug n'est plus cablée
 */
void test_driver_i2cSlave(void) {
    bool status = false;
    int16_t i = 0;

    while (ENDLESS_LOOP) {
        USER_LOG("Test : %d ", i++);

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

    while (ENDLESS_LOOP) {
        USER_LOG("Test : %d ", i++);

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

    while (ENDLESS_LOOP) {
        USER_LOG("Test : %d ", i++);

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

    while (ENDLESS_LOOP) {
        if (TOUCH_BUTTON_get_button_state(TOUCH_BUTTON_ADDRESS_1) == true) {
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
            USER_LOG("Test : %d : push button state : On ", i++);
        } else {
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
            USER_LOG("Test : %d : push button state : Off", i++);
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

    while (ENDLESS_LOOP) {
        if (TOUCH_BUTTON_get_button_state(TOUCH_BUTTON_ADDRESS_1) == true) {
            TOUCH_BUTTON_debug_led_set_state(TOUCH_BUTTON_ADDRESS_1, ON);
            TOUCH_BUTTON_RGB_leds_set_color(TOUCH_BUTTON_ADDRESS_2, COLOR_RED);
            USER_LOG("Test : %d : push button n°1 state : On ", i);
        } else {
            TOUCH_BUTTON_debug_led_set_state(TOUCH_BUTTON_ADDRESS_1, OFF);
            TOUCH_BUTTON_RGB_leds_set_color(TOUCH_BUTTON_ADDRESS_2, COLOR_BLUE);
            USER_LOG("Test : %d : push button n°1 state : Off", i);
        }

        if (TOUCH_BUTTON_get_button_state(TOUCH_BUTTON_ADDRESS_2) == true) {
            TOUCH_BUTTON_debug_led_set_state(TOUCH_BUTTON_ADDRESS_2, ON);
            TOUCH_BUTTON_RGB_leds_set_color(TOUCH_BUTTON_ADDRESS_1, COLOR_RED);
            USER_LOG("Test : %d : push button n°2 state : On ", i++);
        } else {
            TOUCH_BUTTON_debug_led_set_state(TOUCH_BUTTON_ADDRESS_2, OFF);
            TOUCH_BUTTON_RGB_leds_set_color(TOUCH_BUTTON_ADDRESS_1, COLOR_BLUE);
            USER_LOG("Test : %d : push button n°2 state : Off", i++);
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

    while (ENDLESS_LOOP) {
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

    while (ENDLESS_LOOP) {
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

    while (ENDLESS_LOOP) {
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

    while (ENDLESS_LOOP) {

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
 *
 * Test de deux canaux seulement, la première Konnectik n'est pas complètement fonctionnelle sur le cannal J6
 *
 * Correction effectuée - schéma complètement fonctionnel
 *
 *
 */
void test_driver_strips_leds(void) {

    for (int i = 0; i < leds_strip_J5.nb_leds(); i++) {
        leds_strip_J5.set_color(i, 102, 0, 235);
    }

    for (int i = 0; i < leds_strip_J6.nb_leds(); i++) {
        leds_strip_J6.set_color(i, 102, 0, 235);
    }

    for (int i = 0; i < leds_strip_J7.nb_leds(); i++) {
        leds_strip_J7.set_color(i, 102, 0, 235);
    }

    HAL_TIM_Base_Start(&htim1);

    leds_strip_J5.send();
    leds_strip_J6.send();
    leds_strip_J7.send();

    while (ENDLESS_LOOP) {
        for (int i = 0; i < 46; i++) {
            leds_strip_J5.set_brightness(i);
            leds_strip_J6.set_brightness(i);
            leds_strip_J7.set_brightness(i);
            leds_strip_J5.send();
            leds_strip_J6.send();
            leds_strip_J7.send();
            HAL_Delay(50);
        }

        for (int i = 45; i >= 0; i--) {
            leds_strip_J5.set_brightness(i);
            leds_strip_J6.set_brightness(i);
            leds_strip_J7.set_brightness(i);
            leds_strip_J5.send();
            leds_strip_J6.send();
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

    USER_LOG("---- BNO08x starting test ---- "); // form feed + texte

    // Try to initialize!
    if (!bno08x.begin_SPI()) {
        DEBUG_LOG("Test IMU : Failed to find BNO08x chip ! ");
        while (1) {
            HAL_Delay(10);
        }
    }
    USER_LOG("BNO08x Found ! ");

    for (int n = 0; n < bno08x.prodIds.numEntries; n++) {
        DEBUG_LOG("Part %lu : Version : %d.%d.%d ", bno08x.prodIds.entry[n].swPartNumber,
                bno08x.prodIds.entry[n].swVersionMajor, bno08x.prodIds.entry[n].swVersionMinor,
                bno08x.prodIds.entry[n].swVersionPatch);
        DEBUG_LOG("Build %lu ", bno08x.prodIds.entry[n].swBuildNumber);
    }

    // Here is where you define the sensor outputs you want to receive
    DEBUG_LOG("Setting desired reports ");
    if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR)) {
        DEBUG_LOG("Could not enable game vector");
    }

    sh2_Quaternion lastQuat = { 0 };
    bool firstRead = true;
    uint32_t lastMotionTime = HAL_GetTick();
    bool isImmobile = false;

    while (ENDLESS_LOOP) {
        if (bno08x.getSensorEvent(&sensorValue)) {
            if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
                sh2_Quaternion q;
                q.x = sensorValue.un.gameRotationVector.i;
                q.y = sensorValue.un.gameRotationVector.j;
                q.z = sensorValue.un.gameRotationVector.k;
                q.w = sensorValue.un.gameRotationVector.real;

                if (firstRead) {
                    firstRead = false;
                    lastMotionTime = HAL_GetTick();
                } else {
                    float diff = quaternionDiff(q, lastQuat);
                    if (diff > IMU_MOTION_THRESHOLD) {
                        lastMotionTime = HAL_GetTick();
                        if (isImmobile) {
                            USER_LOG("Movement detected !");
                            isImmobile = false;
                        }
                    } else {
                        if (!isImmobile && ((HAL_GetTick() - lastMotionTime) > IMU_IMMOBILE_TIME_MS)) {
                            isImmobile = true;
                            USER_LOG("No move since %u ms ", IMU_IMMOBILE_TIME_MS);
                        }
                    }
                }
                lastQuat.x = q.x;
                lastQuat.y = q.y;
                lastQuat.z = q.z;
                lastQuat.w = q.w;
            }
        }
        HAL_Delay(250);
    }
}

/**
 * Test des modules RFID
 *
 * Réalisé en mai 2025
 * Repris en octobre 2025 avec konectik v1.0 et nouvelle carte RFID (attention au brochage)
 *
 */
void test_RFID_connection_and_detection(void) {
    RFID_MFRC522_Status_t status;

    USER_LOG("---- MRFC 522 test ----");
    // Try to initialize! _MFRC522_Reset(); // inutile, le init fait le reset
    config_SPI_before_RFID();
    //config_SPI_before_IMU();

    //while (1) {
    MFRC522_Init(&myMFRC522);
    HAL_Delay(50);
    //}

    MFRC522_dumpVersionToSerial(&myMFRC522);
    HAL_Delay(50);

    while (ENDLESS_LOOP) {
        uint8_t id[10];
        if (MFRC522_isCardDetected(&myMFRC522)) {
            status = MFRC522_ReadUid(&myMFRC522, id);
            if (status == RFID_MFRC522_OK) {
                USER_LOG("MRFC522 - Card UID : %02X %02X %02X %02X %02X", id[0], id[1], id[2], id[3], id[4]);
            }
        }
        HAL_Delay(50);
    }
}

/**
 * Ce test utilise les deux périphériques reliés au bus SPI
 *      ==> il faut reconfigurer le bus SPI à chaque changement de périphérique
 */
void test_IMU_and_RFID_communication(void) {
    RFID_MFRC522_Status_t status;
    sh2_SensorValue_t sensorValue;

    printf("MRFC 522 test \n\r");
    // Try to initialize! _MFRC522_Reset(); // inutile, le init fait le reset
    config_SPI_before_RFID();
    MFRC522_Init(&myMFRC522);
    HAL_Delay(50);
    MFRC522_dumpVersionToSerial(&myMFRC522);
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

    while (ENDLESS_LOOP) {
        //Reading RFID
        uint8_t id[10];
        //uint8_t type[10];

        config_SPI_before_RFID();
        if (MFRC522_isCardDetected(&myMFRC522)) {
            status = MFRC522_ReadUid(&myMFRC522, id);
            if (status == RFID_MFRC522_OK) {
                printf("MRFC522 : Carte lue num %02X %02X %02X %02X %02X \n\r", id[0], id[1], id[2], id[3], id[4]);
            }
        }

        //status = RFID_MFRC522_check(id, type);
        //if (status == MI_OK) {
        //    printf("MRFC522 : Carte lue num %02X %02X %02X %02X %02X \n\r", id[0], id[1], id[2], id[3], id[4]);
        //}

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
                lastQuat.x = q.x;
                lastQuat.y = q.y;
                lastQuat.z = q.z;
                lastQuat.w = q.w;
            }
        }
        HAL_Delay(250);
    }
}

/**
 *
 */
void test_CAN_BUS_send_only(void) {
    can_bus.begin();

    uint8_t toSend[5];
    toSend[0] = 0x12;
    toSend[1] = 0x34;

    uint8_t count = 0;
    while (ENDLESS_LOOP) {
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
    while (ENDLESS_LOOP) {
        USER_LOG("count = %d ", count);
        toSend[4] = toSend[3] = toSend[2] = count++;
        can_bus.send(toSend, 5);
        HAL_Delay(2500);
    }
}

/**
 *
 */
void test_driver_moteur_vibrant(void) {
    uint8_t data[6];

    HAL_TIM_Base_Init(&htim4);

    for (int i = 0; i < 3; i++) {
        start_vibrating_motor(50);
        HAL_Delay(100);
        stop_vibrating_motor();
        HAL_Delay(1900);
    }

    //uint16_t can_bus_callback_vibrating_motor(uint16_t sender, uint8_t data[6]);

    while (ENDLESS_LOOP) {
        data[0] = 10; // durée
        data[1] = 40; // rapport cyclique

        can_bus_callback_vibrating_motor(0xffff, data);
        while (HAL_GetTick() <= time_for_stop_vibrating_motor)
            ;
        stop_vibrating_motor();
        HAL_Delay(2000);
    }
}

/*
 *
 */
void test_complet_3_12_2025(void) {
    uint8_t data[6];

    uint32_t current_time = 0;
    uint32_t time_for_RFID = 0;
    uint32_t time_for_IMU = 0;
    uint32_t time_for_can_bus_automatic_message = 0;
    uint32_t time_for_read_touch_buttons = 0;
    uint32_t time_for_change_led_state = 0;
    uint32_t time_for_vibrating_motor = 1000;
    uint32_t time_for_change_led_strip = 1000;
    uint8_t cmpt_led_strip = 0;

    USER_LOG("---- Test application start main loop ----");
    while (ENDLESS_LOOP) {
        current_time = HAL_GetTick();

        // Lecture état RFID
        if (current_time >= time_for_RFID) {
            time_for_RFID += PERIODE_LECTURE_RFID;
            uint8_t tmp = read_RFID();
            if (tmp == 0x69) {
                data[0] = 4; // num binaire bandeau
                data[1] = 21; // red
                data[2] = 21; // green
                data[3] = 0; // blue
                data[4] = 21; // brightness
                can_bus_callback_leds_strips(0xffff, data);

                data[0] = 0x3; // num button
                data[1] = 0x24; // mode + shape
                data[2] = 21; // red
                data[3] = 0; // green
                data[4] = 21; // blue
                data[5] = 255; // brightnes
                can_bus_callback_ledRGB_touch_button(0xffff, data);

            } else if (tmp == 0xC7) {
                data[0] = 4; // num binaire bandeau
                data[1] = 0; // red
                data[2] = 41; // green
                data[3] = 42; // blue
                data[4] = 43; // brightness
                can_bus_callback_leds_strips(0xffff, data);

                data[0] = 0x3; // num button
                data[1] = 0x24; // mode + shape
                data[2] = 21; // red
                data[3] = 0; // green
                data[4] = 21; // blue
                data[5] = 255; // brightnes
                can_bus_callback_ledRGB_touch_button(0xffff, data);


            } else if (tmp == 0xd4) {
                data[0] = 4; // num binaire bandeau
                data[1] = 51; // red
                data[2] = 51; // green
                data[3] = 0; // blue
                data[4] = 53; // brightness
                can_bus_callback_leds_strips(0xffff, data);
            } else if (tmp == 0xEE) {
                data[0] = 4; // num binaire bandeau
                data[1] = 68; // red
                data[2] = 69; // green
                data[3] = 100; // blue
                data[4] = 255; // brightness
                can_bus_callback_leds_strips(0xffff, data);
            } else if (tmp == 0xA7) {
                data[0] = 4; // num binaire bandeau
                data[1] = 100; // red
                data[2] = 20; // green
                data[3] = 21; // blue
                data[4] = 55; // brightness
                can_bus_callback_leds_strips(0xffff, data);

                data[0] = 0x03; // num button
                data[1] = 0x30; // mode + shape
                data[2] = 100; // red
                data[3] = 20; // green
                data[4] = 21; // blue
                data[5] = 255; // brightness
                can_bus_callback_ledRGB_touch_button(0xffff, data);
            }
        }

        // Lecture état IMU
        if (current_time >= time_for_IMU) {
            time_for_IMU += PERIODE_LECTURE_IMU;
            read_IMU();
        }

        // Lecture état boutons tactiles
        if (current_time >= time_for_read_touch_buttons) {
            time_for_read_touch_buttons += PERIODE_LECTURE_TOUCH_BUTTONS;
            read_touch_buttons();
        }

        if (current_time >= time_for_change_led_strip) {
            time_for_change_led_strip += 1000;
            if ((++cmpt_led_strip & 0x07) == 7) {
                data[0] = 1; // num binaire bandeau
                data[1] = 100; // red
                data[2] = 10; // green
                data[3] = 21; // blue
                data[4] = 30; // brightness
                can_bus_callback_leds_strips(0xffff, data);

                data[0] = 0x03; // num button
                data[1] = 0x40; // shape + mode
                data[2] = 100; // red
                data[3] = 10; // green
                data[4] = 21; // blue
                data[5] = 50; // brightness
                can_bus_callback_ledRGB_touch_button(0xffff, data);

            } else if ((cmpt_led_strip & 0x07) == 5) {
                data[0] = 3; // num binaire bandeau
                data[1] = 2; // red
                data[2] = 2; // green
                data[3] = 88; // blue
                data[4] = 99; // brightness
                can_bus_callback_leds_strips(0xffff, data);
                data[0] = 3; // num button
                data[1] = 0x30; // shape + mode
                data[2] = 2; // red
                data[3] = 2; // green
                data[4] = 88; // blue
                data[5] = 100; // brightness
                can_bus_callback_ledRGB_touch_button(0xffff, data);

            } else if ((cmpt_led_strip & 0x07) == 3) {
                data[0] = 7; // num binaire bandeau
                data[1] = 20; // red
                data[2] = 100; // green
                data[3] = 20; // blue
                data[4] = 20; // brightness
                can_bus_callback_leds_strips(0xffff, data);
                data[0] = 3; // num button
                data[1] = 0x20; // shape + mode
                data[2] = 20; // red
                data[3] = 100; // green
                data[4] = 20; // blue
                data[5] = 150; // brightness
                can_bus_callback_ledRGB_touch_button(0xffff, data);

            } else if ((cmpt_led_strip & 0x07) == 1) {
                data[0] = 7; // num binaire bandeau
                data[1] = 10; // red
                data[2] = 10; // green
                data[3] = 10; // blue
                data[4] = 99; // brightness
                can_bus_callback_leds_strips(0xffff, data);
                data[0] = 3; // num button
                data[1] = 0x10; // shape + mode
                data[2] = 10; // red
                data[3] = 10; // green
                data[4] = 10; // blue
                data[5] = 200; // brightness
                can_bus_callback_ledRGB_touch_button(0xffff, data);

            }
        }

        if (rgbled_touch_button_1_change_flag) {
            change_ledRGB_touch_button_1();
        }

        if (rgbled_touch_button_2_change_flag) {
            change_ledRGB_touch_button_2();
        }

        if (leds_strip_J5_change_flag) {
            change_leds_strips_J5(0);
        }
        if (leds_strip_J6_change_flag) {
            change_leds_strips_J6(0);
        }
        if (leds_strip_J7_change_flag) {
            change_leds_strips_J7(0);
        }

        // Test moteur vibrant
        if (current_time >= time_for_vibrating_motor) {
            data[0] = 11; // durée
            data[1] = 33; // rapport cyclique
            can_bus_callback_vibrating_motor(0xffff, data);
            time_for_vibrating_motor += 5000;
        }

        if (current_time >= time_for_stop_vibrating_motor) {
            stop_vibrating_motor();
        }

        // Ecriture message CAN de vie (pour vérifier que le can fonctionne)
        if (current_time >= time_for_can_bus_automatic_message) {
            uint8_t toSend[8] = { 0xC0, 0x1D, 0xC0, 0xFF, 0xEE, 0xBA, 0xDB, 0xAD };
            can_bus.send(toSend, 8);
            time_for_can_bus_automatic_message += PERIODE_CAN_BUS_AUTOMATIC_MESSAGE;
        }

        // Modification état led debug (pour vérifier que l'application fonctionne)
        if (current_time >= time_for_change_led_state) {
            time_for_change_led_state += change_state_user_led();
        }
    }
}

void test_mode_strip_leds (void) {

    uint8_t data[6];
    uint32_t current_time = 0;
    uint32_t time_for_change_led_strip_J5 = 1000;
    uint32_t time_for_change_led_strip_J6 = 1100;
    uint32_t time_for_change_led_strip_J7 = 1200;

    USER_LOG("---- Test application start main loop ----");

    data[0] = 1; // num binaire bandeau
    data[1] = 255; // red
    data[2] = 10; // green
    data[3] = 10; // blue
    data[4] = 99; // brightness
    data[5] = BLINK;
    can_bus_callback_leds_strips(0xffff, data);

    data[0] = 2; // num binaire bandeau
    data[1] = 20; // red
    data[2] = 200; // green
    data[3] = 10; // blue
    data[4] = 99; // brightness
    data[5] = FADING_BLINK;
    can_bus_callback_leds_strips(0xffff, data);

    data[0] = 4; // num binaire bandeau
    data[1] = 50; // red
    data[2] = 10; // green
    data[3] = 255; // blue
    data[4] = 99; // brightness
    data[5] = ON;
    can_bus_callback_leds_strips(0xffff, data);

    while (ENDLESS_LOOP) {
        current_time = HAL_GetTick();

        if ((current_time >= time_for_change_led_strip_J5) || leds_strip_J5_change_flag) {
            time_for_change_led_strip_J5 = change_leds_strips_J5(time_for_change_led_strip_J5);
        }

        if ((current_time >= time_for_change_led_strip_J6) || leds_strip_J6_change_flag) {
            time_for_change_led_strip_J6 = change_leds_strips_J6(time_for_change_led_strip_J6);
        }

        if ((current_time >= time_for_change_led_strip_J7) || leds_strip_J7_change_flag) {
            time_for_change_led_strip_J7 = change_leds_strips_J7(time_for_change_led_strip_J7);
        }
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
    //test_driver_strips_leds ();

    //Attention bien penser a changer de mode SPI pour les 2 interfaces IMU et RFID
    //testIMU_connection_and_detection();
    //test_RFID_connection_and_detection();
    //test_IMU_and_RFID_communication();

    //test_CAN_BUS_send_only();
    //test_CAN_BUS_send_receive ();

    //test_integration_one_touch_button_and_RGB_leds();
    //test_integration_two_touch_buttons_and_RGB_leds();
    //test_integration_three_touch_buttons_and_RGB_leds ();

    test_driver_moteur_vibrant();
}

/**
 *:
 */
void test_integration(void) {
    //test_complet_3_12_2025();
    test_mode_strip_leds();
}

// End of file

