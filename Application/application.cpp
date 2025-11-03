/*
 * application.cpp
 *
 *  Created on: Apr 11, 2025
 *      Author: phifo
 */

#include <stdio.h>

#include "main.h"
#include "../../Common/common_data.h"
#include "../../Common/can_ids.h"

#include "GPIO_Pin.h"
#include "can_bus.h"
#include "RFID_MFRC522.h"
#include "myAdafruit_BNO08x.h"
#include "touch_button.h"

#include "application.h"
#include "led_WS2812.h"

#define printf_debug printf

void tests_unitaires(void);

// Definition des objets et des variables globaux
GPIO_Pin imu_reset(IMU_RST_GPIO_Port, IMU_RST_Pin);
GPIO_Pin imu_cs(IMU_CS_GPIO_Port, IMU_CS_Pin);
GPIO_Pin imu_irq(IMU_IRQ_GPIO_Port, IMU_IRQ_Pin);

Adafruit_BNO08x bno08x(&hspi1, imu_reset, imu_cs, imu_irq);

CAN_BUS can_bus(0x431);

volatile bool auto_led_debug = true;
volatile uint32_t time_for_stop_vibrating_motor = UINT32_MAX;

LED_WS2812 leds_strip_J5(TIM_CHANNEL_1, LEDS_STRIPS_J5_NB_LEDS);
LED_WS2812 leds_strip_J6 (TIM_CHANNEL_2, LEDS_STRIPS_J6_NB_LEDS);
LED_WS2812 leds_strip_J7(TIM_CHANNEL_3, LEDS_STRIPS_J7_NB_LEDS);

typedef union RGBLED_TOUCH_BUTTON_DATA {
    uint8_t CANBUSdataArray[6];

    struct {
        LEDS_mode_t rgbLedsMode;
        LEDS_color_t rgbLedsColor;
        uint8_t rgbLedBrightness;
    } clientDataStruct;
} rgbled_data_t;

volatile rgbled_data_t rgbled_button_1_tmp_data;
volatile rgbled_data_t rgbled_button_2_tmp_data;
volatile rgbled_data_t rgbled_button_3_tmp_data;

volatile bool rgbled_touch_button_1_change_flag = false;
volatile bool rgbled_touch_button_2_change_flag = false;
volatile bool rgbled_touch_button_3_change_flag = false;

volatile rgbled_data_t leds_strip_J5_tmp_data;
volatile rgbled_data_t leds_strip_J6_tmp_data;
volatile rgbled_data_t leds_strip_J7_tmp_data;

volatile bool leds_strip_J5_change_flag = false;
volatile bool leds_strip_J6_change_flag = false;
volatile bool leds_strip_J7_change_flag = false;

/**
 * Gestion de la led de vie (tant qu'elle n'est pas utilisée par le BUS CAN
 *
 */
uint32_t change_state_user_led(void) {

    static bool led_state = false;
    uint32_t time = 0;

    if (led_state) {
        led_state = false;
        if (auto_led_debug) {
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
        }
        time = USER_LED_LOW_TIME;
    } else {
        led_state = true;
        if (auto_led_debug) {
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
        }
        time = USER_LED_HIGH_TIME;
    }
    return time;
}

/**
 * Fonction appelée par le fin de la transmission strip led par DMA
 *
 * @todo Ajouter le channel manquant
 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {

    switch (htim->Channel) {
    case HAL_TIM_ACTIVE_CHANNEL_1:
        HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_1);
        leds_strip_J5.reset_flag();
        break;

    case HAL_TIM_ACTIVE_CHANNEL_2:
        HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_2);
        leds_strip_J6.reset_flag();
        break;

    default:
    case HAL_TIM_ACTIVE_CHANNEL_3:
        HAL_TIMEx_PWMN_Stop_DMA(htim, TIM_CHANNEL_3);
        leds_strip_J7.reset_flag();
        break;

    }
}

/**
 *
 */
void strip_leds_set_color(LEDS_color_t color) {
    for (int i = 0; i < MAX_LED; i++) {
        leds_strip_J5.set_color(i, color.red, color.green, color.blue);
    }
}

/**
 *
 */
void strip_leds_set_brightness(uint8_t brightness) {

    leds_strip_J5.set_brightness(brightness);
    leds_strip_J7.set_brightness(brightness);
}

/**
 *
 */
void config_SPI_before_IMU(void) {

    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
    hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 7;
    hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        Error_Handler();
    }
}

/**
 *
 */
void config_SPI_before_RFID(void) {

    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 7;
    hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * Lecture de l'état du RFID et si un badge est lu envoi l'identifiant sur le bus can
 *
 *
 */
void read_RFID(void) {
    RFID_MFRC522_Status_t rfid_status;
    HAL_StatusTypeDef canbus_status;

    uint8_t toSend[8] = { 0 };
    uint8_t id[10];
    uint8_t type[10];

    config_SPI_before_RFID();
    rfid_status = RFID_MFRC522_check(id, type);
    if (rfid_status == MI_OK) {
        printf_debug("MRFC522 : carte lue num %02X %02X %02X %02X %02X \n\r", id[0], id[1], id[2], id[3], id[4]);

        toSend[0] = (uint8_t) ((ARBITRATION_ID_RFID_READ >> 8) & 0x00FF);
        toSend[1] = (uint8_t) (ARBITRATION_ID_RFID_READ & 0x00FF);
        for (uint8_t i = 0; i < 6; i++) {
            toSend[i + 2] = id[i];
        }

        if ((canbus_status = can_bus.send(toSend, 8)) != HAL_OK) {
            printf_debug("CAN BUS : Error ==> endless loop in read_rfid \n\r");
            Error_Handler();
        }
    }
}

/**
 * Lecture de l'imu et detection de changement d'état
 *      soit ça commence à bouger
 *      soit ça n'a plus bougé depuis xx ms
 */
void read_IMU(void) {
    sh2_SensorValue_t sensorValue;
    imu_state_t state;

    config_SPI_before_IMU();
    bno08x.getSensorEvent(&sensorValue);
    if ((state = IMU_change_state_detection(&sensorValue)) != NO_CHANGE) {
        HAL_StatusTypeDef canbus_status;
        uint8_t toSend[8] = { 0 };

        printf_debug("BNO085  : Changement etat : %d \n\r", state);
        toSend[0] = (uint8_t) ((ARBITRATION_ID_IMU_UPDATE >> 8) & 0x00FF);
        toSend[1] = (uint8_t) (ARBITRATION_ID_IMU_UPDATE & 0x00FF);
        toSend[2] = state;

        if ((canbus_status = can_bus.send(toSend, 8)) != HAL_OK) {
            printf_debug("CAN BUS : Error ==> endless loop in read_IMU \n\r");
            Error_Handler();
        }
    }
}

/**
 * Lecture de l'état des boutons tactiles
 *
 * emission de message ssi il y a un changement d'état
 */
void read_touch_buttons(void) {

    static bool state_button1 = false;
    static bool state_button2 = false;
    static bool state_button3 = false;
    bool state_changed = false;

    if (state_button1 == false) {
        if (TOUCH_BUTTON_get_button_state(TOUCH_BUTTON_ADDRESS_1) == true) {
            state_button1 = true;
            state_changed = true;
            printf_debug("Touch buttons : button 1 press detected\n\r");
        }
    } else if (TOUCH_BUTTON_get_button_state(TOUCH_BUTTON_ADDRESS_1) == false) {
        state_button1 = false;
        state_changed = true;
        printf_debug("Touch buttons : button 1 released\n\r");
    }

    if (state_button2 == false) {
        if (TOUCH_BUTTON_get_button_state(TOUCH_BUTTON_ADDRESS_2) == true) {
            state_button2 = true;
            state_changed = true;
            printf_debug("Touch buttons : button 2 press detected\n\r");
        }
    } else if (TOUCH_BUTTON_get_button_state(TOUCH_BUTTON_ADDRESS_2) == false) {
        state_button2 = false;
        state_changed = true;
        printf_debug("Touch buttons : button 2 released\n\r");
    }
    if (state_button3 == false) {
        if (TOUCH_BUTTON_get_button_state(TOUCH_BUTTON_ADDRESS_3) == true) {
            state_button3 = true;
            state_changed = true;
            printf_debug("Touch buttons : button 3 press detected\n\r");
        }
    } else if (TOUCH_BUTTON_get_button_state(TOUCH_BUTTON_ADDRESS_3) == false) {
        state_button3 = false;
        state_changed = true;
        printf_debug("Touch buttons : button 3 released\n\r");
    }

    if (state_changed) {
        HAL_StatusTypeDef canbus_status;
        uint8_t toSend[8] = { 0 };

        toSend[0] = (uint8_t) ((ARBITRATION_ID_PROXIMITY_UPDATE >> 8) & 0x00FF);
        toSend[1] = (uint8_t) (ARBITRATION_ID_PROXIMITY_UPDATE & 0x00FF);
        toSend[2] = (uint8_t) state_button1;
        toSend[3] = (uint8_t) state_button2;
        toSend[4] = (uint8_t) state_button3;

        if ((canbus_status = can_bus.send(toSend, 8)) != HAL_OK) {
            printf_debug("CAN BUS : Error ==> endless loop in touch button \n\r");
            Error_Handler();
        }
    }
}

/**
 *
 */
uint16_t can_bus_callback_debug_led(uint16_t sender, uint8_t data[6]) {

    switch (data[0]) {
    case 0:
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
        auto_led_debug = false;
        break;

    case 1:
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
        auto_led_debug = false;
        break;

    case 2:
    default:
        auto_led_debug = true;
    }
    return 0;
}

/**
 *
 */
uint16_t can_bus_callback_uart_tx(uint16_t sender, uint8_t data[6]) {
    uint8_t len = data[0];

    switch (len) {
    case 1:
        printf("%04X [%d] %02X \r\n", sender, len, data[0]);
        break;
    case 2:
        printf("%04X [%d] %02X %02X \r\n", sender, len, data[0], data[1]);
        break;
    case 3:
        printf("%04X [%d] %02X %02X %02X \r\n", sender, len, data[0], data[1], data[2]);
        break;
    case 4:
        printf("%04X [%d] %02X %02X %02X %02X \r\n", sender, len, data[0], data[1], data[2], data[3]);
        break;
    case 5:
        printf("%04X [%d] %02X %02X %02X %02X %02X \r\n", sender, len, data[0], data[1], data[2], data[3], data[4]);
        break;
    default:
        printf("Erreur \r\n");
        break;
    }
    return 0;
}

/**
 * Modification état led bouton tactile
 *
 * NB : Cette fonction est appellée par IT donc interdiction
 * d'appeler directement les communications I2C ou SPI
 */
uint16_t can_bus_callback_ledRGB_touch_button_1(uint16_t sender, uint8_t data[6]) {

    rgbled_button_1_tmp_data.clientDataStruct.rgbLedsMode = (LEDS_mode_t) data[0];
    rgbled_button_1_tmp_data.clientDataStruct.rgbLedsColor.red = data[1];
    rgbled_button_1_tmp_data.clientDataStruct.rgbLedsColor.green = data[2];
    rgbled_button_1_tmp_data.clientDataStruct.rgbLedsColor.blue = data[3];
    rgbled_button_1_tmp_data.clientDataStruct.rgbLedBrightness = data[4];

    rgbled_touch_button_1_change_flag = true;
    return 0;
}

/**
 *
 */
uint16_t can_bus_callback_ledRGB_touch_button_2(uint16_t sender, uint8_t data[6]) {

    rgbled_button_2_tmp_data.clientDataStruct.rgbLedsMode = (LEDS_mode_t) data[0];
    rgbled_button_2_tmp_data.clientDataStruct.rgbLedsColor.red = data[1];
    rgbled_button_2_tmp_data.clientDataStruct.rgbLedsColor.green = data[2];
    rgbled_button_2_tmp_data.clientDataStruct.rgbLedsColor.blue = data[3];
    rgbled_button_2_tmp_data.clientDataStruct.rgbLedBrightness = data[4];

    rgbled_touch_button_2_change_flag = true;
    return 0;
}

/**
 *
 */
void change_ledRGB_touch_button_1(void) {
    LEDS_color_t color;
    color.blue = rgbled_button_1_tmp_data.clientDataStruct.rgbLedsColor.blue;
    color.red = rgbled_button_1_tmp_data.clientDataStruct.rgbLedsColor.red;
    color.green = rgbled_button_1_tmp_data.clientDataStruct.rgbLedsColor.green;
    TOUCH_BUTTON_RGB_leds_set_mode(TOUCH_BUTTON_ADDRESS_1, rgbled_button_1_tmp_data.clientDataStruct.rgbLedsMode);
    TOUCH_BUTTON_RGB_leds_set_color(TOUCH_BUTTON_ADDRESS_1, color);
    TOUCH_BUTTON_RGB_leds_set_intensity(TOUCH_BUTTON_ADDRESS_1,
            rgbled_button_1_tmp_data.clientDataStruct.rgbLedBrightness);
    rgbled_touch_button_1_change_flag = false;
}

/**
 *
 */
void change_ledRGB_touch_button_2(void) {
    LEDS_color_t color;
    color.blue = rgbled_button_2_tmp_data.clientDataStruct.rgbLedsColor.blue;
    color.red = rgbled_button_2_tmp_data.clientDataStruct.rgbLedsColor.red;
    color.green = rgbled_button_2_tmp_data.clientDataStruct.rgbLedsColor.green;
    TOUCH_BUTTON_RGB_leds_set_mode(TOUCH_BUTTON_ADDRESS_2, rgbled_button_2_tmp_data.clientDataStruct.rgbLedsMode);
    TOUCH_BUTTON_RGB_leds_set_color(TOUCH_BUTTON_ADDRESS_2, color);
    TOUCH_BUTTON_RGB_leds_set_intensity(TOUCH_BUTTON_ADDRESS_2,
            rgbled_button_2_tmp_data.clientDataStruct.rgbLedBrightness);
    rgbled_touch_button_2_change_flag = false;
}

/**
 *
 */
void change_ledRGB_touch_button_3(void) {
    LEDS_color_t color;
    color.blue = rgbled_button_3_tmp_data.clientDataStruct.rgbLedsColor.blue;
    color.red = rgbled_button_3_tmp_data.clientDataStruct.rgbLedsColor.red;
    color.green = rgbled_button_3_tmp_data.clientDataStruct.rgbLedsColor.green;
    TOUCH_BUTTON_RGB_leds_set_mode(TOUCH_BUTTON_ADDRESS_3, rgbled_button_3_tmp_data.clientDataStruct.rgbLedsMode);
    TOUCH_BUTTON_RGB_leds_set_color(TOUCH_BUTTON_ADDRESS_3, color);
    TOUCH_BUTTON_RGB_leds_set_intensity(TOUCH_BUTTON_ADDRESS_3,
            rgbled_button_3_tmp_data.clientDataStruct.rgbLedBrightness);
    rgbled_touch_button_3_change_flag = false;
}

/**
 *  Modification état leds adressables : leds strips
 */
uint16_t can_bus_callback_leds_strips(uint16_t sender, uint8_t data[6]) {

    if ((data[0] & 0x01) != 0) {
        leds_strip_J5_tmp_data.clientDataStruct.rgbLedsColor.red = data[1];
        leds_strip_J5_tmp_data.clientDataStruct.rgbLedsColor.green = data[2];
        leds_strip_J5_tmp_data.clientDataStruct.rgbLedsColor.blue = data[3];
        leds_strip_J5_tmp_data.clientDataStruct.rgbLedBrightness = data[4];
        leds_strip_J5_change_flag = true;
    }

    if ((data[0] & 0x02) != 0) {
        leds_strip_J6_tmp_data.clientDataStruct.rgbLedsColor.red = data[1];
        leds_strip_J6_tmp_data.clientDataStruct.rgbLedsColor.green = data[2];
        leds_strip_J6_tmp_data.clientDataStruct.rgbLedsColor.blue = data[3];
        leds_strip_J6_tmp_data.clientDataStruct.rgbLedBrightness = data[4];
        leds_strip_J6_change_flag = true;
    }

    if ((data[0] & 0x04) != 0) {
        leds_strip_J7_tmp_data.clientDataStruct.rgbLedsColor.red = data[1];
        leds_strip_J7_tmp_data.clientDataStruct.rgbLedsColor.green = data[2];
        leds_strip_J7_tmp_data.clientDataStruct.rgbLedsColor.blue = data[3];
        leds_strip_J7_tmp_data.clientDataStruct.rgbLedBrightness = data[4];
        leds_strip_J7_change_flag = true;
    }
    return 0;
}

/**
 *
 */
void change_leds_strips_J5(void) {
    for (int i = 0; i < leds_strip_J5.nb_leds(); i++) {
        leds_strip_J5.set_color(i, leds_strip_J5_tmp_data.clientDataStruct.rgbLedsColor.red,
                leds_strip_J5_tmp_data.clientDataStruct.rgbLedsColor.green,
                leds_strip_J5_tmp_data.clientDataStruct.rgbLedsColor.blue);
        leds_strip_J5.set_brightness(leds_strip_J5_tmp_data.clientDataStruct.rgbLedBrightness);
    }

    leds_strip_J5.send();
    leds_strip_J5_change_flag = false;
}

///**
// *
// */
//void change_leds_strips_J6(void) {
//    for (int i = 0; i < leds_strip_J6.nb_leds (); i++) {
//        leds_strip_J6.set_color(i, leds_strip_J6_tmp_data.clientDataStruct.rgbLedsColor.red,
//                leds_strip_J6_tmp_data.clientDataStruct.rgbLedsColor.green,
//                leds_strip_J6_tmp_data.clientDataStruct.rgbLedsColor.blue);
//        leds_strip_J6.set_brightness(leds_strip_J6_tmp_data.clientDataStruct.rgbLedBrightness);
//    }
//
//    leds_strip_J6.send();
//    leds_strip_J6_change_flag = false;
//}

/**
 *
 */
void change_leds_strips_J7(void) {
    for (int i = 0; i < leds_strip_J7.nb_leds(); i++) {
        leds_strip_J7.set_color(i, leds_strip_J7_tmp_data.clientDataStruct.rgbLedsColor.red,
                leds_strip_J7_tmp_data.clientDataStruct.rgbLedsColor.green,
                leds_strip_J7_tmp_data.clientDataStruct.rgbLedsColor.blue);
        leds_strip_J7.set_brightness(leds_strip_J7_tmp_data.clientDataStruct.rgbLedBrightness);
    }

    leds_strip_J7.send();
    leds_strip_J7_change_flag = false;
}

/**
 * dc : duty cycle in ]0..99] interval
 */
void start_vibrating_motor(uint8_t dc) {
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, dc);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
}

/**
 *
 */
void stop_vibrating_motor(void) {
    HAL_TIM_PWM_Stop (&htim4, TIM_CHANNEL_1);
    time_for_stop_vibrating_motor = UINT32_MAX;
}

/**
 * Modification état moteur vibrant
 */
uint16_t can_bus_callback_vibrating_motor(uint16_t sender, uint8_t data[6]) {
    time_for_stop_vibrating_motor = HAL_GetTick() + (((uint32_t) data[0]) * (uint32_t) VIBRATING_MOTOR_BASE_TIME);
    start_vibrating_motor(data[1]);
    return 0;
}

/**
 * Initialisation de l'application
 */
void my_setup(void) {

    //tests_unitaires();
    printf_debug("\n\r--- Application start ! ---\n\r");

    config_SPI_before_RFID();
    RFID_MFRC522_init();
    HAL_Delay(50);
    RFID_MFRC522_dumpVersionToSerial();
    HAL_Delay(50);

    config_SPI_before_IMU();
    bno08x.setup();

    can_bus.begin();

    can_bus.register_callback_function(ARBITRATION_ID_BUTTONS_CONFIG, can_bus_callback_ledRGB_touch_button_1);
    can_bus.register_callback_function((arbitrationId_t) 0x8888, can_bus_callback_vibrating_motor);
    can_bus.register_callback_function(ARBITRATION_ID_LEDSTRIP_CONFIG, can_bus_callback_leds_strips);
    can_bus.register_callback_function(ARBITRATION_ID_LED_CONFIG, can_bus_callback_debug_led);
    can_bus.register_callback_function(ARBITRATION_ID_TIME_CONFIG, can_bus_callback_uart_tx);

    printf_debug("\n\r--- Application start main loop ---\n\r");

}

/**
 * Boucle principale de l'application
 */
void my_loop(void) {

    uint32_t current_time = 0;
    uint32_t time_for_RFID = 0;
    uint32_t time_for_IMU = 0;
    uint32_t time_for_can_bus_automatic_message = 0;
    uint32_t time_for_read_touch_buttons = 0;
    uint32_t time_for_change_led_state = 0;

    while (1) {
        current_time = HAL_GetTick();

        // Lecture état RFID
        if (current_time >= time_for_RFID) {
            time_for_RFID += PERIODE_LECTURE_RFID;
            read_RFID();
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

        // Données modifiées par le BUS CAN a transmettre
        if (rgbled_touch_button_1_change_flag == true) {
            change_ledRGB_touch_button_1();
        }

        //  Données modifiées par le BUS CAN a transmettre
        if (rgbled_touch_button_2_change_flag == true) {
            change_ledRGB_touch_button_2();
        }

        //  Données modifiées par le BUS CAN a transmettre
        if (rgbled_touch_button_3_change_flag == true) {
            change_ledRGB_touch_button_3();
        }

        if (leds_strip_J5_change_flag) {
            change_leds_strips_J5();
        }

        //if (leds_strip_J6_change_flag) {
        //    change_leds_strips_J6();
        //}

        if (leds_strip_J7_change_flag) {
            change_leds_strips_J7();
        }

        // Arrêt moteur vibrant
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

// end of file
