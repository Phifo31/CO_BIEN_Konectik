/*
 * application.cpp
 *
 *  Created on: Apr 11, 2025
 *      Author: phifo
 */

#include <stdio.h>
#include <string.h>

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

// ✅ Définir la période de lecture des boutons tactiles si pas déjà définie
#ifndef PERIODE_LECTURE_TOUCH_BUTTONS
#define PERIODE_LECTURE_TOUCH_BUTTONS 100  // 100 ms pour réactivité instantanée
#endif

void tests_unitaires(void);
void test_integration(void);

void start_vibrating_motor(uint8_t dc);
void stop_vibrating_motor(void);

// Definition des objets et des variables globaux
GPIO_Pin imu_reset(IMU_RST_GPIO_Port, IMU_RST_Pin);
GPIO_Pin imu_cs(IMU_CS_GPIO_Port, IMU_CS_Pin);
GPIO_Pin imu_irq(IMU_IRQ_GPIO_Port, IMU_IRQ_Pin);

Adafruit_BNO08x bno08x(&hspi1, imu_reset, imu_cs, imu_irq);

CAN_BUS can_bus(0x431);

volatile bool auto_led_debug = true;
volatile uint32_t time_for_stop_vibrating_motor = UINT32_MAX;

LED_WS2812 leds_strip_J5(TIM_CHANNEL_1, LEDS_STRIPS_J5_NB_LEDS);
LED_WS2812 leds_strip_J6(TIM_CHANNEL_2, LEDS_STRIPS_J6_NB_LEDS);
LED_WS2812 leds_strip_J7(TIM_CHANNEL_3, LEDS_STRIPS_J7_NB_LEDS);

typedef union RGBLED_TOUCH_BUTTON_DATA {
    uint8_t CANBUSdataArray[6];

    struct {
        LEDS_mode_t rgbLedsMode;
        LEDS_shape_t rgbLedsShape;
        LEDS_color_t rgbLedsColor;
        uint8_t rgbLedBrightness;
        uint8_t rgbLedsBlinkTempo;
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

// ✅ Flags globaux pour tracker si les boutons ont reçu des données CAN valides
static bool button1_initialized = false;
static bool button2_initialized = false;
static bool button3_initialized = false;

LEDS_color_t touched_color = {0, 0, 0};

MFRC522_t myMFRC522 = { &hspi1, RFID_SS_GPIO_Port, RFID_SS_Pin, RFID_RST_GPIO_Port, RFID_RST_Pin };

/**
 * Affichage de tick système en secondes et milli-secondes
 */
uint16_t seconds;
uint16_t reste;
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
 * Fonction appelée par la fin de la transmission strip led par DMA
 *
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
 */
uint8_t read_RFID(void) {
    RFID_MFRC522_Status_t rfid_status;
    HAL_StatusTypeDef canbus_status;
    static bool is_card_detected = false;
    uint8_t ret = 0;

    uint8_t toSend[8] = { 0 };
    uint8_t id[10];

    config_SPI_before_RFID();
    if (is_card_detected) {
        if (MFRC522_isCardDetected(&myMFRC522) == false) {
            is_card_detected = false;
        }
    } else {
        if (MFRC522_isCardDetected(&myMFRC522)) {
            rfid_status = MFRC522_ReadUid(&myMFRC522, id);
            if (rfid_status == RFID_MFRC522_OK) {
                is_card_detected = true;
                USER_LOG("MRFC522 : Carte lue num %02X %02X %02X %02X", id[0], id[1], id[2], id[3]);

                toSend[0] = (uint8_t) ((ARBITRATION_ID_RFID_READ >> 8) & 0x00FF);
                toSend[1] = (uint8_t) (ARBITRATION_ID_RFID_READ & 0x00FF);
                for (uint8_t i = 0; i < 4; i++) {
                    toSend[i + 2] = id[i];
                }
                ret = id[3];

                if ((canbus_status = can_bus.send(toSend, 8)) != HAL_OK) {
                    USER_LOG("CAN BUS : Error ==> endless loop in read_rfid ");
                    Error_Handler();
                }
            }
        }
    }
    return ret;
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

        USER_LOG("BNO085 : State change ==> %d ", state);
        toSend[0] = (uint8_t) ((ARBITRATION_ID_IMU_UPDATE >> 8) & 0x00FF);
        toSend[1] = (uint8_t) (ARBITRATION_ID_IMU_UPDATE & 0x00FF);
        toSend[2] = state;

        if ((canbus_status = can_bus.send(toSend, 8)) != HAL_OK) {
            USER_LOG("CAN BUS : Error ==> endless loop in read_IMU");
            Error_Handler();
        }
    }
}

/**
 * Lecture de l'état des boutons tactiles
 *
 * emission de message ssi il y a un changement d'état
 *
 * ✅ Lors du relâchement, on restaure immédiatement la couleur SI des données CAN ont été reçues
 */
void read_touch_buttons(void) {

    static bool state_button1 = false;
    static bool state_button2 = false;
    static bool state_button3 = false;
    bool state_changed = false;
    static uint8_t state_send = 0;

    // Bouton 1
    if (state_button1 == false) {
        if (TOUCH_BUTTON_get_button_state(TOUCH_BUTTON_ADDRESS_1) == true) {
            TOUCH_BUTTON_RGB_leds_set_color(TOUCH_BUTTON_ADDRESS_1, touched_color);
            TOUCH_BUTTON_RGB_leds_set_intensity(TOUCH_BUTTON_ADDRESS_1, 0);
            state_button1 = true;
            state_send = 01;
            state_changed = true;
            USER_LOG("Touch buttons : button 1 press detected ");

            start_vibrating_motor(50);  // 50% de puissance
            time_for_stop_vibrating_motor = HAL_GetTick() + 50;  // 50ms de vibration
        }
    } else if (TOUCH_BUTTON_get_button_state(TOUCH_BUTTON_ADDRESS_1) == false) {
        state_button1 = false;
        state_send = 00;
        state_changed = true;
        USER_LOG("Touch buttons : button 1 released");

        // ✅ Restauration immédiate SEULEMENT si données CAN valides
        if (button1_initialized) {
            rgbled_touch_button_1_change_flag = true;
        }
    }

    // Bouton 2
    if (state_button2 == false) {
        if (TOUCH_BUTTON_get_button_state(TOUCH_BUTTON_ADDRESS_2) == true) {
            TOUCH_BUTTON_RGB_leds_set_color(TOUCH_BUTTON_ADDRESS_2, touched_color);
            TOUCH_BUTTON_RGB_leds_set_intensity(TOUCH_BUTTON_ADDRESS_2, 0);
            state_button2 = true;
            state_send = 02;
            state_changed = true;
            USER_LOG("Touch buttons : button 2 press detected");

            start_vibrating_motor(50);  // 50% de puissance
            time_for_stop_vibrating_motor = HAL_GetTick() + 50;  // 50ms de vibration
        }
    } else if (TOUCH_BUTTON_get_button_state(TOUCH_BUTTON_ADDRESS_2) == false) {
        state_button2 = false;
        state_send = 00;
        state_changed = true;
        USER_LOG("Touch buttons : button 2 released");

        // ✅ Restauration immédiate SEULEMENT si données CAN valides
        if (button2_initialized) {
            rgbled_touch_button_2_change_flag = true;
        }
    }

    // Bouton 3
    if (state_button3 == false) {
        if (TOUCH_BUTTON_get_button_state(TOUCH_BUTTON_ADDRESS_3) == true) {
            TOUCH_BUTTON_RGB_leds_set_color(TOUCH_BUTTON_ADDRESS_3, touched_color);
            TOUCH_BUTTON_RGB_leds_set_intensity(TOUCH_BUTTON_ADDRESS_3, 0);
            state_button3 = true;
            state_send = 03;
            state_changed = true;
            USER_LOG("Touch buttons : button 3 press detected");

            start_vibrating_motor(50);  // 50% de puissance
            time_for_stop_vibrating_motor = HAL_GetTick() + 50;  // 50ms de vibration
        }
    } else if (TOUCH_BUTTON_get_button_state(TOUCH_BUTTON_ADDRESS_3) == false) {
        state_button3 = false;
        state_send = 00;
        state_changed = true;
        USER_LOG("Touch buttons : button 3 released");

        // ✅ Restauration immédiate SEULEMENT si données CAN valides
        if (button3_initialized) {
            rgbled_touch_button_3_change_flag = true;
        }
    }

    if (state_changed) {
        HAL_StatusTypeDef canbus_status;
        uint8_t toSend[8] = { 0 };

        toSend[0] = (uint8_t) ((ARBITRATION_ID_SENSORS_UPDATE >> 8) & 0x00FF);
        toSend[1] = (uint8_t) (ARBITRATION_ID_SENSORS_UPDATE & 0x00FF);
        toSend[2] = (uint8_t) state_send;

        if ((canbus_status = can_bus.send(toSend, 8)) != HAL_OK) {
            USER_LOG("CAN BUS : Error ==> endless loop in touch button \n\r");
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
 * d'appeler directement les communications I2C ou SPI dans cette interruption
 * A la place, positionnement d'un drapeau traité par la tâche de fond
 */
uint16_t can_bus_callback_ledRGB_touch_button(uint16_t sender, uint8_t data[6]) {

    if ((data[0] & 0x01) != 0) {
        rgbled_button_1_tmp_data.clientDataStruct.rgbLedsMode = (LEDS_mode_t) (data[1] & 0x0F);
        rgbled_button_1_tmp_data.clientDataStruct.rgbLedsShape = (LEDS_shape_t) ((data[1] >> 4) & 0x0F);
        rgbled_button_1_tmp_data.clientDataStruct.rgbLedsColor.red = data[2];
        rgbled_button_1_tmp_data.clientDataStruct.rgbLedsColor.green = data[3];
        rgbled_button_1_tmp_data.clientDataStruct.rgbLedsColor.blue = data[4];
        rgbled_button_1_tmp_data.clientDataStruct.rgbLedBrightness = data[5];

        rgbled_touch_button_1_change_flag = true;

        // ✅ LOG pour debug
        USER_LOG("CAN Button 1: R=%d G=%d B=%d Mode=%d Shape=%d I=%d",
            data[2], data[3], data[4], (data[1] & 0x0F), ((data[1] >> 4) & 0x0F), data[5]);
    }

    if ((data[0] & 0x02) != 0) {
        rgbled_button_2_tmp_data.clientDataStruct.rgbLedsMode = (LEDS_mode_t) (data[1] & 0x0F);
        rgbled_button_2_tmp_data.clientDataStruct.rgbLedsShape = (LEDS_shape_t) ((data[1] >> 4) & 0x0F);
        rgbled_button_2_tmp_data.clientDataStruct.rgbLedsColor.red = data[2];
        rgbled_button_2_tmp_data.clientDataStruct.rgbLedsColor.green = data[3];
        rgbled_button_2_tmp_data.clientDataStruct.rgbLedsColor.blue = data[4];
        rgbled_button_2_tmp_data.clientDataStruct.rgbLedBrightness = data[5];

        rgbled_touch_button_2_change_flag = true;

        // ✅ LOG pour debug
        USER_LOG("CAN Button 2: R=%d G=%d B=%d Mode=%d Shape=%d I=%d",
            data[2], data[3], data[4], (data[1] & 0x0F), ((data[1] >> 4) & 0x0F), data[5]);
    }

    if ((data[0] & 0x04) != 0) {
        rgbled_button_3_tmp_data.clientDataStruct.rgbLedsMode = (LEDS_mode_t) (data[1] & 0x0F);
        rgbled_button_3_tmp_data.clientDataStruct.rgbLedsShape = (LEDS_shape_t) ((data[1] >> 4) & 0x0F);
        rgbled_button_3_tmp_data.clientDataStruct.rgbLedsColor.red = data[2];
        rgbled_button_3_tmp_data.clientDataStruct.rgbLedsColor.green = data[3];
        rgbled_button_3_tmp_data.clientDataStruct.rgbLedsColor.blue = data[4];
        rgbled_button_3_tmp_data.clientDataStruct.rgbLedBrightness = data[5];

        rgbled_touch_button_3_change_flag = true;

        // ✅ LOG pour debug
        USER_LOG("CAN Button 3: R=%d G=%d B=%d Mode=%d Shape=%d I=%d",
            data[2], data[3], data[4], (data[1] & 0x0F), ((data[1] >> 4) & 0x0F), data[5]);
    }


    return 0;
}

/**
 *
 */
void change_ledRGB_touch_button_1(void) {
    I2C_data_t data;

    data.I2C_clientDataArray[1] = RGB_LED_COLOR; // first address
    // LEDS_color_t
    data.I2C_clientDataStruct.rgbLedsColor.red = rgbled_button_1_tmp_data.clientDataStruct.rgbLedsColor.red;
    data.I2C_clientDataStruct.rgbLedsColor.green = rgbled_button_1_tmp_data.clientDataStruct.rgbLedsColor.green;
    data.I2C_clientDataStruct.rgbLedsColor.blue = rgbled_button_1_tmp_data.clientDataStruct.rgbLedsColor.blue;

    data.I2C_clientDataStruct.rgbLedsMode = rgbled_button_1_tmp_data.clientDataStruct.rgbLedsMode;
    data.I2C_clientDataStruct.rgbLedIntensity = rgbled_button_1_tmp_data.clientDataStruct.rgbLedBrightness;
    data.I2C_clientDataStruct.rgbLedsBlinkTempo = 100;
    data.I2C_clientDataStruct.rgbLedShape = rgbled_button_1_tmp_data.clientDataStruct.rgbLedsShape;

    TOUCH_BUTTON_RGB_leds_set_RGB(TOUCH_BUTTON_ADDRESS_1, data.I2C_clientDataArray+1, 8);
    rgbled_touch_button_1_change_flag = false;
}

/**
 *
 */
void change_ledRGB_touch_button_2(void) {
    I2C_data_t data;

    // ✅ LOG pour debug : voir exactement ce qui est envoyé
    USER_LOG("Button 2 UPDATE: R=%d G=%d B=%d Mode=%d Shape=%d Intensity=%d",
        rgbled_button_2_tmp_data.clientDataStruct.rgbLedsColor.red,
        rgbled_button_2_tmp_data.clientDataStruct.rgbLedsColor.green,
        rgbled_button_2_tmp_data.clientDataStruct.rgbLedsColor.blue,
        rgbled_button_2_tmp_data.clientDataStruct.rgbLedsMode,
        rgbled_button_2_tmp_data.clientDataStruct.rgbLedsShape,
        rgbled_button_2_tmp_data.clientDataStruct.rgbLedBrightness
    );

    data.I2C_clientDataArray[1] = RGB_LED_COLOR; // first address
    // LEDS_color_t
    data.I2C_clientDataStruct.rgbLedsColor.red = rgbled_button_2_tmp_data.clientDataStruct.rgbLedsColor.red;
    data.I2C_clientDataStruct.rgbLedsColor.green = rgbled_button_2_tmp_data.clientDataStruct.rgbLedsColor.green;
    data.I2C_clientDataStruct.rgbLedsColor.blue = rgbled_button_2_tmp_data.clientDataStruct.rgbLedsColor.blue;

    data.I2C_clientDataStruct.rgbLedsMode = rgbled_button_2_tmp_data.clientDataStruct.rgbLedsMode;
    data.I2C_clientDataStruct.rgbLedIntensity = rgbled_button_2_tmp_data.clientDataStruct.rgbLedBrightness;
    data.I2C_clientDataStruct.rgbLedsBlinkTempo = 100;
    data.I2C_clientDataStruct.rgbLedShape = rgbled_button_2_tmp_data.clientDataStruct.rgbLedsShape;

    TOUCH_BUTTON_RGB_leds_set_RGB(TOUCH_BUTTON_ADDRESS_2, data.I2C_clientDataArray+1, 8);
    rgbled_touch_button_2_change_flag = false;
}

/**
 *
 */
void change_ledRGB_touch_button_3(void) {
    I2C_data_t data;

    data.I2C_clientDataArray[1] = RGB_LED_COLOR; // first address
    // LEDS_color_t
    data.I2C_clientDataStruct.rgbLedsColor.red = rgbled_button_3_tmp_data.clientDataStruct.rgbLedsColor.red;
    data.I2C_clientDataStruct.rgbLedsColor.green = rgbled_button_3_tmp_data.clientDataStruct.rgbLedsColor.green;
    data.I2C_clientDataStruct.rgbLedsColor.blue = rgbled_button_3_tmp_data.clientDataStruct.rgbLedsColor.blue;

    data.I2C_clientDataStruct.rgbLedsMode = rgbled_button_3_tmp_data.clientDataStruct.rgbLedsMode;
    data.I2C_clientDataStruct.rgbLedIntensity = rgbled_button_3_tmp_data.clientDataStruct.rgbLedBrightness;
    data.I2C_clientDataStruct.rgbLedsBlinkTempo = 100;
    data.I2C_clientDataStruct.rgbLedShape = rgbled_button_3_tmp_data.clientDataStruct.rgbLedsShape;

    TOUCH_BUTTON_RGB_leds_set_RGB(TOUCH_BUTTON_ADDRESS_3, data.I2C_clientDataArray+1, 8);
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

/**
 *
 */
void change_leds_strips_J6(void) {
    for (int i = 0; i < leds_strip_J6.nb_leds(); i++) {
        leds_strip_J6.set_color(i, leds_strip_J6_tmp_data.clientDataStruct.rgbLedsColor.red,
                leds_strip_J6_tmp_data.clientDataStruct.rgbLedsColor.green,
                leds_strip_J6_tmp_data.clientDataStruct.rgbLedsColor.blue);
        leds_strip_J6.set_brightness(leds_strip_J6_tmp_data.clientDataStruct.rgbLedBrightness);
    }
    leds_strip_J6.send();
    leds_strip_J6_change_flag = false;
}

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
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
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

    //tests_unitaires ();

    USER_LOG("---- Application start ! ----");

    // ✅ CRITIQUE : Initialiser les structures à zéro AVANT toute utilisation
    // Cela évite d'avoir des données aléatoires en mémoire (garbage)
    memset((void*)&rgbled_button_1_tmp_data, 0, sizeof(rgbled_data_t));
    memset((void*)&rgbled_button_2_tmp_data, 0, sizeof(rgbled_data_t));
    memset((void*)&rgbled_button_3_tmp_data, 0, sizeof(rgbled_data_t));

    // Initialisation du lecteur de carte
    config_SPI_before_RFID();
    MFRC522_Init(&myMFRC522);
    HAL_Delay(50);
    MFRC522_dumpVersionToSerial(&myMFRC522);
    HAL_Delay(50);

    // Initialisation de la centrale inertielle
    config_SPI_before_IMU();
    bno08x.setup();

    // Initialisation du bus CAN
    can_bus.begin();

    // Initialisation des boutons tactiles avec couleur par défaut
    LEDS_color_t default_color;
    default_color.red = 10;
    default_color.green = 11;
    default_color.blue = 100;

    // ✅ Initialiser les structures rgbled_button_X_tmp_data avec les valeurs par défaut
    // (Nécessaire pour la restauration de couleur lors du relâchement)

    // Bouton 1
    rgbled_button_1_tmp_data.clientDataStruct.rgbLedsMode = FADING_BLINK;
    rgbled_button_1_tmp_data.clientDataStruct.rgbLedsShape = ALL;
    rgbled_button_1_tmp_data.clientDataStruct.rgbLedsColor.red = default_color.red;
    rgbled_button_1_tmp_data.clientDataStruct.rgbLedsColor.green = default_color.green;
    rgbled_button_1_tmp_data.clientDataStruct.rgbLedsColor.blue = default_color.blue;
    rgbled_button_1_tmp_data.clientDataStruct.rgbLedBrightness = 255;

    // Bouton 2
    rgbled_button_2_tmp_data.clientDataStruct.rgbLedsMode = FADING_BLINK;
    rgbled_button_2_tmp_data.clientDataStruct.rgbLedsShape = ALL;
    rgbled_button_2_tmp_data.clientDataStruct.rgbLedsColor.red = default_color.red;
    rgbled_button_2_tmp_data.clientDataStruct.rgbLedsColor.green = default_color.green;
    rgbled_button_2_tmp_data.clientDataStruct.rgbLedsColor.blue = default_color.blue;
    rgbled_button_2_tmp_data.clientDataStruct.rgbLedBrightness = 255;

    // Bouton 3
    rgbled_button_3_tmp_data.clientDataStruct.rgbLedsMode = FADING_BLINK;
    rgbled_button_3_tmp_data.clientDataStruct.rgbLedsShape = ALL;
    rgbled_button_3_tmp_data.clientDataStruct.rgbLedsColor.red = default_color.red;
    rgbled_button_3_tmp_data.clientDataStruct.rgbLedsColor.green = default_color.green;
    rgbled_button_3_tmp_data.clientDataStruct.rgbLedsColor.blue = default_color.blue;
    rgbled_button_3_tmp_data.clientDataStruct.rgbLedBrightness = 255;

    // ✅ Marquer les boutons comme initialisés (données par défaut valides)
    button1_initialized = true;
    button2_initialized = true;
    button3_initialized = true;

    // Envoyer la configuration initiale aux boutons physiques
    // Bouton 1
    TOUCH_BUTTON_RGB_leds_set_color(TOUCH_BUTTON_ADDRESS_1, default_color);
    TOUCH_BUTTON_RGB_leds_set_intensity(TOUCH_BUTTON_ADDRESS_1, 255);
    TOUCH_BUTTON_RGB_leds_set_mode(TOUCH_BUTTON_ADDRESS_1, FADING_BLINK);
    TOUCH_BUTTON_RGB_leds_set_shape(TOUCH_BUTTON_ADDRESS_1, ALL);

    // Bouton 2
    TOUCH_BUTTON_RGB_leds_set_color(TOUCH_BUTTON_ADDRESS_2, default_color);
    TOUCH_BUTTON_RGB_leds_set_intensity(TOUCH_BUTTON_ADDRESS_2, 255);
    TOUCH_BUTTON_RGB_leds_set_mode(TOUCH_BUTTON_ADDRESS_2, FADING_BLINK);
    TOUCH_BUTTON_RGB_leds_set_shape(TOUCH_BUTTON_ADDRESS_2, ALL);

    // Bouton 3
    TOUCH_BUTTON_RGB_leds_set_color(TOUCH_BUTTON_ADDRESS_3, default_color);
    TOUCH_BUTTON_RGB_leds_set_intensity(TOUCH_BUTTON_ADDRESS_3, 255);
    TOUCH_BUTTON_RGB_leds_set_mode(TOUCH_BUTTON_ADDRESS_3, FADING_BLINK);
    TOUCH_BUTTON_RGB_leds_set_shape(TOUCH_BUTTON_ADDRESS_3, ALL);

    can_bus.register_callback_function(ARBITRATION_ID_BUTTONS_CONFIG, can_bus_callback_ledRGB_touch_button);
    can_bus.register_callback_function((arbitrationId_t) 0x8888, can_bus_callback_vibrating_motor);
    can_bus.register_callback_function(ARBITRATION_ID_LEDSTRIP_CONFIG, can_bus_callback_leds_strips);
    can_bus.register_callback_function(ARBITRATION_ID_LED_CONFIG, can_bus_callback_debug_led);
    can_bus.register_callback_function(ARBITRATION_ID_TIME_CONFIG, can_bus_callback_uart_tx);
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
    uint32_t time_for_led_keepalive = 30000;  // ✅ Keepalive toutes les 30s (bien avant timeout ~60-75s)

    //test_integration();

    USER_LOG("---- Application start main loop ----");

    while (ENDLESS_LOOP) {
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

        // Données modifiées par le BUS CAN a transmettre au bouton
        if (rgbled_touch_button_1_change_flag) {
            change_ledRGB_touch_button_1();
            button1_initialized = true;  // ✅ Marquer comme initialisé
        }

        if (rgbled_touch_button_2_change_flag) {
            change_ledRGB_touch_button_2();
            button2_initialized = true;  // ✅ Marquer comme initialisé
        }

        if (rgbled_touch_button_3_change_flag) {
            change_ledRGB_touch_button_3();
            button3_initialized = true;  // ✅ Marquer comme initialisé
        }

        if (leds_strip_J5_change_flag) {
            change_leds_strips_J5();
        }

        if (leds_strip_J6_change_flag) {
            change_leds_strips_J6();
        }

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

        // ✅ Keepalive intelligent : SEULEMENT pour les boutons initialisés via CAN
        if (current_time >= time_for_led_keepalive) {
            time_for_led_keepalive += 30000;  // ✅ 30 secondes (bien avant timeout firmware ~60-75s)

            // Rafraîchir SEULEMENT les boutons qui ont reçu des données CAN
            if (button1_initialized) {
                USER_LOG("Keepalive button 1");
                change_ledRGB_touch_button_1();
            }
            if (button2_initialized) {
                USER_LOG("Keepalive button 2");
                change_ledRGB_touch_button_2();
            }
            if (button3_initialized) {
                USER_LOG("Keepalive button 3");
                change_ledRGB_touch_button_3();
            }
        }
    }
}

// end of file
