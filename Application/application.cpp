/*
 * application.cpp
 *
 *  Created on: Apr 11, 2025
 *      Author: phifo
 */

#include <stdio.h>

#include "main.h"
#include "../../Common/common_data.h"

#include "can_bus.h"
#include "RFID_MFRC522.h"

#include "application.h"

#define printf_debug printf

#define PERIODE_LECTURE_RFID 100
#define USER_LED_LOW_TIME 950
#define USER_LED_HIGH_TIME 50

#define CAN_BUS_RFID_READ_ID 0x1234

void tests_unitaires(void);

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
    hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        Error_Handler();
    }
}

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
    hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        Error_Handler();
    }
}

CAN_BUS can_bus(0x431);

void my_setup(void) {

    can_bus.begin();

}

/**
 * Lecture de l'état du RFID et si un badge est lu envoi l'identifiant sur le bus can
 *
 *
 */
void read_RFID(uint32_t current_time) {
    static uint32_t time_for_RFID = 0;
    RFID_MFRC522_Status_t status;
    uint8_t toSend[8];
    uint8_t id[10];
    uint8_t type[10];

    if (current_time >= time_for_RFID) {

        config_SPI_before_RFID();
        status = RFID_MFRC522_check(id, type);
        if (status == MI_OK) {
            printf_debug("MRFC522 : Carte lue num %02X %02X %02X %02X %02X \n\r", id[0], id[1], id[2], id[3], id[4]);

            toSend[0] = (uint8_t) ((CAN_BUS_RFID_READ_ID >> 8) & 0x00FF);
            toSend[1] = (uint8_t) (CAN_BUS_RFID_READ_ID & 0x00FF);

            toSend[2] = id[0];
            toSend[3] = id[1];
            toSend[4] = id[2];
            toSend[5] = id[3];
            toSend[6] = id[4];
            toSend[7] = id[5];

            can_bus.send(toSend, 8);
        }
        time_for_RFID += PERIODE_LECTURE_RFID;
    }
}

/**
 * Gestion de la led de vie (tant qu'elle n'est pas utilisé par le BUS CAN
 *
 *
 */
void change_state_user_led(uint32_t current_time) {
    static uint32_t time_for_change_state = 0;
    static bool led_state = false;

    if (current_time >= time_for_change_state) {
        if (led_state) {
            led_state = false;
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
            time_for_change_state += USER_LED_LOW_TIME;
        } else {
            led_state = true;
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
            time_for_change_state += USER_LED_HIGH_TIME;
        }
    }
}

void my_loop(void) {

    uint32_t current_time = 0;
    //tests_unitaires();

    while (1) {
        current_time = HAL_GetTick();

        // Lecture état RFID
        read_RFID(current_time);

        // Lecture état IMU

        // Lecture état boutons tactiles

        // Lecture messages CAN

        // Modification état led debug
        change_state_user_led(current_time);

        // Modification état leds boutons tactiles

        // Modification état leds adressables

        // Modification état moteur vibrant

        // Emission message CAN

    }

}

