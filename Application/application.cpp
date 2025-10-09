/*
 * application.cpp
 *
 *  Created on: Apr 11, 2025
 *      Author: phifo
 */

#include <stdio.h>

#include "main.h"
#include "../../Common/common_data.h"
#include "tests.h"
#include "can_bus.h"

#include "application.h"

// Déclarations externes
extern Adafruit_BNO08x bno08x;

// --- Mapping des broches IMU ---
#define BNO08X_CS_Pin     IMU_CS_Pin
#define BNO08X_CS_Port    IMU_CS_GPIO_Port
#define BNO08X_INT_Pin    IMU_IRQ_Pin
#define BNO08X_INT_Port   IMU_IRQ_GPIO_Port
#define BNO08X_RESET_Pin  IMU_RST_Pin
#define BNO08X_RESET_Port IMU_RST_GPIO_Port


void tests_unitaires(void);

void config_SPI_before_IMU (void){

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
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
      Error_Handler();
    }
}

void config_SPI_before_RFID (void){

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
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
      Error_Handler();
    }
}


void my_setup(void) {
		printf("Initialisation IMU...\n");

	    // Configurer SPI pour l’IMU
	    config_SPI_before_IMU();

	    // Faire le reset matériel du BNO08x
	    HAL_GPIO_WritePin(BNO08X_RESET_Port, BNO08X_RESET_Pin, GPIO_PIN_RESET);
	    HAL_Delay(10);
	    HAL_GPIO_WritePin(BNO08X_RESET_Port, BNO08X_RESET_Pin, GPIO_PIN_SET);
	    HAL_Delay(100);

	    printf("Reset IMU effectué.\n");

}

void my_loop(void) {
	tests_unitaires();
}

