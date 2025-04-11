/*
 * application.cpp
 *
 *  Created on: Apr 11, 2025
 *      Author: phifo
 */

#include <stdio.h>

#include "main.h"

#include "application.h"

void my_setup(void) {

}


void test_i2cOutput (void)
{
	static bool status = false;

	uint8_t data_on [2] = {0x00, 0x01};
	uint8_t data_off [2] = {0x00, 0x00};

	if (status)
	{
		HAL_I2C_Master_Transmit(&hi2c1, 0x38, data_on, 2, 100);
	}
	else
	{
		HAL_I2C_Master_Transmit(&hi2c1, 0x38, data_off, 2, 100);

	}
}

void my_loop(void) {

	int16_t i = 0;

	while (1) {
		printf("Test : %d \n\r", i++);

		test_i2cOutput ();

		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		HAL_Delay(50);
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		HAL_Delay(450);
	}
}
