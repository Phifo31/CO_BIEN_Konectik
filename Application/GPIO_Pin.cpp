/*
 * GPIO_Pin.cpp
 *
 *  Created on: May 26, 2025
 *      Author: phifo
 */

#include <GPIO_Pin.h>

GPIO_Pin::GPIO_Pin(GPIO_TypeDef *port, uint16_t pin) {
	// TODO Auto-generated constructor stub
	port_ = port;
	pin_ = pin;
}

GPIO_Pin::~GPIO_Pin() {
	// TODO Auto-generated destructor stub
}

void GPIO_Pin::set (void) {
	// TODO Auto-generated destructor stub
	HAL_GPIO_WritePin(port_, pin_, GPIO_PIN_SET);
}

void GPIO_Pin::reset (void) {
	// TODO Auto-generated destructor stub
	HAL_GPIO_WritePin(port_, pin_, GPIO_PIN_RESET);
}
