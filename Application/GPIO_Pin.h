/*
 * GPIO_Pin.h
 *
 *  Created on: May 26, 2025
 *      Author: phifo
 */

#ifndef GPIO_PIN_H_
#define GPIO_PIN_H_

#include "main.h"

class GPIO_Pin {
public:
	GPIO_Pin (GPIO_TypeDef * port, uint16_t pin);
	virtual ~GPIO_Pin();

	void set (void);
	void reset (void);

	GPIO_TypeDef * port_;
	uint16_t pin_;
};

#endif /* GPIO_PIN_H_ */
