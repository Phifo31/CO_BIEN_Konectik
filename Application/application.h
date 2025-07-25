/*
 * application.h
 *
 *  Created on: Apr 11, 2025
 *      Author: phifo
 */

#ifndef APPLICATION_H_
#define APPLICATION_H_

#ifdef __cplusplus
extern "C" {
#endif

extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;

void config_SPI_before_IMU (void);
void config_SPI_before_RFID (void);


void my_setup (void);
void my_loop (void);



#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_H_ */
