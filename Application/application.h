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

#define PERIODE_LECTURE_RFID 100
#define PERIODE_LECTURE_IMU 250
#define PERIODE_LECTURE_TOUCH_BUTTONS 150
#define PERIODE_CAN_BUS_AUTOMATIC_MESSAGE 5000

#define USER_LED_LOW_TIME 950
#define USER_LED_HIGH_TIME 50

#define IMU_MOTION_THRESHOLD  0.01f   // Variation threshold(à ajuster)
#define IMU_IMMOBILE_TIME_MS  2000    // motionless time before validation (ms)

#define VIBRATING_MOTOR_BASE_TIME 10

#define CAN_BUS_RFID_READ_ID 0x1234
#define CAN_BUS_IMU_MOVE_ID   0x1111
#define CAN_BUS_TOUCH_BUTTON_1_ID 0x2222
#define CAN_BUS_TOUCH_BUTTON_2_ID 0x2223

#define CAN_BUS_LED_DEBUG_ID 0x3210
#define CAN_BUS_UART_DEBUG_ID 0x7777
#define CAN_BUS_VIBRATING_MOTOR_ID 0x6789
#define CAN_BUS_NOTIFICATION_LEDS_ID 0xABCD


#define TOUCH_BUTTON_ADDRESS_1    0x38
#define TOUCH_BUTTON_ADDRESS_2    0x48


extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim1;
extern DMA_HandleTypeDef hdma_tim1_ch1;


void my_setup (void);
void my_loop (void);

// Pour les tests uniquement
void config_SPI_before_IMU (void);
void config_SPI_before_RFID (void);
uint16_t can_bus_callback_led(uint16_t sender, uint8_t len, uint8_t data[6]);
uint16_t can_bus_callback_uart_tx(uint16_t sender, uint8_t len, uint8_t data[6]);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_H_ */
