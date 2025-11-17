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

//#define PRINT_DEBUG 1
#define ENABLE_USER_LOG   1
#define ENABLE_DEBUG_LOG  0

#define ENDLESS_LOOP 1

#define PERIODE_LECTURE_RFID 50
#define PERIODE_LECTURE_IMU 250
#define PERIODE_LECTURE_TOUCH_BUTTONS 150
#define PERIODE_CAN_BUS_AUTOMATIC_MESSAGE 10000

#define USER_LED_LOW_TIME 950
#define USER_LED_HIGH_TIME 50

#define IMU_MOTION_THRESHOLD  0.01f   // Variation threshold(à ajuster)
#define IMU_IMMOBILE_TIME_MS  2000    // motionless time before validation (ms)

#define VIBRATING_MOTOR_BASE_TIME 10

#define TOUCH_BUTTON_ADDRESS_1    0x38
#define TOUCH_BUTTON_ADDRESS_2    0x48
#define TOUCH_BUTTON_ADDRESS_3    0x68

#define MAX_LED 100
#define USE_BRIGHTNESS 1

//#define LEDS_STRIPS_J5_NB_LEDS 28
//#define LEDS_STRIPS_J6_NB_LEDS 2
//#define LEDS_STRIPS_J7_NB_LEDS 31

#define LEDS_STRIPS_J5_NB_LEDS 5
#define LEDS_STRIPS_J6_NB_LEDS 6
#define LEDS_STRIPS_J7_NB_LEDS 7

extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart2;
extern FDCAN_HandleTypeDef hfdcan1;
extern DMA_HandleTypeDef hdma_tim1_ch1;
extern DMA_HandleTypeDef hdma_tim1_ch3;

void my_setup(void);
void my_loop(void);


// Sortie texte

extern uint16_t seconds;
extern uint16_t reste;
uint16_t msec2sec(uint32_t n, uint16_t *reste);

#if ENABLE_USER_LOG
#define USER_LOG(fmt, ...) ({ \
            seconds = msec2sec(HAL_GetTick(), &reste); \
            printf("%05u:%03u - ", seconds, reste); \
            printf("[USER] " fmt "\r\n", ##__VA_ARGS__); })
#else
  #define USER_LOG(fmt, ...)
#endif

#if ENABLE_DEBUG_LOG
#define DEBUG_LOG(fmt, ...) ({ \
            seconds = msec2sec(HAL_GetTick(), &reste); \
            printf("%05u:%03u - ", seconds, reste); \
            printf("[DEBUG] " fmt "\r\n", ##__VA_ARGS__); })
#else
  #define DEBUG_LOG(fmt, ...) UNUSED(fmt)
#endif

// Pour les tests uniquement
void config_SPI_before_IMU(void);
void config_SPI_before_RFID(void);
uint16_t can_bus_callback_led(uint16_t sender, uint8_t data[6]);
uint16_t can_bus_callback_uart_tx(uint16_t sender, uint8_t data[6]);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_H_ */
