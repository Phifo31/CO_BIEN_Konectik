/*
 * can_bus.h
 *
 *  Created on: Oct 2, 2025
 *      Author: phifo
 */

#ifndef CAN_BUS_H_
#define CAN_BUS_H_

#define MAX_CALLBACK_FUNCTIONS 10

typedef struct
{
    arbitrationId_t ident;
    uint16_t (* function_pointer) (uint16_t sender, uint8_t data [6]);
} callback_function_t;


/*!
 *    @brief  Class that stores state and functions for interacting with
 *            the stm32 CAN peripheral
 */
class CAN_BUS {
public:
  CAN_BUS (uint16_t ident);
  void begin (void);
  HAL_StatusTypeDef send (uint8_t * buffer,  uint8_t len);
  HAL_StatusTypeDef register_callback_function (arbitrationId_t filtre, uint16_t (*fp)(uint16_t, uint8_t*));

private:

  uint16_t id_;
  FDCAN_TxHeaderTypeDef txHeader_;
  uint8_t txData_[12];
};


#endif /* CAN_BUS_H_ */
