#ifndef INC_JOYSTICK_H_
#define INC_JOYSTICK_H_

#include "stm32g4xx_hal.h"

#define PS2_DATA_LENGTH 8

extern SPI_HandleTypeDef hspi1;
extern uint8_t SPIRx[PS2_DATA_LENGTH];

void PS2_ReadData();

#endif /* INC_JOYSTICK_H_ */
