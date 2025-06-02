#ifndef INC_JOYSTICK_H_
#define INC_JOYSTICK_H_

#include "stm32g4xx_hal.h"

#define PS2_DATA_LENGTH 8

extern SPI_HandleTypeDef hspi1;
extern uint8_t SPIRx[PS2_DATA_LENGTH];

void PS2_ReadData();

uint8_t PS2_ButtonCircle();
uint8_t PS2_ButtonSquare();
uint8_t PS2_ButtonTriangle();
uint8_t PS2_ButtonCross();
uint8_t PS2_ButtonR1();
uint8_t PS2_ButtonR2();
uint8_t PS2_ButtonSelect();
uint8_t PS2_ButtonStart();
uint8_t PS2_ButtonL1();

void PS2_Move();
void Select_ten_points();
void Auto_Mode();

#endif /* INC_JOYSTICK_H_ */
