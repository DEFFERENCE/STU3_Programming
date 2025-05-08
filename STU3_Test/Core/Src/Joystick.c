#include "Joystick.h"

uint8_t SPITx[PS2_DATA_LENGTH] = {0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t SPIRx[PS2_DATA_LENGTH];

#define PS2_CS_PORT GPIOC
#define PS2_CS_PIN  GPIO_PIN_10

void PS2_ReadData() {
	HAL_Delay(1);
	HAL_GPIO_WritePin(PS2_CS_PORT, PS2_CS_PIN, GPIO_PIN_RESET);

	for (int i = 0; i < PS2_DATA_LENGTH; i++) {
		HAL_SPI_TransmitReceive(&hspi1, &SPITx[i], &SPIRx[i], 1, HAL_MAX_DELAY);
		HAL_Delay(1);
	}

	HAL_GPIO_WritePin(PS2_CS_PORT, PS2_CS_PIN, GPIO_PIN_SET);
	HAL_Delay(1);
}

uint8_t PS2_ButtonCircle() {
	return !(SPIRx[4] & 0x20);
}

uint8_t PS2_ButtonSquare() {
	return !(SPIRx[4] & 0x80);
}

uint8_t PS2_ButtonTriangle() {
	return !(SPIRx[4] & 0x10);
}

uint8_t PS2_ButtonCross() {
	return !(SPIRx[4] & 0x40);
}

uint8_t PS2_ButtonR1() {
	return !(SPIRx[4] & 0x08);
}

uint8_t PS2_ButtonR2() {
	return !(SPIRx[4] & 0x02);
}

uint8_t PS2_ButtonSelect() {
	return !(SPIRx[3] & 0x01);
}

uint8_t PS2_ButtonStart() {
	return !(SPIRx[3] & 0x08);
}
