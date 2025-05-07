#include "Joystick.h"

uint8_t SPITx[PS2_DATA_LENGTH] = {0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t SPIRx[PS2_DATA_LENGTH];

#define PS2_CS_PORT GPIOC
#define PS2_CS_PIN  GPIO_PIN_10

void PS2_ReadData() {
	HAL_GPIO_WritePin(PS2_CS_PORT, PS2_CS_PIN, GPIO_PIN_RESET);
	for (int i = 0; i < PS2_DATA_LENGTH; i++) {
		HAL_SPI_TransmitReceive(&hspi1, &SPITx[i], &SPIRx[i], 1, HAL_MAX_DELAY);
		HAL_Delay(1);  // delay เล็กๆ เพื่อให้จอยตอบกลับได้ทัน
	}
	HAL_GPIO_WritePin(PS2_CS_PORT, PS2_CS_PIN, GPIO_PIN_SET);
}
