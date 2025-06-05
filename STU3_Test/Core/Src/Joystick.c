#include "Joystick.h"

uint8_t SPITx[PS2_DATA_LENGTH] = {0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t SPIRx[PS2_DATA_LENGTH];

#define PS2_CS_PORT GPIOC
#define PS2_CS_PIN  GPIO_PIN_10

void PS2_ReadData() {
//    HAL_Delay(1);
    HAL_GPIO_WritePin(PS2_CS_PORT, PS2_CS_PIN, GPIO_PIN_RESET);

    for (int i = 0; i < PS2_DATA_LENGTH; i++) {
        HAL_SPI_TransmitReceive(&hspi1, &SPITx[i], &SPIRx[i], 1, HAL_MAX_DELAY);
//        HAL_Delay(5);
    }

    HAL_GPIO_WritePin(PS2_CS_PORT, PS2_CS_PIN, GPIO_PIN_SET);
//    HAL_Delay(5);
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

uint8_t PS2_ButtonL2() {
	 return !(SPIRx[4] & 0x01);
}

//void PS2_Move() {
//	if (PS2_ButtonCircle()) {
//		// Move Right (Revolute)
//		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 1); // 0 or 1
//		__HAL_TIM_SET_COMPARE(&htim20,TIM_CHANNEL_3,10000);
//	} else if (PS2_ButtonSquare()) {
//		// Move Left (Revolute)
//		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 0); // 0 or 1
//		__HAL_TIM_SET_COMPARE(&htim20,TIM_CHANNEL_3,10000);
//	} else if (PS2_ButtonTriangle()) {
//		// Move Up (Prismatic)
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 1); // 0 or 1
//		__HAL_TIM_SET_COMPARE(&htim20,TIM_CHANNEL_1,10000);
//	} else if (PS2_ButtonCross()) {
//		// Move Down (Prismatic)
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0); // 0 or 1
//		__HAL_TIM_SET_COMPARE(&htim20,TIM_CHANNEL_1,10000);
//	} else if (PS2_ButtonR1()) {
		// Servo/Pen Move up
//		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 65535);
//	} else if (PS2_ButtonR2()) {
//		// Servo/Pen Move Down
//		 __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 0);
//	}
//}

//void Select_ten_points() {
//	uint8_t selectPressed = PS2_ButtonSelect();
//	static uint8_t prevSelect = 0;
//	if (selectPressed && !prevSelect) {
//		if (count < 10) {
//			PrismaticTenPoints[count] = ((int) (Encoder_GetPosition_mm(&encoder1)) * 10) / 10.0f;
//			RevoluteTenPoints[count] = ((int) Encoder_GetDegree(&encoder2) * 10) / 10.0f;
//			count += 1;
//		}
//	}
//	prevSelect = selectPressed;
//	HAL_Delay(50);
//}
