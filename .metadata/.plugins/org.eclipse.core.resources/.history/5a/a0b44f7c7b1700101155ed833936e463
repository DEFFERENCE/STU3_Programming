#ifndef ENCODER_H
#define ENCODER_H

#include "stm32g4xx_hal.h"

typedef struct {
    TIM_HandleTypeDef *htim;

    int32_t lastRawPosition;

    float position;
    float velocity;
    float acceleration;

    float lastPosition;
    float lastVelocity;
} Encoder;

void Encoder_Init(Encoder *enc, TIM_HandleTypeDef *htim);
void Encoder_Update(Encoder *enc, float dt);

float Encoder_GetPosition(Encoder *enc);
float Encoder_GetVelocity(Encoder *enc);
float Encoder_GetAcceleration(Encoder *enc);

#endif
