#ifndef ENCODER_H
#define ENCODER_H

#include "stm32g4xx_hal.h"

typedef struct {
    TIM_HandleTypeDef *htim;

    int32_t lastRawPosition;

    float position;
    float velocity;
    float acceleration;
    float position_degree;

    float lastPosition;
    float lastVelocity;

    float position_mm;
    float velocity_mm;
    float acceleration_mm;

    float lastPosition_mm;
    float lastVelocity_mm;
} Encoder;

void Encoder_Init(Encoder *enc, TIM_HandleTypeDef *htim);
void Encoder_Update(Encoder *enc, float dt);
void Encoder_setLimit(Encoder *enc, float limit);

float Encoder_GetPosition(Encoder *enc);
float Encoder_GetVelocity(Encoder *enc);
float Encoder_GetAcceleration(Encoder *enc);
float Encoder_GetDegree(Encoder *enc);
float Encoder_GetPosition_mm(Encoder *enc);
float Encoder_GetVelocity_mm(Encoder *enc);
float Encoder_GetAcceleration_mm(Encoder *enc);

#endif
