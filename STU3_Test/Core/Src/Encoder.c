#include "Encoder.h"

#define Count_PER_REV 12000.0f
#define TWO_PI 6.283185f

void Encoder_Init(Encoder *enc, TIM_HandleTypeDef *htim) {
    enc->htim = htim;
    HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);

    enc->lastRawPosition = __HAL_TIM_GET_COUNTER(htim);
    enc->position = 0;
    enc->velocity = 0;
    enc->acceleration = 0;
    enc->lastPosition = 0;
    enc->lastVelocity = 0;
}

void Encoder_Update(Encoder *enc, float dt) {
    int32_t rawPosition = __HAL_TIM_GET_COUNTER(enc->htim);
    int32_t deltaRaw = rawPosition - enc->lastRawPosition;

    // Handle 16-bit counter wrap-around
    if (deltaRaw > 30000) {
        deltaRaw -= 60000;
    } else if (deltaRaw < -30000) {
        deltaRaw += 60000;
    }

    enc->position += ((float)deltaRaw * TWO_PI) / Count_PER_REV;
    enc->velocity = (enc->position - enc->lastPosition) / dt;
    enc->acceleration = (enc->velocity - enc->lastVelocity) / dt;

    enc->lastRawPosition = rawPosition;
    enc->lastPosition = enc->position;
    enc->lastVelocity = enc->velocity;
}

void Encoder_setLimit(Encoder *enc, float limit)
{
		enc->position = limit;
}

float Encoder_GetPosition(Encoder *enc) {
    return enc->position;
}

float Encoder_GetVelocity(Encoder *enc) {
    return enc->velocity;
}

float Encoder_GetAcceleration(Encoder *enc) {
    return enc->acceleration;
}
