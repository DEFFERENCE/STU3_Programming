#include "Encoder.h"

#define Count_PER_REV 12000.0f
#define TWO_PI 6.283185f
#define PI 3.14286f
#define Lead 11.0f

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
    enc->position_degree = enc->position * (180.0f / PI);

    enc->lastRawPosition = rawPosition;
    enc->lastPosition = enc->position;
    enc->lastVelocity = enc->velocity;

    enc->position_mm += ((float)deltaRaw / Count_PER_REV) * Lead;
    enc->velocity_mm = (enc->position_mm - enc->lastPosition_mm) / dt;
    enc->acceleration_mm = (enc->velocity_mm - enc->lastVelocity_mm) / dt;

    enc->lastPosition_mm = enc->position_mm;
    enc->lastVelocity_mm = enc->velocity_mm;
}

void Encoder_setLimit(Encoder *enc, float limit) {
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

float Encoder_GetDegree(Encoder *enc) {
	return enc->position_degree;
}

float Encoder_GetPosition_mm(Encoder *enc) {
    return enc->position_mm;
}

float Encoder_GetVelocity_mm(Encoder *enc) {
    return enc->velocity_mm;
}

float Encoder_GetAcceleration_mm(Encoder *enc) {
    return enc->acceleration_mm;
}
