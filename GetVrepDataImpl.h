#ifndef GETVREPDATA_H
#define GETVREPDATA_H
#include "TypeDef.h"
#ifdef __cplusplus
extern "C"{
#endif

int impl_get_vrep_data_init();
int impl_get_vrep_data_exit();
int Impl_SetVelocity(int16_t left, int16_t right);
int Impl_GetSensorData(uint64_t* timestamp, float* gyro, int32_t* l_encoder, int32_t* r_encoder, uint8_t* bumpers_left, uint8_t* bumpers_right, uint8_t* bumpers_front,  uint8_t* kidnapped);
int Impl_GetImageData(uint8_t** frame_image, int* width, int* height, uint64_t* frame_when);
int Impl_GetLdsRanges(float r[360]);
int Impl_GetLdsMap(float x, float y, uint32_t w, uint32_t h, float* map_x, float* map_y, float* resolution, uint8_t* buffer, uint32_t size);
uint64_t Impl_timer_ms();


#ifdef __cplusplus
}
#endif
#endif // GETVREPDATA_H
