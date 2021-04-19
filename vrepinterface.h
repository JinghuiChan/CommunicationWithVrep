#ifndef VREPINTERFACE_H
#define VREPINTERFACE_H
#include "TypeDef.h"

class VrepInterface
{
public:
    VrepInterface(){}
    virtual ~VrepInterface(){}

    VrepInterface(const VrepInterface&) = delete;
    VrepInterface& operator=(const VrepInterface&) = delete;

    virtual int Impl_SetVelocity(int16_t left, int16_t right) = 0;
    virtual int Impl_GetSensorData(uint64_t* timestamp, float* gyro, int32_t* l_encoder, int32_t* r_encoder, uint8_t* bumpers_left, uint8_t* bumpers_right, uint8_t* bumpers_front,  uint8_t* kidnapped) = 0;
    virtual int Impl_GetImageData(uint8_t** frame_image, int* width, int* height, uint64_t* frame_when) = 0;
    virtual int Impl_GetLdsRanges(float r[360]) = 0;
    virtual int Impl_GetLdsMap(float x, float y, uint32_t w, uint32_t h, float* map_x, float* map_y, float* resolution, uint8_t* buffer, uint32_t size) = 0;
    virtual uint64_t now_ms_from_base() = 0;
};

#endif // VREPINTERFACE_H
