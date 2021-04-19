#include "GetVrepDataImpl.h"
#include "VrepInterface.h"
#include "Vrep.h"

enum addon_type_e{
    addon_none,
    addon_dataset,
    addon_vrep,
};

addon_type_e addon_type = addon_vrep;

using namespace std;

static VrepInterface *s_addon = NULL;


int impl_get_vrep_data_init()
{
    if(s_addon != NULL)
        return 0;
    switch (addon_type) {
    case addon_dataset:
        break;
    case addon_vrep:
        s_addon = new  vrep_model::VrepImpl();
        break;
    }
    return 0;
}

int impl_get_vrep_data_exit()
{
    if(s_addon != NULL){
        delete s_addon;
        s_addon = NULL;
    }
    return 0;
}

int Impl_SetVelocity(int16_t left, int16_t right)
{
    if(s_addon != NULL)
        return s_addon->Impl_SetVelocity(left, right);
    return 0;
}

int Impl_GetSensorData(uint64_t *timestamp, float *gyro, int32_t *l_encoder, int32_t *r_encoder, uint8_t *bumpers_left, uint8_t *bumpers_right, uint8_t *bumpers_front, uint8_t *kidnapped)
{
    if(s_addon != NULL){
        return s_addon->Impl_GetSensorData(timestamp, gyro, l_encoder, r_encoder, bumpers_left, bumpers_right, bumpers_front, kidnapped);
    }
    return 0;
}

int Impl_GetImageData(uint8_t **frame_image, int *width, int *height, uint64_t *frame_when)
{
    if(s_addon != NULL)
        return s_addon->Impl_GetImageData(frame_image, width, height, frame_when);
    printf("@cjh-test: Impl_GetImageData( null ) \n");
    return 0;
}

int Impl_GetLdsRanges(float r[360])
{
    if(s_addon != NULL)
        return s_addon->Impl_GetLdsRanges(r);
    return 0;
}

int Impl_GetLdsMap(float x, float y, uint32_t w, uint32_t h, float *map_x, float *map_y, float *resolution, uint8_t *buffer, uint32_t size)
{
    if(s_addon != NULL)
        return s_addon->Impl_GetLdsMap(x, y, w, h, map_x, map_y, resolution, buffer, size);
    return 0;
}

uint64_t Impl_timer_ms()
{
    if(s_addon != NULL)
        return s_addon->now_ms_from_base();
    return 0;
}

