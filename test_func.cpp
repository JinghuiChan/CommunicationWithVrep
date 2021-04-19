#include "GetVrepDataImpl.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include<iostream>
#include "testfunc.h"


int testFunc(){
    //--------- image data ---------
    uint8_t * imageRequest;
    imageRequest = (uint8_t *)malloc(320 * 240 * sizeof (uint8_t));
    int width;
    int height;
    uint64_t frame_when;
    Impl_GetImageData(&imageRequest, &width, &height, &frame_when);
    printf("imageData(%d, %d, %lld) \n", width, height, frame_when);
    //--------- image data ---------


    //--------- sensor data ---------
    uint64_t timestamp;
    float gyro;
    int32_t l_encoder;
    int32_t r_encoder;
    uint8_t bumpers_left;
    uint8_t bumpers_right;
    uint8_t bumpers_front;
    uint8_t kidnapped;
    Impl_GetSensorData(&timestamp, &gyro, &l_encoder, &r_encoder, &bumpers_left, &bumpers_right, &bumpers_front, &kidnapped);
    printf("sensor data(%lld, %f, %d, %d, %hhu, %hhu, %hhu, %hhu) \n", timestamp, gyro, l_encoder, r_encoder, bumpers_left, bumpers_right, bumpers_front, kidnapped);
   //--------- sensor data ---------


    return 0;
}
