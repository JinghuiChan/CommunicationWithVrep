#ifndef VREP_H
#define VREP_H

#include "VrepInterface.h"
//#include <stdio.h>
//#include <stdlib.h>
#include <thread>
#include <mutex>
#include <string>
#include <chrono>

using namespace std;

extern "C" {
#include "extApi.h"
#include "extApiPlatform.h"
//#include "v_repConst.h"
#include "v_repLib.h"
}

namespace vrep_model {

class VrepImpl : public VrepInterface
{
public:
    VrepImpl();
    virtual ~VrepImpl();

    int Impl_SetVelocity(int16_t left, int16_t right) override;
    int Impl_GetSensorData(uint64_t* timestamp, float* gyro, int32_t* l_encoder, int32_t* r_encoder, uint8_t* bumpers_left, uint8_t* bumpers_right, uint8_t* bumpers_front,  uint8_t* kidnapped) override;
    int Impl_GetImageData(uint8_t** frame_image, int* width, int* height, uint64_t* frame_when) override;
#ifdef HOSLAM
    int GetLdsRanges(float r[360]) override;
    int GetLdsMap(float x, float y, uint32_t w, uint32_t h, float* map_x, float* map_y, float* resolution, uint8_t* buffer, uint32_t size) override;
 #else
    int Impl_GetLdsRanges(float r[360]) override{return 0;}
    int Impl_GetLdsMap(float x, float y, uint32_t w, uint32_t h, float* map_x, float* map_y, float* resolution, uint8_t* buffer, uint32_t size) override
    {
        return 0;
    }
 #endif
    uint64_t now_ms_from_base()override;

private:
    std::thread t;
    bool stopThread;

    int ThreadMain();

    int GetImage(simxInt * resolution, simxUChar ** imageRequest);

    int GetLightTouch(LTBumpMask lt_masks[]);

    int GetLdsRanges_H();

    int GetCliff(CliffMask cliff_maskd[]);
    int GetBumper();

    int GetOdometry();
    void SetTargetVelocity();

    int GetGyro();
    int GetAccelGyro();

    void unpack_floats(uint8_t* buf, float *f, int count);

    bool isTargetMotorIntial;
    int clientImage;
    int clientID;
    std::mutex clientMutex;
    int leftMotorHandle, rightMotorHandle, visionHandle;
    int frontBumperHandle, leftBumperHandle, rightBumperHandle;
    int buttonsHandle;

    std::mutex velMutex;
    bool isVelocitySet;
    int16_t leftVelocity, rightVelocity;

    std::mutex sensorMutex;
    std::condition_variable sensorCV;
    bool isSensorRequest;
    uint64_t sensorTimestamp;
    float sensorGyro;
    int32_t sensorLEncoder, sensorREncoder;
    float leftTotalAux, rightTotalAux;
    uint8_t sensorBumper;
    uint8_t bumpers_left;
    uint8_t bumpers_right;
    uint8_t bumpers_front;
    uint8_t sensorRangeBump;
    float lastLeftPosition;
    float lastRightPosition;
    float totalLeftPosition;
    float totalRightPosition;
    bool vrepButtons[3];
    bool lastVrepButtons[3];
    std::mutex ldsPoseMutex;

    uint8_t cliff_sensor[CLIFF_MAX];
    int cliff_handles[CLIFF_MAX];
    CliffMask cliff_masks[CLIFF_MAX] = {
        CLIFF_MASK_FRONT_CENTER,
        CLIFF_MASK_SIDE_LEFT,
        CLIFF_MASK_SIDE_RIGHT,
    };

    string strCliffNames[CLIFF_MAX] = {
        "cliff_front_sensor",
        "cliff_left_sensor",
        "cliff_right_sensor",
    };

    int lt_handles[LT_MAX];
    LTBumpMask lt_masks[LT_MAX] = {
        LT_BUMP_MASK_RIGHT,
        LT_BUMP_MASK_FRONT_RIGHT,
        LT_BUMP_MASK_FRONT_CENTER_LEFT,
        LT_BUMP_MASK_FRONT_LEFT,
        LT_BUMP_MASK_LEFT,
    };
    string strLTName[LT_MAX] = {
        "lt_right_sensor",
        "lt_front_right_sensor",
        "lt_front_sensor",
        "lt_front_left_sensor",
        "lt_left_sensor",
    };

    robot_cliff_t g_cliff_state;
    robot_ir_range_sensor_t ir_state;

    std::mutex imageMutex;
    std::condition_variable imageCV;
    bool isImageRequest;
    uint8_t* image;
    uint64_t imageTimestamp;
    int imageWidth, imageHeight;

    simxInt resolution[2];
    simxUChar *imageRequest;
    simxUChar **imageRequest2;

    float sensorAccel[6];

    void sleep_ms(int ms);
    uint64_t base_ms = -1;
    uint64_t now_ms();

    //uint64_t base_ms = -1;

    std::string vrep_ip;
    int vrep_remote_port;
    int vrep_camera_port;

};
}

#endif // VREP_H
