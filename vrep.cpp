#include "Vrep.h"
#include <string>
#include "UseOpencv.h"
#include "VrepConfig.h"

using namespace std;
using namespace chrono;

namespace vrep_model {
void VrepImpl::unpack_floats(uint8_t* buf, float *f, int count){
    for (int i = 0; i < count; ++i) {
        float* v = &f[i];
        ((char*)v)[0] = buf[sizeof(float)*i + 0];
        ((char*)v)[1] = buf[sizeof(float)*i + 1];
        ((char*)v)[2] = buf[sizeof(float)*i + 2];
        ((char*)v)[3] = buf[sizeof(float)*i + 3];
    }
}

void VrepImpl::sleep_ms(int ms)
{
    this_thread::sleep_for(chrono::milliseconds(ms));
}

uint64_t VrepImpl::now_ms()
{
    return time_point_cast<milliseconds>(system_clock::now()).time_since_epoch().count();
}

int VrepImpl::GetOdometry()
{
    simxFloat Wheel;

    int ret = simxGetJointPosition(clientID, leftMotorHandle, &Wheel, /*simx_opmode_buffer*/simx_opmode_streaming);

    if (ret == simx_return_ok) {
        simxFloat diff = Wheel - lastLeftPosition;
        simxFloat aux;
        if (diff >= 0)
            aux = fmod((diff + M_PI), (M_PI * 2)) - M_PI;
        else
            aux = fmod((diff - M_PI), (M_PI * 2)) + M_PI;
        lastLeftPosition = Wheel;
        leftTotalAux += aux;
        sensorLEncoder = leftTotalAux*TICKS_PER_RAD;
    }

    ret = simxGetJointPosition(clientID, rightMotorHandle, &Wheel, /*simx_opmode_buffer*/simx_opmode_streaming);

    if (ret == simx_return_ok) {
        simxFloat diff = Wheel - lastRightPosition;
        simxFloat aux;
        if (diff >= 0)
            aux = fmod((diff + M_PI), (M_PI * 2)) - M_PI;
        else
            aux = fmod((diff - M_PI), (M_PI * 2)) + M_PI;
        lastRightPosition = Wheel;
        rightTotalAux += aux;
        sensorREncoder = rightTotalAux*TICKS_PER_RAD;
    }
    return ret;
}

void VrepImpl::SetTargetVelocity()
{
    if (isVelocitySet) {
        int ret = simxGetObjectHandle(clientID, "wheel_left_joint", &leftMotorHandle, simx_opmode_blocking);
        ret = simxGetObjectHandle(clientID, "wheel_right_joint", &rightMotorHandle, simx_opmode_blocking);

        simxSetJointTargetVelocity(clientID, leftMotorHandle, leftVelocity * 0.02819284, simx_opmode_streaming);
        simxSetJointTargetVelocity(clientID, rightMotorHandle, rightVelocity * 0.02819284, simx_opmode_streaming);
        isVelocitySet = false;
    }

}

int VrepImpl::GetGyro(){
    simxUChar* signal;
    simxInt length;
    bool valid = false;
    int ret = simxReadStringStream(clientID, "gyro", &signal, &length, /*simx_opmode_buffer*/simx_opmode_streaming);
    if (ret == simx_return_ok) {
        if (length >= 4) {
            float gyro;
            unpack_floats(signal, &gyro, 1);
            sensorGyro = gyro;
            valid = true;
        }
    }

    return valid ? 0 : -1;
}

VrepImpl::VrepImpl():
    stopThread(false),
            sensorGyro(0), sensorLEncoder(0), sensorREncoder(0), leftTotalAux(0), rightTotalAux(0), sensorBumper(0), sensorRangeBump(0),
            image(0), imageWidth(DEFAULT_IMAGE_WIDTH), imageHeight(DEFAULT_IMAGE_HEIGHT),base_ms(-1),
            lastLeftPosition(0.f), lastRightPosition(0.f), isVelocitySet(false), isSensorRequest(false), isImageRequest(false)
            , vrepButtons{ 0 }, lastVrepButtons{0}, imageRequest(NULL), resolution{ 0 }, imageTimestamp(20), clientImage(-1), sensorAccel{ 0 }
{
    image = (uint8_t *)malloc(DEFAULT_IMAGE_WIDTH*DEFAULT_IMAGE_HEIGHT*sizeof(uint8_t));
    imageRequest= (uint8_t *)malloc(DEFAULT_IMAGE_WIDTH*DEFAULT_IMAGE_HEIGHT * sizeof(uint8_t));
    //TODO

    clientID = simxStart("127.0.0.1", /*VREP_PORT*/32630, true, true, 2000, 5);
    printf("connect to client :%d\r\n", clientID);

    int temptime = 500;
    while (clientID == -1)//????WTF
        {
          while (temptime)
            {
              temptime--;
             }
          clientID = simxStart("127.0.0.1", VREP_PORT, true, true, 2000, 5);
          temptime = 500;
         }
     temptime = 0;
     t = std::thread(&VrepImpl::ThreadMain, this);
     SetThreadPriority(t.native_handle(), THREAD_PRIORITY_TIME_CRITICAL);
}

VrepImpl::~VrepImpl()
{
    //TODO
    free(image);
    free(imageRequest);
    stopThread = true;
    if (t.joinable()) t.join();
}

int VrepImpl::Impl_SetVelocity(int16_t left, int16_t right)
{

    leftVelocity = left == 0 ? 1 : left;
    rightVelocity = right == 0 ? 1 : right;

    isVelocitySet = true;
    return 0;
}

int VrepImpl::Impl_GetSensorData(uint64_t* timestamp, float* gyro, int32_t* l_encoder, int32_t* r_encoder, uint8_t* bumpers_left_, uint8_t* bumpers_right_, uint8_t* bumpers_front_,  uint8_t* kidnapped)
{
    if (clientID != -1) {
        int ret = GetGyro();
        ret = GetOdometry();
        ret = GetBumper();

        *timestamp = now_ms_from_base(); //not timestemp, FPGA add timestamp because not sync bettwen this project and FPGA
        *gyro = sensorGyro;
        *l_encoder = sensorLEncoder;
        *r_encoder = sensorREncoder;
        *bumpers_left_ = bumpers_left;
        *bumpers_right_ = bumpers_right;
        *bumpers_front_ = bumpers_front;
        *kidnapped = 0;
        return 0;
    }
    return -1;
}

int VrepImpl::Impl_GetImageData(uint8_t **frame_image, int *width, int *height, uint64_t *frame_when)
{
    int ret = GetImage(resolution, frame_image);
    if (!ret)
        {
            *width = resolution[0];
            *height = resolution[1];
            *frame_when = imageTimestamp;
            return 0;
        }
    return -1;
}

uint64_t VrepImpl::now_ms_from_base()
{
    if (base_ms == -1) {
        base_ms = now_ms();
        return base_ms;
    }
    return now_ms() - base_ms;
}

int vrep_model::VrepImpl::ThreadMain()
{
    int ret;
    if(clientID != -1){
        //get handles
        ret = simxGetObjectHandle(clientID, "wheel_left_joint", &leftMotorHandle, simx_opmode_blocking);
        printf("leftMotorHandle=%d\r\n", leftMotorHandle);
        ret = simxGetObjectHandle(clientID, "wheel_right_joint", &rightMotorHandle, simx_opmode_blocking);
        printf("rightMotorHandle=%d\r\n", rightMotorHandle);
        ret = simxGetObjectHandle(clientID, "camera", &visionHandle, simx_opmode_blocking);
        printf("visionHandle=%d\r\n", visionHandle);
        ret = simxGetObjectHandle(clientID, "bumper_front_joint", &frontBumperHandle, simx_opmode_blocking);
        printf("frontBumperHandle=%d\r\n", frontBumperHandle);
        ret = simxGetObjectHandle(clientID, "bumper_left_joint", &leftBumperHandle, simx_opmode_blocking);
        printf("leftBumperHandle=%d\r\n", leftBumperHandle);
        ret = simxGetObjectHandle(clientID, "bumper_right_joint", &rightBumperHandle, simx_opmode_blocking);
        printf("rightBumperHandle=%d\r\n", rightBumperHandle);
        ret = simxGetUIHandle(clientID, "Kobuki_Buttons_UI", &buttonsHandle, simx_opmode_blocking);
        printf("buttonsHandle=%d\r\n", buttonsHandle);
        simxInt buttonState[2], buttonid = -1;
        for (int i = 0; i < sizeof(cliff_handles) / sizeof(int); i++)
        {
            ret = simxGetObjectHandle(clientID, strCliffNames[i].c_str(), &cliff_handles[i], simx_opmode_blocking);

            float detectedPt[4], detectN[3];
            int detectedObject;
            uint8_t uState;
            ret = simxReadProximitySensor(clientID, cliff_handles[i], &uState, detectedPt, &detectedObject,
                detectN, simx_opmode_streaming);
        }
#if 0
        for (int i = 0; i < sizeof(lt_handles) / sizeof(int); i++)
        {
            ret = simxGetObjectHandle(clientID, strLTNames[i].c_str(), &lt_handles[i], simx_opmode_blocking);
            float detectedPt[4], detectN[3];
            int detectedObject;
            uint8_t uState;
            ret = simxReadProximitySensor(clientID, lt_handles[i], &uState, detectedPt, &detectedObject,
                detectN, simx_opmode_streaming);
        }
#endif
        //start
        uint64_t start_time = now_ms_from_base();
        while (!stopThread && simxGetConnectionId(clientID) != -1){
            //press button?
            ret = simxGetUIEventButton(clientID, buttonsHandle, &buttonid, buttonState, simx_opmode_oneshot);
            for (int i = 0; i < 3; i++) {
                if (buttonid == i + 3)
                    vrepButtons[i] = buttonState[1];
                else
                    vrepButtons[i] = false;
            }
            if (!lastVrepButtons[0] && vrepButtons[0]) {
                printf("button0:ok!!!\n");//test button
                //event_send(&app_ctrl_event_queue, AC_E_START_CLEANING, 0);
            }
            if (!lastVrepButtons[1] && vrepButtons[1]) {
                printf("button1:ok!!!\n");//test button
             }
            memcpy(lastVrepButtons, vrepButtons, 3);

            SetTargetVelocity();
#if 0
            if(isSensorRequest){
                ret = GetOdometry();
                ret = GetBumper();
                ret = GetGyro();
                isSensorRequest = false;
            }
#endif
#if 0
            if(isImageRequest){
                ret = GetImage(resolution, imageRequest2);
            }
#endif

            //sleep
            sleep_ms(10);
        }
        int pingTime;
        simxGetPingTime(clientID, &pingTime);
        simxFinish(clientID);
        clientID = -1;
    }
    return 0;
}

int VrepImpl::GetImage(simxInt *resolution, simxUChar **imageRequest){
    int ret;

    ret = simxGetObjectHandle(clientID, "camera", &visionHandle, simx_opmode_buffer);
    ret = simxGetVisionSensorImage(clientID, visionHandle, resolution, imageRequest, 1, simx_opmode_streaming);

    if(ret == simx_return_ok){
        imageTimestamp = now_ms_from_base();
#ifdef USE_OPENCV
        //--------------- show image with opencv -----------
        cv::namedWindow("image320x240",CV_WINDOW_AUTOSIZE);
        cv::Mat channel(240, 320, CV_8UC1, *imageRequest);
        cv::flip(channel, channel, 0);
        cv::imshow("image320x240", channel);
        cv::waitKey(10);
        //--------------- show image with opencv -----------
#endif
    }
    return ret;
}

int VrepImpl::GetLightTouch(LTBumpMask lt_masks[])
{
    //TODO
    /*
    int ret;
    uint8_t lt_mask = 0;
    ir_state.timestamp = sensorTimestamp;
    memset((void*)&ir_state, 0, sizeof (ir_state));
    uint16_t *shift_value = (uint16_t*)&ir_state;

    for (int i = 0; i < sizeof(lt_handles) / sizeof(int); i++)
    {
        //ret = simxGetJointPosition(clientID, cliff_handles[i], &cliff_value, simx_opmode_buffer);
        float detectedPt[4] = { 0 }, detectN[3] = { 0 };
        int detectedObject;
        uint8_t uState;
        shift_value[i] = 1000;

        ret = simxReadProximitySensor(clientID, lt_handles[i], &uState, detectedPt, &detectedObject,
            detectN, simx_opmode_buffer);

        if (ret != simx_return_ok)
        {
            continue;
        }
        if (uState == 0)
        {
            continue;
        }
        if (detectedObject == 0)
        {
            continue;
        }

        if (detectedPt[2] <= 0.8)
        {
            shift_value[i] = detectedPt[2] * 1000;
        }
        else
        {
            shift_value[i] = 1000;
        }
        if (detectedPt[2] <= 0.2)
        {
            lt_mask |= lt_masks[i];
        }
        else
        {
            lt_mask &= ~lt_masks[i];
        }

    }

    //if (lt_mask)
    {
        //lt_mask |= WALL_IR_MASK_FRONT_CENTER;
        robot_set_wall_ir(&ir_state, (light_bump_t*)&lt_mask);
        if(ir_state.right > 255) {
            sensorRangeBump = 255;
        } else {
            sensorRangeBump = ir_state.right;
        }
    }
    return ret;
*/
    return -1;
}

int VrepImpl::GetLdsRanges_H()
{
    //TODO
    return -1;
}

int VrepImpl::GetCliff(CliffMask cliff_maskd[])
{
    //TODO
    return -1;
}

int VrepImpl::GetBumper()
{
    int ret;
    simxFloat Bumper = 0.f;
    ret = simxGetJointPosition(clientID, frontBumperHandle, &Bumper, /*simx_opmode_buffer*/simx_opmode_streaming);

    if (ret == simx_return_ok && Bumper < -0.001) {
        bumpers_front = 1;
    }
    ret = simxGetJointPosition(clientID, leftBumperHandle, &Bumper, /*simx_opmode_buffer*/simx_opmode_streaming);
    if (ret == simx_return_ok && Bumper < -0.001) {
        bumpers_left = 1;
    }
    ret = simxGetJointPosition(clientID, rightBumperHandle, &Bumper, /*simx_opmode_buffer*/simx_opmode_streaming);
    if (ret == simx_return_ok && Bumper < -0.001) {
        bumpers_right = 1;
    }
    return ret;
}

int VrepImpl::GetAccelGyro(){
    simxUChar* signal;
    simxInt length;
    bool valid = false;
    if (simxReadStringStream(clientID, "accelgyro", &signal, &length, simx_opmode_buffer) == simx_return_ok) {
        if (length > 6 * 4) {
            unpack_floats(signal, sensorAccel, 6);
            uint64_t time_us = (uint64_t)(atof((const char*)&signal[6 * 4]) * 1000000L);
            //SendImuData(sensorAccel, time_us);
            valid = true;
        }
    }
    return valid ? 0 : -1;
}

}
