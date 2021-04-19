#include <QCoreApplication>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include<iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "GetVrepDataImpl.h"
#include "testfunc.h"

extern "C" {
#include "extApi.h"
#include "extApiPlatform.h"
//#include "v_repConst.h"
#include "v_repLib.h"
}

using namespace std;
//using namespace cv;

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    Impl_timer_ms();
    impl_get_vrep_data_init();

    while(1){
        testFunc();
        Sleep(100);
    }
    return a.exec();
}
