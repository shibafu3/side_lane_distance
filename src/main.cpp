#ifdef __WIN32
#ifdef _DEBUG
//Debugモードの場合
#pragma comment(lib,"D:/home/Sources/opencv/opencv310/build/install/lib/opencv_world310d.lib")
#else
//Releaseモードの場合
#pragma comment(lib,"D:/home/Sources/opencv/opencv310/build/install/lib/opencv_world310.lib") 
#endif

#ifdef _DEBUG
//Debugモードの場合
#pragma comment(lib,"D:/home/Sources/CANUSB/lib/canusbdrv64.lib")
#else
//Releaseモードの場合
#pragma comment(lib,"D:/home/Sources/CANUSB/lib/canusbdrv64.lib")
#endif
#endif

#include <iostream>

#include "side_lane_distance.hpp"


using namespace std;
using namespace cv;

int main() {
    LaneDistanceDetector ldd("data/filter_param.xml", "data/in_external_param.xml", "data/mask_image.bmp", "data/Video2_20180625183321.avi");

    while(1) {
        if (ldd.ReadFrame()) { break; }
        ldd.ProccessImage();
        ldd.ViewImage(1, 0, 0, 0, 0, 0, 1);
        waitKey(1);
    }
    return 0;
}