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

int main(char argc, char *argv[]) {
    LaneDistanceDetector ldd1("sample/filter_param.xml", "sample/in_external_param.xml", "sample/mask_image.bmp", 0);
    LaneDistanceDetector ldd2("sample/filter_param.xml", "sample/in_external_param.xml", "sample/mask_image.bmp", 2);

    while(1) {
        if (ldd1.ReadFrame()) { break; }
        if (ldd2.ReadFrame()) { break; }
        cout << ldd1.ProccessImage() << endl;
        cout << ldd2.ProccessImage() << endl;
        //ldd1.ViewImage(1, 0, 0, 0, 0, 0, 1);
        //ldd2.ViewImage(1, 0, 0, 0, 0, 0, 1);
        waitKey(1);
    }
    return 0;
}