#include <iostream>
#include <iomanip>
#include <sstream>
#include "lawicel_can.h"

#include "side_lane_distance.hpp"


using namespace std;

union MultiTypeUnion {
    float float32[2];
    uint_least8_t uint8[8];
    int_least32_t int32[2];
    uint_least32_t uint32[2];
    MultiTypeUnion() {
        uint32[0] = 0;
        uint32[1] = 0;
    }
};


int main(char argc, char *argv[]) {
    //現在日時を取得する
    time_t time_raw = time(nullptr);

    //形式を変換する
    tm date_raw;
    localtime_s(&date_raw, &time_raw);

    //sに独自フォーマットになるように連結していく
    std::stringstream s;
    s << "20";
    s << setfill('0') << setw(2) << date_raw.tm_year - 100; //100を引くことで20xxのxxの部分になる
    s << setfill('0') << setw(2) << date_raw.tm_mon + 1; //月を0からカウントしているため
    s << setfill('0') << setw(2) << date_raw.tm_mday; //そのまま
    s << setfill('0') << setw(2) << date_raw.tm_hour; //そのまま
    s << setfill('0') << setw(2) << date_raw.tm_min; //そのまま
    s << setfill('0') << setw(2) << date_raw.tm_sec; //そのまま
    std::string video_file_name1 = s.str() + "_1.avi";
    std::string video_file_name2 = s.str() + "_2.avi";

    int frame_count = 0;


    CANHANDLE canhandle = -1;

    MultiTypeUnion can_data1;
    CANMsg canmsg_l;
    canmsg_l.id = 0x124;
    canmsg_l.len = 8;
    canmsg_l.flags = 0;

    MultiTypeUnion can_data2;
    CANMsg canmsg_r;
    canmsg_r.id = 0x125;
    canmsg_r.len = 8;
    canmsg_r.flags = 0;

    if (!((canhandle = canusb_Open(NULL, "500", CANUSB_ACCEPTANCE_CODE_ALL, CANUSB_ACCEPTANCE_MASK_ALL, CANUSB_FLAG_TIMESTAMP)) > 0)) {
        cout << "Can't find tha CANUSB device." << endl;
        cout << "It will be runned on No CANUSB Mode" << endl;
    }
    Sleep(1000);


	LaneDistanceDetector ldd1("sample/filter_param.xml", "sample/in_external_param.xml", "sample/mask_image.bmp", 0, "testl.wmv");
	LaneDistanceDetector ldd2("sample/filter_param.xml", "sample/in_external_param.xml", "sample/mask_image.bmp", 1, "testr.wmv");


    while(1) {
        if (ldd1.ReadFrame()) { break; }
        if (ldd2.ReadFrame()) { break; }
        can_data1.float32[0] = ldd1.ProccessImage();
        can_data2.float32[0] = ldd2.ProccessImage();
        cout << can_data1.float32[0] << endl;
        cout << can_data2.float32[0] << endl;
		ldd1.ViewImage(1, 0, 0, 0, 0, 0, 1);
		ldd2.ViewImage(1, 0, 0, 0, 0, 0, 1);
		cv::waitKey(1);

        // Write CAN message
        if (canhandle > 0) {
            for (int i = 0; i < 8; ++i) {
                canmsg_l.data[i] = can_data1.uint8[i];
                canmsg_r.data[i] = can_data2.uint8[i];
            }
            canusb_Write(canhandle, &canmsg_l);
            canusb_Write(canhandle, &canmsg_r);
        }

        if (cv::waitKey(1) == 27) { break; }
        ++frame_count;
    }

    if (canhandle > 0) { canusb_Close(canhandle); }
    return 0;
}