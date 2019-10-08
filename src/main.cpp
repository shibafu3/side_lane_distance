#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
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
	std::stringstream s, t;
	s << "20";
	s << setfill('0') << setw(2) << date_raw.tm_year - 100; //100を引くことで20xxのxxの部分になる
	s << setfill('0') << setw(2) << date_raw.tm_mon + 1; //月を0からカウントしているため
	s << setfill('0') << setw(2) << date_raw.tm_mday; //そのまま
	s << setfill('0') << setw(2) << date_raw.tm_hour; //そのまま
	s << setfill('0') << setw(2) << date_raw.tm_min; //そのまま
	s << setfill('0') << setw(2) << date_raw.tm_sec; //そのまま
	t << setfill('0') << setw(2) << date_raw.tm_mday; //そのまま
	t << setfill('0') << setw(2) << date_raw.tm_hour; //そのまま
	t << setfill('0') << setw(2) << date_raw.tm_min; //そのまま
	t << setfill('0') << setw(2) << date_raw.tm_sec; //そのまま
	std::string video_file_name1 = s.str() + "_1.avi";
	std::string video_file_name2 = s.str() + "_2.avi";


    string filter_param_l_file_path = "sample/filter_param.xml";
    string filter_param_r_file_path = "sample/filter_param.xml";
    string in_external_paraml_l_file_path = "sample/in_external_param.xml";
    string in_external_paraml_r_file_path = "sample/in_external_param.xml";
    string mask_image_l_file_path = "sample/mask_image.bmp";
    string mask_image_r_file_path = "sample/mask_image.bmp";
    string write_video_l_file_path = "test_l.wmv";
    string write_video_r_file_path = "test_r.wmv";
    int camera_num1 = 0;
    int camera_num2 = 1;
	if (argc == 9) {
		camera_num1 = stoi(argv[1]);
		camera_num2 = stoi(argv[2]);
		filter_param_l_file_path = string(argv[3]);
		filter_param_r_file_path = string(argv[4]);
		in_external_paraml_l_file_path = string(argv[5]);
		in_external_paraml_r_file_path = string(argv[6]);
		mask_image_l_file_path = string(argv[7]);
		mask_image_r_file_path = string(argv[8]);
		write_video_l_file_path = video_file_name1;
		write_video_r_file_path = video_file_name2;
	}
	if (argc == 11) {
		camera_num1 = stoi(argv[1]);
		camera_num2 = stoi(argv[2]);
		filter_param_l_file_path = string(argv[3]);
		filter_param_r_file_path = string(argv[4]);
		in_external_paraml_l_file_path = string(argv[5]);
		in_external_paraml_r_file_path = string(argv[6]);
		mask_image_l_file_path = string(argv[7]);
		mask_image_r_file_path = string(argv[8]);
		write_video_l_file_path = string(argv[9]);
		write_video_r_file_path = string(argv[10]);
	}

    int frame_count = 0;


    CANHANDLE canhandle = -1;

    MultiTypeUnion distance_l;
    CANMsg canmsg0x123;
    canmsg0x123.id = 0x123;
    canmsg0x123.len = 8;
    canmsg0x123.flags = 0;

    MultiTypeUnion distance_r;
    CANMsg canmsg0x124;
    canmsg0x124.id = 0x124;
    canmsg0x124.len = 8;
    canmsg0x124.flags = 0;

    MultiTypeUnion frame_l;
    CANMsg canmsg0x125;
    canmsg0x125.id = 0x125;
    canmsg0x125.len = 8;
    canmsg0x125.flags = 0;

    MultiTypeUnion frame_r;
    CANMsg canmsg0x126;
    canmsg0x126.id = 0x126;
    canmsg0x126.len = 8;
    canmsg0x126.flags = 0;

	char *szAdapter = NULL;
	cout << "Find the \"" << canusb_getFirstAdapter(szAdapter, 32) << "\" CANUSB devices." << endl;
    if (!((canhandle = canusb_Open(NULL, "500", CANUSB_ACCEPTANCE_CODE_ALL, CANUSB_ACCEPTANCE_MASK_ALL, CANUSB_FLAG_TIMESTAMP || CANUSB_FLAG_QUEUE_REPLACE)) > 0)) {
        cout << "Can't initialize the CANUSB device. Please rerun this program or recoonect the CANUSB devicel." << endl;
        Sleep(1000);
    }
    if (canhandle <= 0) { return -1; }


    LaneDistanceDetector ldd1(filter_param_l_file_path, in_external_paraml_l_file_path, mask_image_l_file_path, camera_num1, write_video_l_file_path);
    LaneDistanceDetector ldd2(filter_param_r_file_path, in_external_paraml_r_file_path, mask_image_r_file_path, camera_num2, write_video_r_file_path);


    while(1) {
        if (ldd1.ReadFrame()) { break; }
        if (ldd2.ReadFrame()) { break; }
        distance_l.float32[0] = ldd1.ProccessImage();
        distance_r.float32[0] = ldd2.ProccessImage();
        frame_l.float32[0] = frame_count;
        frame_r.uint32[0] = stol(t.str());
        cout << setfill('0') << setw(8) << frame_r.uint32[0] << " ";
        cout << setfill('0') << setw(6) << frame_l.float32[0] << " ";
        cout << setfill('0') << setw(5) << distance_l.float32[0] << " ";
        cout << setfill('0') << setw(5) << distance_r.float32[0] << " ";
        cout << endl;

        // Write CAN message
        if (canhandle > 0) {
            for (int i = 0; i < 8; ++i) {
                canmsg0x123.data[i] = distance_l.uint8[i];
                canmsg0x124.data[i] = distance_r.uint8[i];
                canmsg0x125.data[i] = frame_l.uint8[i];
                canmsg0x126.data[i] = frame_r.uint8[i];
            }
            canusb_Write(canhandle, &canmsg0x123);
            canusb_Write(canhandle, &canmsg0x124);
            canusb_Write(canhandle, &canmsg0x125);
            canusb_Write(canhandle, &canmsg0x126);
        }

        ldd1.ViewResizeImage(1, 0, 0, 0, 0, 0, 1);
        ldd2.ViewResizeImage(1, 0, 0, 0, 0, 0, 1);
        if (cv::waitKey(1) == 27) { break; }
        ++frame_count;
    }

    if (canhandle > 0) { canusb_Close(canhandle); }
    return 0;
}