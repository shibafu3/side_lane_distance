#include <iostream>
#include <iomanip>
#include <sstream>

#include "side_lane_distance.hpp"


using namespace std;
using namespace cv;

int main(char argc, char *argv[]) {
    //現在日時を取得する
    time_t time_raw = time(nullptr);

    //形式を変換する
    tm *date_raw;
    date_raw = localtime(&time_raw);

    //sに独自フォーマットになるように連結していく
    std::stringstream s;
    s << "20";
    s << setfill('0') << setw(2) << date_raw->tm_year - 100; //100を引くことで20xxのxxの部分になる
    s << setfill('0') << setw(2) << date_raw->tm_mon + 1; //月を0からカウントしているため
    s << setfill('0') << setw(2) << date_raw->tm_mday; //そのまま
    s << setfill('0') << setw(2) << date_raw->tm_hour; //そのまま
    s << setfill('0') << setw(2) << date_raw->tm_min; //そのまま
    s << setfill('0') << setw(2) << date_raw->tm_sec; //そのまま
    std::string video_file_name1 = s.str() + "_1.avi";
    std::string video_file_name2 = s.str() + "_2.avi";

    int frame_count = 0;


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

        if (waitKey(1) == 27) { break; }
        ++frame_count;
    }
    return 0;
}