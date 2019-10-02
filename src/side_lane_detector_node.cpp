#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>

#include "ads_msgs/canmsg.h"
#include "ads_msgs/float64.h"
#include "ads_msgs/deviation.h"
#include "can/message_can_wrapper.hpp"
#include "can/dbc.hpp"
#include "can/multi_type_union.hpp"

#include <iostream>
#include <iomanip>
#include <sstream>
#include "side_lane_distance.hpp"

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "side_lane_detector_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    MultiTypeUnion side_lane_distance_l;
    MultiTypeUnion side_lane_distance_r;
    MultiTypeUnion side_lane_frame_l;
    MultiTypeUnion side_lane_frame_r;
    PublisherWrapper side_lane_distance_l_publisher  = PublisherWrapper("side_lane_distance_l", "distance", "/home/nvidia/Documents/shibahata/data/ros/can/DPX_MAB_CAN.DBC");
    PublisherWrapper side_lane_distance_r_publisher  = PublisherWrapper("side_lane_distance_r", "distance", "/home/nvidia/Documents/shibahata/data/ros/can/DPX_MAB_CAN.DBC");
    PublisherWrapper side_lane_frame_l_publisher     = PublisherWrapper("side_lane_frame_l"   , "frame"   , "/home/nvidia/Documents/shibahata/data/ros/can/DPX_MAB_CAN.DBC");
    PublisherWrapper side_lane_frame_r_publisher     = PublisherWrapper("side_lane_frame_r"   , "frame"   , "/home/nvidia/Documents/shibahata/data/ros/can/DPX_MAB_CAN.DBC");



    //現在日時を取得する
    time_t time_raw = time(nullptr);

    //形式を変換する
    tm *date_raw;
    date_raw = localtime(&time_raw);

    //sに独自フォーマットになるように連結していく
    std::stringstream s, t;
    s << "20";
    s << setfill('0') << setw(2) << date_raw->tm_year - 100; //100を引くことで20xxのxxの部分になる
    s << setfill('0') << setw(2) << date_raw->tm_mon + 1; //月を0からカウントしているため
    s << setfill('0') << setw(2) << date_raw->tm_mday; //そのまま
    s << setfill('0') << setw(2) << date_raw->tm_hour; //そのまま
    s << setfill('0') << setw(2) << date_raw->tm_min; //そのまま
    s << setfill('0') << setw(2) << date_raw->tm_sec; //そのまま
    t << setfill('0') << setw(2) << date_raw->tm_mday; //そのまま
    t << setfill('0') << setw(2) << date_raw->tm_hour; //そのまま
    t << setfill('0') << setw(2) << date_raw->tm_min; //そのまま
    t << setfill('0') << setw(2) << date_raw->tm_sec; //そのまま
    std::string video_file_name1 = s.str() + "_1.avi";
    std::string video_file_name2 = s.str() + "_2.avi";

    int frame_count = 0;


    LaneDistanceDetector ldd1("/home/nvidia/catkin_ws/src/side_lane_detector/sample/filter_param.xml", "/home/nvidia/catkin_ws/src/side_lane_detector/sample/in_external_param.xml", "/home/nvidia/catkin_ws/src/side_lane_detector/sample/mask_image.bmp", 0);
    LaneDistanceDetector ldd2("/home/nvidia/catkin_ws/src/side_lane_detector/sample/filter_param.xml", "/home/nvidia/catkin_ws/src/side_lane_detector/sample/in_external_param.xml", "/home/nvidia/catkin_ws/src/side_lane_detector/sample/mask_image.bmp", 1);



    while (ros::ok()) {
        if (ldd1.ReadFrame()) { break; }
        if (ldd2.ReadFrame()) { break; }

        side_lane_distance_l.f[0] = ldd1.ProccessImage();
        side_lane_distance_r.f[0] = ldd2.ProccessImage();
        side_lane_frame_l.f[0] = frame_count;
        side_lane_frame_r.ui[0] = stol(t.str());

        side_lane_distance_l_publisher.publish(side_lane_distance_l.ul);
        side_lane_distance_r_publisher.publish(side_lane_distance_r.ul);
        side_lane_frame_l_publisher.publish(side_lane_frame_l.ul);
        side_lane_frame_r_publisher.publish(side_lane_frame_r.ul);

        cout << t.str() << " " << side_lane_frame_l.f[0] << " " << side_lane_distance_l.f[0] << endl;
        cout << side_lane_frame_r.ui[0] << " " << side_lane_frame_l.f[0] << " " << side_lane_distance_r.f[0] << endl;
        ldd1.ViewImage(1, 0, 0, 0, 0, 0, 1);
        ldd2.ViewImage(1, 0, 0, 0, 0, 0, 1);
        if (cv::waitKey(1) == 27) { break; }

        ++frame_count;
        loop_rate.sleep();
    }
    return 0;
}