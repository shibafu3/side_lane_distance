#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>

#include "ads_msgs/canmsg.h"
#include "ads_msgs/float64.h"
#include "ads_msgs/deviation.h"
#include "can/message_can_wrapper.hpp"
#include "can/dbc.hpp"
#include "can/multi_type_union.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "side_lane_detector_node");
    ros::Rate loop_rate(1);

    MultiTypeUnion side_lane_distance_l;
    MultiTypeUnion side_lane_distance_r;
    MultiTypeUnion side_lane_frame_l;
    MultiTypeUnion side_lane_frame_r;
    PublisherWrapper side_lane_distance_l_publisher  = PublisherWrapper("side_lane_distance_l", "distance", "/home/nvidia/Documents/shibahata/data/ros/can/DPX_MAB_CAN.DBC");
    PublisherWrapper side_lane_distance_r_publisher  = PublisherWrapper("side_lane_distance_r", "distance", "/home/nvidia/Documents/shibahata/data/ros/can/DPX_MAB_CAN.DBC");
    PublisherWrapper side_lane_frame_l_publisher     = PublisherWrapper("side_lane_frame_l"   , "frame"   , "/home/nvidia/Documents/shibahata/data/ros/can/DPX_MAB_CAN.DBC");
    PublisherWrapper side_lane_frame_r_publisher     = PublisherWrapper("side_lane_frame_r"   , "frame"   , "/home/nvidia/Documents/shibahata/data/ros/can/DPX_MAB_CAN.DBC");

    side_lane_distance_l.f[0] = 0;

    while (ros::ok()) {
        side_lane_distance_l.f[0] = 0;
        side_lane_distance_r.f[0] = 0;
        side_lane_frame_l.f[0] = 0;
        side_lane_frame_r.f[0] = 0;
        side_lane_distance_l_publisher.publish(side_lane_distance_l);
        side_lane_distance_r_publisher.publish(side_lane_distance_r);
        side_lane_frame_l_publisher.publish(side_lane_frame_l);
        side_lane_frame_r_publisher.publish(side_lane_frame_r);

        loop_rate.sleep();
    }
    return 0;
}