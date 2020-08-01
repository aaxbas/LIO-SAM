#include "bagReader.h"

#include <ros/ros.h>
#include <rosbag/view.h>

#include <gtest/gtest.h>

#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH

int main(int argc, char** argv){
    ros::init(argc, argv, "bag_reader");

    ros::NodeHandle n;

    ros::Publisher pubImuRaw = n.advertise<sensor_msgs::Imu>("/imu_raw", 1000);
    ros::Publisher pubImuCorrect = n.advertise<sensor_msgs::Imu>("/imu_correct", 1000);
    ros::Publisher pubPointsRaw = n.advertise<sensor_msgs::PointCloud2>("/points_raw", 1000);

    bagReader bgr;

    bgr.load(std::string("/home/osboxes/Documents/cust_space/src/LIO-SAM/datasets/casual_walk.bag"));

    foreach(rosbag::MessageInstance m, bgr.view){
        //bgr.read(m, imu_raw, imu_correct, points_raw);
        
        if(m.getTopic() == "/imu_raw"){
                sensor_msgs::Imu::ConstPtr s = m.instantiate<sensor_msgs::Imu>();
                if (s != NULL){
                    pubImuRaw.publish(*s);
                }
            }

            if(m.getTopic() == "/imu_correct"){
                sensor_msgs::Imu::ConstPtr s = m.instantiate<sensor_msgs::Imu>();
                if (s != NULL){
                    pubImuCorrect.publish(*s);
                }
            }

            if(m.getTopic() == "/points_raw"){
                sensor_msgs::PointCloud2::ConstPtr s = m.instantiate<sensor_msgs::PointCloud2>();
                if (s != NULL){
                    pubPointsRaw.publish(*s);
                }
            }

        ros::spinOnce();
    }
}