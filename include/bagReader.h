/**
 *  bagReader.h
 *  Read data from rosbag using rosbag API
 *  Author: Ahmed Abbas
 **/


#pragma once
#ifndef _BAGREADER_H_
#define _BAGREADER_H_

#include <ros/ros.h>
#include <rosbag/view.h>

#include <gtest/gtest.h>

#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

/**
 * Custom rosbag reader class to implement lazy-loading easily
 */
class bagReader {

public:
    rosbag::Bag bag;
    std::vector<std::string> topics{std::string("/imu_raw"),std::string("/imu_correct"),std::string("/points_raw")};
    rosbag::View view;
    
    bagReader(){};
    
    /**
     *  Load bag from filename 
     * 
     *  @param filename the name of the rosbag file to read
     */
    void load(const std::string &filename)
    {
        bag.open(filename, rosbag::bagmode::Read);
        view.addQuery(bag, rosbag::TopicQuery(topics));
    }
     
     /**
      * Read message at MessageInstance m and write to v
      * 
      * TODO: Implement this 
      * 
      *  @param m[in] MessageInstance to read from
      *  @param imu_raw[out] Raw imu data to read from bag file
      *  @param imu_correct[out] Correct imu data to read from bag file
      *  @param points_raw[out] Raw points to read from bag file
      */
     void read(rosbag::MessageInstance m, sensor_msgs::Imu::ConstPtr imu_raw, sensor_msgs::Imu::ConstPtr imu_correct, sensor_msgs::PointCloud2ConstPtr points_raw){
         /*  */
            if(m.getTopic() == "/imu_raw"){
                sensor_msgs::Imu::ConstPtr s = m.instantiate<sensor_msgs::Imu>();
                if (s != NULL){
                    imu_raw = s;
                }
            }

            if(m.getTopic() == "/imu_correct"){
                sensor_msgs::Imu::ConstPtr s = m.instantiate<sensor_msgs::Imu>();
                if (s != NULL){
                    imu_correct = s;
                }
            }

            if(m.getTopic() == "/points_raw"){
                sensor_msgs::PointCloud2::ConstPtr s = m.instantiate<sensor_msgs::PointCloud2>();
                if (s != NULL){
                    points_raw = s;
                }
            }

    }

    ~bagReader(){
        /* Clean-up */
        bag.close();
    }

};

#endif