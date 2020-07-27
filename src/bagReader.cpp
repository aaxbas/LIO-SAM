/**
 *  bagReader.cpp
 *  Read data from rosbag
 * 
 * TODO: Add relevant data for integration with other LIO-SAM packages
 * TODO: Improve this to be callable to other functions
 **/

#include <ros/ros.h>
#include <rosbag/view.h>

#include <gtest/gtest.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/Imu.h>

#define foreach BOOST_FOREACH

int main (int argc, char** argv)
{
    ros::init (argc, argv, "bag_it");
    rosbag::Bag bag;

    bag.open("src/LIO-SAM/datasets/casual_walk.bag", rosbag::bagmode::Read);

    std::vector<std::string> topics;

    // Read raw IMU data from bag
    topics.push_back(std::string("/imu_raw"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    foreach(rosbag::MessageInstance const m, view)
    {   
        // print raw IMU data
        sensor_msgs::Imu::ConstPtr s = m.instantiate<sensor_msgs::Imu>();
        if (s != NULL)
            std::cout << s->angular_velocity  << std::endl;
            std::cout << s->orientation  << std::endl;
            std::cout << s->linear_acceleration  << std::endl << std::endl;
    }

    bag.close();
    return (0);
}