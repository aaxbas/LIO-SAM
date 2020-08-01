#include "utility.h"
#include "bagReader.h"
#include "featureExtraction.h"
#include "imageProjection.h"
#include "imuPreintegration.h"
#include "mapOptmization.h"

#include <ros/ros.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH


int main(int argc, char** argv){

    // Initialize ROS
    ros::init(argc, argv, "main_loam");

    // Create node and set loop rate
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    
    // Publish correct imu data at /imu_correct
    ros::Publisher imu_correct_pub = n.advertise<sensor_msgs::Imu>("/imu_correct", 1000);

    // LIO-SAM class definitions
    FeatureExtraction FE;
    mapOptimization MO;
    ImageProjection IP;
    TransformFusion TF;
    IMUPreintegration ImuP;
    
    // Load bag file
    bagReader bgr;
    bgr.load(std::string(argv[1]));
    
    // Read each message in bag file
    foreach(rosbag::MessageInstance m, bgr.view){

        if(m.getTopic() == "/imu_raw"){
                sensor_msgs::Imu::ConstPtr s = m.instantiate<sensor_msgs::Imu>();
                if (s != NULL){
                    // Handle raw IMU values using LIO-SAM class functions
                    ImuP.imuHandler(s);
                    IP.imuHandler(s);
                }
            }

        if(m.getTopic() == "/imu_correct"){
            sensor_msgs::Imu::ConstPtr s = m.instantiate<sensor_msgs::Imu>();
            if (s != NULL){
                // Publish correct imu data
                imu_correct_pub.publish(m.instantiate<sensor_msgs::Imu>());
            }
        }

        if(m.getTopic() == "/points_raw"){
            sensor_msgs::PointCloud2ConstPtr s = m.instantiate<sensor_msgs::PointCloud2>();
            if (s != NULL){
                // Handle point cloud data using LIO-SAM class function
                IP.cloudHandler(s);
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    
}