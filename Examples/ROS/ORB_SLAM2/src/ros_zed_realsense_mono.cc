/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// include for publishing ROS odometry
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
// include for subscribing ROS IMU topic
#include "sensor_msgs/Imu.h"
#include <mutex>

#include <opencv2/core/core.hpp>

#include "../../../include/System.h"

// include for IMU processing
#include <eigen3/Eigen/Dense>
#include "../../../src/IMU/imudata.h"
#include "../../../src/IMU/configparam.h"

#include <sstream>
#include </opt/ros/melodic/include/std_msgs/String.h>


using namespace std;
// using namespace Eigen;
#define PI 3.141592653589793

class ImageGrabber {
  public:
    ImageGrabber(ORB_SLAM2::System *pSLAM): mpSLAM(pSLAM) {}
    void GrabImage(const sensor_msgs::ImageConstPtr &msg);
    void Publish(const cv::Mat Tcw);
   
    ORB_SLAM2::System *mpSLAM;
    bool do_rectify;
    cv::Mat M1l, M2l, M1r, M2r;

    ros::NodeHandle nh;
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("orb_odom", 10);
    tf::TransformBroadcaster odom_broadcaster;
    tf::Transform transform;
    ros::Time current_time = ros::Time::now();
    ros::Time last_time = ros::Time::now();
    ros::Time imu_last_time;
    float yaw_angle_accums = 0;
    std::mutex mMutexYawAngle;

    double x, y, z;
    double dx, dy, dz;
    // string order;

    ros::Publisher pub = nh.advertise<std_msgs::String>("/dreamdeck", 1000);
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "ZED");
    ros::start();

    if (argc != 5) {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Stereo path_to_vocabulary path_to_settings do_rectify do_relocation" << endl;
        ros::shutdown();
        return 1;
    }

    stringstream ss_view(argv[3]);
    bool do_view;
    ss_view >> boolalpha >> do_view;
    
    stringstream ss_relocation(argv[4]);
    bool do_relocation;
    ss_relocation >> boolalpha >> do_relocation;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.

    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR, do_view, do_relocation);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/camera/color/image_raw", 1, &ImageGrabber::GrabImage,&igb);
    ros::spin();
    // Stop all threads
    SLAM.Shutdown();
    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr &msg) {
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(msg);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat Tcw;

    Tcw = mpSLAM->TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec());
    // cout << "Current timestemp = " << setprecision(18) << cv_ptr->header.stamp.toSec() << ", position : " << Tcw << endl;
    // cout << "Tcw.at<float>(1,2) = " << Tcw.at<float>(1,2) << endl;
    if (!Tcw.empty()) {
        Publish(Tcw);
    }

}

void ImageGrabber::Publish(const cv::Mat Tcw) 
{
    // Now start publishing odometry topic into ROS
    cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
    cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);
    
    float sy = sqrt(Rwc.at<float>(0,0)*Rwc.at<float>(0,0) + Rwc.at<float>(1,0)*Rwc.at<float>(1,0));
    
    float thetax, thetay, thetaz, thetay1, thetay2;
    thetax = 0;
    thetaz = 0;
    thetay1 = atan2(-Rwc.at<float>(2,0), sy) / PI * 180;
    thetay2 = atan2(Rwc.at<float>(0,2), Rwc.at<float>(2,2)) / PI * 180;
    if (abs(thetay1)>=abs(thetay2))
        thetay = thetay1;
    else
        thetay = thetay2;


    std_msgs::String msg;
    ostringstream ss;

    ss << "pos:" << twc.at<float>(0, 2) << "," << twc.at<float>(0, 0) << "," << -twc.at<float>(0, 1) << "," 
            << thetaz << "," << -thetax << "," << thetay << endl;
    msg.data = ss.str();
    pub.publish(msg);
}