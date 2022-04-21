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
#include <ctime>


using namespace std;
// using namespace Eigen;
#define PI 3.141592653589793

class ImageGrabber {
  public:
    ImageGrabber(ORB_SLAM2::System *pSLAM): mpSLAM(pSLAM) {}

    void GrabStereo(const sensor_msgs::ImageConstPtr &msgLeft, const sensor_msgs::ImageConstPtr &msgRight);
    void GrabImage(const sensor_msgs::ImageConstPtr &msg);
    void PublishOdomAndTf(const cv::Mat Tcw);
    void Publish(const cv::Mat Tcw);
    void GrabIMU(const sensor_msgs::Imu::ConstPtr &msg);
    void GrabStereoWithIMU(const sensor_msgs::ImageConstPtr &msgLeft, const sensor_msgs::ImageConstPtr &msgRight);
    // for two stereos
    void GrabTwoStereoWithIMU(const sensor_msgs::ImageConstPtr &msgLeft, const sensor_msgs::ImageConstPtr &msgRight, const sensor_msgs::ImageConstPtr &msgBackLeft, const sensor_msgs::ImageConstPtr &msgBackRight);
    // void ProcessIMU(double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity);
    

    ORB_SLAM2::System *mpSLAM;
    // bool do_rectify;
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
    // // For IMU integration:
    // Vector3d acc_0, gyr_0;
    // Vector3d Bas[(WINDOW_SIZE + 1)];
    // Vector3d Bgs[(WINDOW_SIZE + 1)];
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "ZED");
    ros::start();

    // if (argc != 5) {
    //     cerr << endl << "Usage: rosrun ORB_SLAM2 Stereo path_to_vocabulary path_to_settings do_rectify do_relocation" << endl;
    //     ros::shutdown();
    //     return 1;
    // }

    bool UseIMU = true;
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    bool UseViewer =true;
    
    stringstream ss_view(argv[3]);
    bool do_view;
    ss_view >> boolalpha >> do_view;

    stringstream ss_loadmap(argv[4]);
    bool do_loadmap;
    ss_loadmap >> boolalpha >> do_loadmap;

    stringstream ss_relocation(argv[5]);
    bool do_relocation;
    ss_relocation >> boolalpha >> do_relocation;

    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::TWOSTEREO, do_view, do_loadmap, do_relocation);
    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;
    ros::Subscriber sub_imu;

    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/camera_front/infra1/image_rect_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/camera_front/infra2/image_rect_raw", 1);
    // for two stereos
    message_filters::Subscriber<sensor_msgs::Image> back_left_sub(nh, "/camera_back/infra1/image_rect_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> back_right_sub(nh, "/camera_back/infra2/image_rect_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub, right_sub, back_left_sub, back_right_sub);
    // sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));

    // for two stereos
    if (UseIMU) {
        sub_imu = nh.subscribe("/camera_front/imu", 1, &ImageGrabber::GrabIMU, &igb);
        sync.registerCallback(boost::bind(&ImageGrabber::GrabTwoStereoWithIMU, &igb, _1, _2, _3, _4));
        // SLAM.SetViewerIMUFlagTrue();
    } else {
        sync.registerCallback(boost::bind(&ImageGrabber::GrabTwoStereoWithIMU, &igb, _1, _2, _3, _4));
        // SLAM.SetViewerIMUFlagFalse();
    }

    ros::spin();
    // Stop all threads
    SLAM.Shutdown();

    // Save customized Map
    // char IsSaveMap;  
    // cout << "Do you want to save the map?(y/n)" << endl;  
    // cin >> IsSaveMap;  
    // if(IsSaveMap == 'Y' || IsSaveMap == 'y')  
    //     SLAM.SaveMap("/home/dreamdeck/Documents/ORB_SLAM2/MapPointandKeyFrame.bin");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr &msgLeft, const sensor_msgs::ImageConstPtr &msgRight) {
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat Tcw;   
    Tcw = mpSLAM->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image, cv_ptrLeft->header.stamp.toSec());
    mpSLAM->SetSaveImageFlag();// This is just telling the viewer that it can save an image. Not actually saving one. The finall decision would be made my the viewer.
    
    //cout << "Current timestemp = " << setprecision(18) << cv_ptrLeft->header.stamp.toSec() << ", position : " << Tcw << endl;
    if (!Tcw.empty()) {
        Publish(Tcw);
    }

}

void ImageGrabber::GrabStereoWithIMU(const sensor_msgs::ImageConstPtr &msgLeft, const sensor_msgs::ImageConstPtr &msgRight) {
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat Tcw;

    //get current yaw_angle_accums
    float current_yaw_angle_accums;
    {
        unique_lock<mutex> lock(mMutexYawAngle);
        current_yaw_angle_accums = yaw_angle_accums;
        yaw_angle_accums = 0;
    }

    Tcw = mpSLAM->TrackStereoWithIMU(cv_ptrLeft->image, cv_ptrRight->image, cv_ptrLeft->header.stamp.toSec(), current_yaw_angle_accums);
    mpSLAM->SetSaveImageFlag();// This is just telling the viewer that it can save an image. Not actually saving one. The finall decision would be made my the viewer.
    
    // cout << "Current timestemp = " << setprecision(18) << cv_ptrLeft->header.stamp.toSec() << ", position : " << Tcw << endl;
    if (!Tcw.empty()) {
        Publish(Tcw);
    }

}

// for two stereos
void ImageGrabber::GrabTwoStereoWithIMU(const sensor_msgs::ImageConstPtr &msgLeft, const sensor_msgs::ImageConstPtr &msgRight, const sensor_msgs::ImageConstPtr &msgBackLeft, const sensor_msgs::ImageConstPtr &msgBackRight)
{
    clock_t startTime = clock();

    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrBackLeft;
    try
    {
        cv_ptrBackLeft = cv_bridge::toCvShare(msgBackLeft);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrBackRight;
    try
    {
        cv_ptrBackRight = cv_bridge::toCvShare(msgBackRight);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat Tcw;

    //get current yaw_angle_accums
    float current_yaw_angle_accums;
    {
        unique_lock<mutex> lock(mMutexYawAngle);
        current_yaw_angle_accums = yaw_angle_accums;
        yaw_angle_accums = 0;
    }

    // Tcw = mpSLAM->TrackStereoWithIMU(cv_ptrLeft->image, cv_ptrRight->image, cv_ptrLeft->header.stamp.toSec(), current_yaw_angle_accums);
    Tcw = mpSLAM->TrackTwoStereoWithIMU(cv_ptrLeft->image, cv_ptrRight->image, cv_ptrBackLeft->image, cv_ptrBackRight->image, cv_ptrLeft->header.stamp.toSec(), current_yaw_angle_accums);
    mpSLAM->SetSaveImageFlag(); // This is just telling the viewer that it can save an image. Not actually saving one. The finall decision would be made my the viewer.

    if (!Tcw.empty())
    {
        Publish(Tcw);
        // for (int i = 0; i < 4; i++)
        // {
        //     for (int j = 0; j < 4; j++)
        //         cout << fixed << setprecision(2) << Tcw.at<float>(i, j) << "\t";
        //     cout << endl;
        // }
        clock_t endTime = clock();
        cout << "fps is: " << 1 / (double(endTime - startTime) / CLOCKS_PER_SEC) << endl;
    }
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
        PublishOdomAndTf(Tcw);
    }

}

void ImageGrabber::GrabIMU(const sensor_msgs::Imu::ConstPtr &msg) {
    unique_lock<mutex> lock(mMutexYawAngle);
    if (!imu_last_time.toSec()) { // Initialize imu_last_time
        imu_last_time = msg->header.stamp;
    }
    // ROS_INFO("Imu heard: seq = [%i], time = [%i, %i], frame_id = [%s]", msg->header.seq, msg->header.stamp.sec, msg->header.stamp.nsec, msg->header.frame_id.c_str());
    // ROS_INFO("Imu Orientation x: [%f], y: [%f], z: [%f], w: [%f]", msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
    // ROS_INFO("Imu angular_velocity x: [%f], y: [%f], z: [%f]", msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);


    ros::Duration imu_duration = msg->header.stamp - imu_last_time;
    // TO DO: subtract bias : - ba[xxx];  - bg[xxx];

    float dx = msg->linear_acceleration.x; // X acceleration
    float dy = msg->linear_acceleration.y; // Y acceleration
    float dz = msg->linear_acceleration.z; // Z acceleration

    float rx = msg->angular_velocity.x; // roll_rate
    float ry = msg->angular_velocity.y; // pitch_rate
    float rz = msg->angular_velocity.z; // yaw_rate

    float dt = imu_duration.toSec();
    float yaw_angle = dt * rz;
    yaw_angle_accums = yaw_angle_accums + yaw_angle;
    // ROS_INFO("Imu timestamp duration is : [%6.6f]", imu_duration.toSec());
    // ROS_INFO("In my calculation, rz = : [%f], dt = : [%f], yaw_angle = : [%f].", rz, dt, yaw_angle);
    // ROS_INFO("yaw_angle_accums = : [%f].", yaw_angle_accums);

    imu_last_time = msg->header.stamp;

    // ProcessIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
}

void ImageGrabber::PublishOdomAndTf(const cv::Mat Tcw) {

    // Now start publishing odometry topic into ROS
    current_time = ros::Time::now();

    cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
    cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);

    vector<float> quat = ORB_SLAM2::Converter::toQuaternion(Rwc);

    const float MAP_SCALE = 1.0f;

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(twc.at<float>(0, 0) * MAP_SCALE, twc.at<float>(0, 1) * MAP_SCALE, twc.at<float>(0, 2) * MAP_SCALE));
    tf::Quaternion quaternion(quat[0], quat[1], quat[2], quat[3]); // x,y,z,w
    transform.setRotation(quaternion);

    odom_broadcaster.sendTransform(tf::StampedTransform(transform, current_time, "orb_odom", "orb_base_link"));

    // // double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    // // double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    // // double delta_th = vth * dt;

    // // x += delta_x;
    // // y += delta_y;
    // // th += delta_th;


    // //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat;
    odom_quat.x = quat[0];
    odom_quat.y = quat[1];
    odom_quat.z = quat[2];
    odom_quat.w = quat[3];

    // //first, we'll publish the transform over tf
    // geometry_msgs::TransformStamped odom_trans;
    // odom_trans.header.stamp = current_time;
    // odom_trans.header.frame_id = "orb_odom";
    // odom_trans.child_frame_id = "orb_base_link";

    // odom_trans.transform.translation.x = -z;
    // odom_trans.transform.translation.y = -x;
    // odom_trans.transform.translation.z = -y;
    // odom_trans.transform.rotation = odom_quat;

    // //send the transform
    // odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "orb_odom";

    //set the position
    odom.pose.pose.position.x = twc.at<float>(0, 0) * MAP_SCALE;
    odom.pose.pose.position.y = twc.at<float>(0, 1) * MAP_SCALE;
    odom.pose.pose.position.z = twc.at<float>(0, 2) * MAP_SCALE;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "orb_base_link";
    // odom.twist.twist.linear.x = vx;
    // odom.twist.twist.linear.y = vy;
    // odom.twist.twist.angular.z = vth;

    //publish the message
    cout << "ImageGrabber::PublishOdomAndTf() --> publish()" << endl;
    odom_pub.publish(odom);

    last_time = current_time;
}

void ImageGrabber::Publish(const cv::Mat Tcw) {

    // Now start publishing odometry topic into ROS
    cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
    cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);
    // vector<float> quat = ORB_SLAM2::Converter::toQuaternion(Rwc);
    
    float sy = sqrt(Rwc.at<float>(0,0)*Rwc.at<float>(0,0) + Rwc.at<float>(1,0)*Rwc.at<float>(1,0));
    
    // bool singular = sy < 0.5;
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