/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<vector>
#include<queue>
#include<thread>
#include<mutex>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>


#include<opencv2/core/core.hpp>
#include<opencv2/imgproc/imgproc.hpp>

#include"../../../include/System.h"


using namespace std;

bool bRGB = true;

cv::Mat K;
cv::Mat DistCoef;


class ImageGrabber {
  public:
    //ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){}
    ImageGrabber(ORB_SLAM2::System *pSLAM, const bool bRect, const bool bClahe) : mpSLAM(pSLAM), do_rectify(bRect), mbClahe(bClahe) {}
    void GrabImageLeft(const sensor_msgs::ImageConstPtr &msg);
    void GrabImageRight(const sensor_msgs::ImageConstPtr &msg);
    // void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);
    void SyncWithImu();


    queue<sensor_msgs::ImageConstPtr> imgLeftBuf, imgRightBuf;
    std::mutex mBufMutexLeft, mBufMutexRight;

    ORB_SLAM2::System *mpSLAM;

    const bool do_rectify;
    cv::Mat M1l, M2l, M1r, M2r;

    const bool mbClahe;
    cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "RGBD");
    ros::start();

    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    bool bEqual = false;

    /*if (argc < 4 || argc > 5)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Stereo_Inertial path_to_vocabulary path_to_settings do_rectify [do_equalize]" << endl;
        ros::shutdown();
        return 1;
    }*/

    std::string sbRect(argv[3]);
    if (argc == 5) {
        std::string sbEqual(argv[4]);
        if (sbEqual == "true")
            bEqual = true;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.

    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::STEREO, true);
    /*char DoLoadMap;
    cout << "Do you want to load map?(y/n)" << endl;
    cin >> DoLoadMap;
    if (DoLoadMap == 'Y' || DoLoadMap == 'y')
        SLAM.LoadMap("Slam_latest_Map.bin");*/



    // ImageGrabber igb(&SLAM);

    ImageGrabber igb(&SLAM, sbRect == "true", bEqual);

    //ros::NodeHandle nodeHandler;

    ros::Subscriber sub_img_left = n.subscribe("/camera/infra1/image_rect_raw", 1, &ImageGrabber::GrabImageLeft, &igb);
    ros::Subscriber sub_img_right = n.subscribe("/camera/infra2/image_rect_raw", 1, &ImageGrabber::GrabImageRight, &igb);


    cv::FileStorage fSettings(argv[2], cv::FileStorage::READ);
    bRGB = static_cast<bool>((int)fSettings["Camera.RGB"]);

    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];


    K = cv::Mat::eye(3, 3, CV_32F);
    K.at<float>(0, 0) = fx;
    K.at<float>(1, 1) = fy;
    K.at<float>(0, 2) = cx;
    K.at<float>(1, 2) = cy;

    DistCoef = cv::Mat::zeros(4, 1, CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if (k3 != 0) {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    thread syns_thread(&ImageGrabber::SyncWithImu, &igb);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();
    // Save map
    // Save camera trajectory
    // SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    // Save customized Map
    char IsSaveMap;
    cout << "Do you want to save the map?(y/n)" << endl;
    cin >> IsSaveMap;
    if (IsSaveMap == 'Y' || IsSaveMap == 'y')
        SLAM.SaveMap("MapPointandKeyFrame.bin");

    ros::shutdown();

    return 0;
}


void ImageGrabber::GrabImageLeft(const sensor_msgs::ImageConstPtr &img_msg) {
    mBufMutexLeft.lock();
    if (!imgLeftBuf.empty())
        imgLeftBuf.pop();
    imgLeftBuf.push(img_msg);
    mBufMutexLeft.unlock();
}

void ImageGrabber::GrabImageRight(const sensor_msgs::ImageConstPtr &img_msg) {
    mBufMutexRight.lock();
    if (!imgRightBuf.empty())
        imgRightBuf.pop();
    imgRightBuf.push(img_msg);
    mBufMutexRight.unlock();
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::ImageConstPtr &img_msg) {
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    if (cv_ptr->image.type() == 0) {
        return cv_ptr->image.clone();
    } else {
        std::cout << "Error type" << std::endl;
        return cv_ptr->image.clone();
    }
}

void ImageGrabber::SyncWithImu() {
    const double maxTimeDiff = 0.01;
    cv_bridge::CvImageConstPtr cv_ptr;
    while (1) {
        cv::Mat imLeft, imRight, Tcw;
        double tImLeft = 0, tImRight = 0;
        if (!imgLeftBuf.empty() && !imgRightBuf.empty()) {
            tImLeft = imgLeftBuf.front()->header.stamp.toSec();
            tImRight = imgRightBuf.front()->header.stamp.toSec();

            this->mBufMutexRight.lock();
            while ((tImLeft - tImRight) > maxTimeDiff && imgRightBuf.size() > 1) {
                imgRightBuf.pop();
                tImRight = imgRightBuf.front()->header.stamp.toSec();
            }
            this->mBufMutexRight.unlock();

            this->mBufMutexLeft.lock();
            while ((tImRight - tImLeft) > maxTimeDiff && imgLeftBuf.size() > 1) {
                imgLeftBuf.pop();
                tImLeft = imgLeftBuf.front()->header.stamp.toSec();
            }
            this->mBufMutexLeft.unlock();

            if ((tImLeft - tImRight) > maxTimeDiff || (tImRight - tImLeft) > maxTimeDiff) {
                // std::cout << "big time difference" << std::endl;
                continue;
            }


            this->mBufMutexLeft.lock();
            cv_ptr = cv_bridge::toCvShare(imgLeftBuf.front(), sensor_msgs::image_encodings::MONO8);
            imLeft = cv_ptr->image.clone();
            // imLeft = GetImage(imgLeftBuf.front());
            imgLeftBuf.pop();
            this->mBufMutexLeft.unlock();

            this->mBufMutexRight.lock();
            cv_ptr = cv_bridge::toCvShare(imgRightBuf.front(), sensor_msgs::image_encodings::MONO8);
            imRight = cv_ptr->image.clone();
            // imRight = GetImage(imgRightBuf.front());
            imgRightBuf.pop();
            this->mBufMutexRight.unlock();


            if (mbClahe) {
                mClahe->apply(imLeft, imLeft);
                mClahe->apply(imRight, imRight);
            }

            if (do_rectify) {
                cv::remap(imLeft, imLeft, M1l, M2l, cv::INTER_LINEAR);
                cv::remap(imRight, imRight, M1r, M2r, cv::INTER_LINEAR);
            }

            //cv::Mat imu;
            cout << endl << "ros_stereo.cc  ImageGrabber::SyncWithImu() --> System::TrackStereo() //starting ................................" << endl;
            Tcw = mpSLAM->TrackStereo(imLeft, imRight, cv_ptr->header.stamp.toSec());


        }
    }
}
