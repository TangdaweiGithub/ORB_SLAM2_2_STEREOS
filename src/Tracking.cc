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


#include "Tracking.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"ORBmatcher.h"
#include"FrameDrawer.h"
#include"Converter.h"
#include"Map.h"
#include"Initializer.h"

#include"Optimizer.h"
#include"PnPsolver.h"

#include<iostream>

#include<mutex>


using namespace std;

namespace ORB_SLAM2
{

Tracking::Tracking(System *pSys, ORBVocabulary* pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Map *pMap, KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor):
    mState(NO_IMAGES_YET), mSensor(sensor), mbOnlyTracking(false), mbVO(false), mpORBVocabulary(pVoc),
    mpKeyFrameDB(pKFDB), mpInitializer(static_cast<Initializer*>(NULL)), mpSystem(pSys), mpViewer(NULL),
    mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpMap(pMap), mnLastRelocFrameId(0)
{
    // Load camera parameters from settings file

    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    float fps = fSettings["Camera.fps"];
    if(fps==0)
        fps=30;

    // Max/Min Frames to insert keyframes and to check relocalisation
    mMinFrames = 0;
    mMaxFrames = fps;

    cout << endl << "Camera Parameters: " << endl;
    cout << "- fx: " << fx << endl;
    cout << "- fy: " << fy << endl;
    cout << "- cx: " << cx << endl;
    cout << "- cy: " << cy << endl;
    cout << "- k1: " << DistCoef.at<float>(0) << endl;
    cout << "- k2: " << DistCoef.at<float>(1) << endl;
    if(DistCoef.rows==5)
        cout << "- k3: " << DistCoef.at<float>(4) << endl;
    cout << "- p1: " << DistCoef.at<float>(2) << endl;
    cout << "- p2: " << DistCoef.at<float>(3) << endl;
    cout << "- fps: " << fps << endl;


    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    // Load ORB parameters

    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(sensor==System::STEREO)
        mpORBextractorRight = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
    if (sensor == System::TWOSTEREO)
    {
        mpORBextractorRight = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);
        mpORBextractorBackLeft = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);
        mpORBextractorBackRight = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);
    }
    if(sensor==System::MONOCULAR)
        mpIniORBextractor = new ORBextractor(2*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    cout << endl  << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

    if(sensor==System::STEREO || sensor==System::RGBD)
    {
        mThDepth = mbf*(float)fSettings["ThDepth"]/fx;
        cout << endl << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
    }

    if(sensor==System::RGBD)
    {
        mDepthMapFactor = fSettings["DepthMapFactor"];
        if(fabs(mDepthMapFactor)<1e-5)
            mDepthMapFactor=1;
        else
            mDepthMapFactor = 1.0f/mDepthMapFactor;
    }

}

void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}
void Tracking::SetLocalMapperBack(LocalMappingBack *pLocalMapperBack)
{
    mpLocalMapperBack = pLocalMapperBack;
}

void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
{
    mpLoopClosing=pLoopClosing;
}

void Tracking::SetViewer(Viewer *pViewer)
{
    mpViewer=pViewer;
}

cv::Mat Tracking::GrabTwoImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const cv::Mat &imRectBackLeft, const cv::Mat &imRectBackRight, const double &timestamp)
{
    mImGray = imRectLeft;
    cv::Mat imGrayRight = imRectRight;
    cv::Mat imGrayBackLeft = imRectBackLeft;
    cv::Mat imGrayBackRight = imRectBackRight;
    mImGrayBack = imRectBackLeft;

    if (mImGray.channels() == 3)
    {
        if (mbRGB)
        {
            cvtColor(mImGray, mImGray, CV_RGB2GRAY);
            cvtColor(imGrayRight, imGrayRight, CV_RGB2GRAY);
            cvtColor(imGrayBackLeft, imGrayBackLeft, CV_RGB2GRAY);
            cvtColor(imGrayBackRight, imGrayBackRight, CV_RGB2GRAY);
        }
        else
        {
            cvtColor(mImGray, mImGray, CV_BGR2GRAY);
            cvtColor(imGrayRight, imGrayRight, CV_BGR2GRAY);
            cvtColor(imGrayBackLeft, imGrayBackLeft, CV_BGR2GRAY);
            cvtColor(imGrayBackRight, imGrayBackRight, CV_BGR2GRAY);
        }
    }
    else if (mImGray.channels() == 4)
    {
        if (mbRGB)
        {
            cvtColor(mImGray, mImGray, CV_RGBA2GRAY);
            cvtColor(imGrayRight, imGrayRight, CV_RGBA2GRAY);
            cvtColor(imGrayBackLeft, imGrayBackLeft, CV_RGBA2GRAY);
            cvtColor(imGrayBackRight, imGrayBackRight, CV_RGBA2GRAY);
        }
        else
        {
            cvtColor(mImGray, mImGray, CV_BGRA2GRAY);
            cvtColor(imGrayRight, imGrayRight, CV_BGRA2GRAY);
            cvtColor(imGrayBackLeft, imGrayBackLeft, CV_BGRA2GRAY);
            cvtColor(imGrayBackRight, imGrayBackRight, CV_BGRA2GRAY);
        }
    }

    mCurrentFrame = Frame(mImGray, imGrayRight, timestamp, mpORBextractorLeft, mpORBextractorRight, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth);
    mCurrentFrameBack = Frame(imGrayBackLeft, imGrayBackRight, timestamp, mpORBextractorBackLeft, mpORBextractorBackRight, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth);

    Track();

    return mCurrentFrame.mTcw.clone();
}

cv::Mat Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp)
{
    mImGray = imRectLeft;
    cv::Mat imGrayRight = imRectRight;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGB2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGR2GRAY);
        }
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGBA2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGRA2GRAY);
        }
    }

    mCurrentFrame = Frame(mImGray,imGrayRight,timestamp,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    Track();

    return mCurrentFrame.mTcw.clone();
}


cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp)
{
    mImGray = imRGB;
    cv::Mat imDepth = imD;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    if((fabs(mDepthMapFactor-1.0f)>1e-5) || imDepth.type()!=CV_32F)
        imDepth.convertTo(imDepth,CV_32F,mDepthMapFactor);

    mCurrentFrame = Frame(mImGray,imDepth,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    Track();

    return mCurrentFrame.mTcw.clone();
}


cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp)
{
    mImGray = im;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    if((mState==NOT_INITIALIZED || mState==NO_IMAGES_YET) && mpMap->GetMaxKFid() == 0){
        // cout << "Tracking::GrabImageMonocular : start from scratch." << endl;
        mCurrentFrame = Frame(mImGray,timestamp,mpIniORBextractor,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
    }
    else{
        // cout << "Tracking::GrabImageMonocular : Already initialized for monocular map." << endl;
        mCurrentFrame = Frame(mImGray,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
        // cout << "Tracking::GrabImageMonocular : New frame made." << endl;
    }

    Track();

    return mCurrentFrame.mTcw.clone();
}

void Tracking::Track()
{
    if(mState==NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }

    mLastProcessedState=mState;

    // Get Map Mutex -> Map cannot be changed
    unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

    cout << endl;
    
    if(mState==NOT_INITIALIZED && mpMap->GetMaxKFid() == 0)
    {

        if(mSensor==System::STEREO || mSensor==System::RGBD){
            cout << "Tracking.cc :: Stereo Initializing......" << endl;
            StereoInitialization();
        }
        else if (mSensor == System::TWOSTEREO)
        {
            cout << "Tracking.cc :: TwoStereo Initializing......" << endl;
            TwoStereoInitialization();
        }
        else{
            cout << "Tracking.cc :: Monocular Initializing......" << endl;
            MonocularInitialization();
        }

        mpFrameDrawer->Update(this);

        if(mState!=OK){
            cout << "mState!=OK, return."<< endl;
            return;
        }
    }

    else if(mState==NOT_INITIALIZED && mpMap->GetMaxKFid() > 0)
    {

        if(mSensor==System::STEREO || mSensor==System::RGBD){
            cout << "Tracking.cc :: Stereo Initializing with map......" << endl;
            StereoInitializationWithMap();
            cout << "Map initialized for Stereo camera. Tracking thread keep working." << endl;
        }
        else{
            cout << "Tracking.cc :: Monocular Initializing with map......" << endl;
            MonocularInitializationWithMap();
        }

        mpFrameDrawer->Update(this);

        if(mState!=OK){
            cout << "mState!=OK, return."<< endl;
            mState = NOT_INITIALIZED;
            return;
        }
    }

    else
    {
        // System is initialized. Track Frame.
        // cout << "System is initialized. Track Frame." << endl;
        bool bOK;

        // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
        if(!mbOnlyTracking)
        {
            // Local Mapping is activated. This is the normal behaviour, unless
            // you explicitly activate the "only tracking" mode.
            // cout << "Tracking.cc :: Local Mapping is activated. mState = " << mState << endl;
            if(mState==OK)
            {
                // Local Mapping might have changed some MapPoints tracked in last frame
                // cout << "Tracking.cc :: CheckReplacedInLastFrame() " << endl;
                CheckReplacedInLastFrame();
                // cout << "Tracking.cc :: CurrentFrame.mnId is: " << mCurrentFrame.mnId << endl;
                // cout << "Tracking.cc :: mnLastRelocFrameId is: " << mnLastRelocFrameId << endl;
                if(mVelocity.empty() || mCurrentFrame.mnId<mnLastRelocFrameId+2)
                {   
                    bOK = TrackReferenceKeyFrame();
                    // if(!bOK){
                    //     cout << "nmatches<15" << endl;
                    // }
                    
                }
                else
                {
                    bOK = TrackWithMotionModel(); // Among all the if branches, our program usually goes into here.
                    // cout << "Tracking.cc :: TrackWithMotionModel(). " << endl;
                    if(!bOK){
                        bOK = TrackReferenceKeyFrame();
                    }
                        
                }
            }
            else
            {
                // cout << "Tracking.cc :: Relocalization() " << endl;
                bOK = Relocalization();
            }
        }
        else
        {
            // Localization Mode: Local Mapping is deactivated
            cout << endl << "Tracking.cc :: Pure localization mode." << endl;

            if(mState==LOST)
            {
                cout << "Tracking.cc :: mState==LOST, Relocalization();" << endl;
                bOK = Relocalization();
            }
            else
            {
                if(!mbVO)
                {
                    // In last frame we tracked enough MapPoints in the map
                    cout << "Tracking.cc :: Now mbVO is false, means in the last frame we tracked enough MapPoints in the map(nmatchesMap>20);" << endl;
                    if(!mVelocity.empty())
                    {
                        cout << "Tracking.cc :: mVelocity not empty, TrackWithMotionModel();" << endl;
                        bOK = TrackWithMotionModel();
                    }
                    else
                    {
                        cout << "Tracking.cc :: mVelocity is empty, TrackReferenceKeyFrame();" << endl;
                        bOK = TrackReferenceKeyFrame();
                    }
                }
                else
                {
                    // In last frame we tracked mainly "visual odometry" points.
                    cout << "Tracking.cc :: Now mbVO is true, means that when we do relocalization, nmatchesMap<10; mbVO will be false if nmatchesMap>20;" << endl;

                    // We compute two camera poses, one from motion model and one doing relocalization.
                    // If relocalization is sucessfull we choose that solution, otherwise we retain
                    // the "visual odometry" solution.

                    bool bOKMM = false;
                    bool bOKReloc = false;
                    vector<MapPoint*> vpMPsMM;
                    vector<bool> vbOutMM;
                    cv::Mat TcwMM;
                    if(!mVelocity.empty())
                    {
                        bOKMM = TrackWithMotionModel();
                        vpMPsMM = mCurrentFrame.mvpMapPoints;
                        vbOutMM = mCurrentFrame.mvbOutlier;
                        TcwMM = mCurrentFrame.mTcw.clone();
                    }
                    bOKReloc = Relocalization();

                    if(bOKMM && !bOKReloc)
                    {
                        cout << "Tracking.cc :: bOKMM && !bOKReloc; means TrackWithMotionModel() succeed, Relocalization() failed." << endl;
                        mCurrentFrame.SetPose(TcwMM);
                        mCurrentFrame.mvpMapPoints = vpMPsMM;
                        mCurrentFrame.mvbOutlier = vbOutMM;

                        if(mbVO)
                        {
                            cout << "Tracking.cc :: mbVO; Add current keypoints to mvpMapPoints." << endl;
                            for(int i =0; i<mCurrentFrame.N; i++)
                            {
                                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                                {
                                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                                }
                            }
                        }
                    }
                    else if(bOKReloc)
                    {
                        mbVO = false;
                        cout << "Tracking.cc :: !bOKMM && bOKReloc; means TrackWithMotionModel() failed, Relocalization() succeed." << endl;
                    }

                    bOK = bOKReloc || bOKMM;
                    if(!bOK){ // for debug use
                        cout << "Tracking.cc :: !bOK, because bOKReloc = " << bOKReloc << ", bOKMM = " << bOKMM << endl;
                    }
                }
            }
        }

        mCurrentFrame.mpReferenceKF = mpReferenceKF;
        // for two stereos
        if (mSensor==System::TWOSTEREO)
            mCurrentFrameBack.mpReferenceKF = mpReferenceKFBack;

        // If we have an initial estimation of the camera pose and matching. Track the local map.
        if(!mbOnlyTracking)
        {
            // cout << "Tracking.cc :: Track(line446), bOK =" << bOK << endl;
            if(bOK){
                // cout << "Tracking.cc :: Track, now into TrackLocalMap()." << endl;
                bOK = TrackLocalMap();
            }
        }
        else
        {
            // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
            // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
            // the camera we will use the local map again.
            if(bOK && !mbVO){
                cout << "Tracking.cc :: bOK && !mbVO; means that there are many matches in the map, so we can TrackLocalMap()." << endl;
                bOK = TrackLocalMap();
            }
        }

        if(bOK){
            cout << "Tracking.cc :: Track(), Now mState is OK." << endl;
            mState = OK;
        }
        else{
            cout << "Tracking.cc :: Track(), Now mState is LOST." << endl;
            mState=LOST;
        }

        // Update drawer
        mpFrameDrawer->Update(this);

        // If tracking were good, check if we insert a keyframe
        if(bOK)
        {
            // Update motion model
            if(!mLastFrame.mTcw.empty())
            {
                cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
                mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
                mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
                mVelocity = mCurrentFrame.mTcw*LastTwc;
            }
            else
                mVelocity = cv::Mat();

            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

            // Clean VO matches
            for(int i=0; i<mCurrentFrame.N; i++)
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(pMP)
                    if(pMP->Observations()<1)
                    {
                        mCurrentFrame.mvbOutlier[i] = false;
                        mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    }
            }

            // Delete temporal MapPoints
            for(list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend =  mlpTemporalPoints.end(); lit!=lend; lit++)
            {
                MapPoint* pMP = *lit;
                delete pMP;
            }
            mlpTemporalPoints.clear();

            // Check if we need to insert a new keyframe
            if(NeedNewKeyFrame())
            {
                CreateNewKeyFrame();
                if(mSensor==System::TWOSTEREO){
                    SetCameraBackTcw(mCurrentFrame,mCurrentFrameBack,0.1);
                    CreateNewKeyFrameBack();
                }                    
            }

            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame.
            for(int i=0; i<mCurrentFrame.N;i++)
            {
                if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
            }
            if(mSensor==System::TWOSTEREO){
                for(int i=0;i<mCurrentFrameBack.N;i++){
                    if(mCurrentFrameBack.mvpMapPoints[i]&&mCurrentFrameBack.mvbOutlier[i])
                        mCurrentFrameBack.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);                    
                }
            }
        }

        // Reset if the camera get lost soon after initialization
        
        if(mState==LOST)
        {
            if(mpMap->KeyFramesInMap()<=5)
            {
                cout << "Track lost soon after initialisation, reseting..." << endl;
                mpSystem->Reset();
                return;
            }
        }

        if(!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

        mLastFrame = Frame(mCurrentFrame);
        if(mSensor==System::TWOSTEREO)
            mLastFrameBack = Frame(mCurrentFrameBack);
    }

    // Store frame pose information to retrieve the complete camera trajectory afterwards.
    if(!mCurrentFrame.mTcw.empty())
    {   
        // cout << "Tracking::Track() : !mCurrentFrame.mTcw.empty())" << endl;
        cv::Mat Tcr = mCurrentFrame.mTcw*mCurrentFrame.mpReferenceKF->GetPoseInverse();
        mlRelativeFramePoses.push_back(Tcr);
        mlpReferences.push_back(mpReferenceKF);
        mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
        mlbLost.push_back(mState==LOST);
    }
    else
    {
        // This can happen if tracking is lost
        cout << "Tracking::Track() : Tracking is lost" << endl;
        mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
        mlpReferences.push_back(mlpReferences.back());
        mlFrameTimes.push_back(mlFrameTimes.back());
        mlbLost.push_back(mState==LOST);
    }

}

void Tracking::TwoStereoInitialization()
{
    if (mCurrentFrame.N > 500 && mCurrentFrameBack.N > 500)
    {
        // Set Frame pose to the origin
        mCurrentFrame.SetPose(cv::Mat::eye(4, 4, CV_32F));
        // for two stereos
        SetCameraBackTcw(mCurrentFrame, mCurrentFrameBack, 0.1);

        // 将当前帧（第一帧）作为初始关键帧（调用关键帧的构造函数）
        KeyFrame *pKFini = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB);
        // For Two Stereo camera back
        KeyFrame *pKFiniBack = new KeyFrame(mCurrentFrameBack, mpMap, mpKeyFrameDB);

        // Insert KeyFrame in the map
        // 将关键帧插入地图中.  KeyFrame中包含了地图、反过来地图中也包含了KeyFrame，相互包含
        mpMap->AddKeyFrame(pKFini);
        // For Two Stereo camera back
        // 创建地图点并将其与关键帧建立联系
        mpMap->AddKeyFrame(pKFiniBack);

        // Create MapPoints and asscoiate to KeyFrame
        for (int i = 0; i < mCurrentFrame.N; i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if (z > 0)
            {
                cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                MapPoint *pNewMP = new MapPoint(x3D, pKFini, mpMap);
                pNewMP->AddObservation(pKFini, i);
                pKFini->AddMapPoint(pNewMP, i);
                pNewMP->ComputeDistinctiveDescriptors();
                pNewMP->UpdateNormalAndDepth();
                mpMap->AddMapPoint(pNewMP);

                mCurrentFrame.mvpMapPoints[i] = pNewMP;
            }
        }
        //For Two stereo camera
        for (int i = 0; i < mCurrentFrameBack.N; i++)
        {
            float z = mCurrentFrameBack.mvDepth[i]; //获取当前帧第i个关键点的深度值
            if (z > 0)
            {
                cv::Mat x3D = mCurrentFrameBack.UnprojectStereo(i);      // 将当前帧的第i个特征点反投影到3D世界坐标系下
                MapPoint *pNewMP = new MapPoint(x3D, pKFiniBack, mpMap); // 用该特征点构造新的地图点
                pNewMP->AddObservation(pKFiniBack, i);                   // 地图点添加关键帧  说明该地图点属于哪一关键帧
                pKFiniBack->AddMapPoint(pNewMP, i);                      // 关键帧添加地图点  表明在该关键帧下可以看到该地图点
                pNewMP->ComputeDistinctiveDescriptors();                 // 从众多观测到该MapPoint的特征点中挑选区分读最高的描述子
                pNewMP->UpdateNormalAndDepth();                          // 更新该MapPoint平均观测方向以及观测距离的范围
                mpMap->AddMapPoint(pNewMP);                              // 将新的地图点加入到地图中

                mCurrentFrameBack.mvpMapPoints[i] = pNewMP;
            }
        }

        cout << "New map created with " << mpMap->MapPointsInMap() << " points" << endl;

        mpLocalMapper->InsertKeyFrame(pKFini);         //在局部地图中添加该初始关键帧
        mpLocalMapperBack->InsertKeyFrame(pKFiniBack); // For Two stereo camera back

        mLastFrame = Frame(mCurrentFrame); //更新上一帧为当前帧
        mnLastKeyFrameId = mCurrentFrame.mnId;
        mpLastKeyFrame = pKFini; // 更新上一关键帧为当前关键帧

        // for two stereos
        mLastFrameBack = Frame(mCurrentFrameBack);
        mnLastKeyFrameIdBack = mCurrentFrameBack.mnId;
        mpLastKeyFrameBack = pKFiniBack;

        mvpLocalKeyFrames.push_back(pKFini);          // 将初始关键帧加入到局部地图的关键帧
        mvpLocalMapPoints = mpMap->GetAllMapPoints(); // 将全部地图点加入到当前局部地图点

        mvpLocalKeyFramesBack.push_back(pKFiniBack);      // for two stereos
        mvpLocalMapPointsBack = mpMap->GetAllMapPoints(); // for two stereos
        mpReferenceKF = pKFini;                           // 将当前关键帧作为参考关键帧
        mCurrentFrame.mpReferenceKF = pKFini;             // 将该关键帧作为当前帧的参考关键帧

        // For Two stereo camera back
        mpReferenceKFBack = pKFiniBack;
        mCurrentFrameBack.mpReferenceKF = pKFiniBack;

        mpMap->SetReferenceMapPoints(mvpLocalMapPoints); // 将当前局部地图点作为整个地图参考地图点,用于画图
        // mpMap->SetReferenceMapPointsBack(mvpLocalMapPointsBack); // for visualization

        mpMap->mvpKeyFrameOrigins.push_back(pKFini); // 将关键帧加入地图的原始的关键帧

        mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw); //将当前帧加入到地图观测器
        // For Two stereo camera back
        mpMapDrawer->SetCurrentCameraPose(mCurrentFrameBack.mTcw);

        mState = OK;
    }
}

void Tracking::StereoInitialization()
{
    if(mCurrentFrame.N>500)
    {
        // Set Frame pose to the origin
        mCurrentFrame.SetPose(cv::Mat::eye(4,4,CV_32F));

        // Create KeyFrame
        KeyFrame* pKFini = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

        // Insert KeyFrame in the map
        mpMap->AddKeyFrame(pKFini);

        // Create MapPoints and asscoiate to KeyFrame
        for(int i=0; i<mCurrentFrame.N;i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                MapPoint* pNewMP = new MapPoint(x3D,pKFini,mpMap);
                pNewMP->AddObservation(pKFini,i);
                pKFini->AddMapPoint(pNewMP,i);
                pNewMP->ComputeDistinctiveDescriptors();
                pNewMP->UpdateNormalAndDepth();
                mpMap->AddMapPoint(pNewMP);

                mCurrentFrame.mvpMapPoints[i]=pNewMP;
            }
        }

        cout << "New map created with " << mpMap->MapPointsInMap() << " points" << endl;

        mpLocalMapper->InsertKeyFrame(pKFini);

        mLastFrame = Frame(mCurrentFrame);
        mnLastKeyFrameId=mCurrentFrame.mnId;
        mpLastKeyFrame = pKFini;

        mvpLocalKeyFrames.push_back(pKFini);
        mvpLocalMapPoints=mpMap->GetAllMapPoints();
        mpReferenceKF = pKFini;
        mCurrentFrame.mpReferenceKF = pKFini;

        mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

        mpMap->mvpKeyFrameOrigins.push_back(pKFini);

        mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

        mState=OK;
    }
}

void Tracking::StereoInitializationWithMap()
{
    if(mCurrentFrame.N>500)
    {
        
        // New map is loaded. First, let's relocalize the current frame.
        mCurrentFrame.SetPose(cv::Mat::eye(4,4,CV_32F));

        bool bOK;
        bOK = Relocalization();

        // Then the rest few lines in this funtion is the same as the last half part of the Tracking::Track() function.
        mCurrentFrame.mpReferenceKF = mpReferenceKF;

        // If we have an initial estimation of the camera pose and matching. Track the local map.
        if(!mbOnlyTracking)
        {
            if(bOK)
                // cout << "Tracking.cc :: Track, now into TrackLocalMap()." << endl;
                bOK = TrackLocalMap();
        }
        else
        {
            // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
            // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
            // the camera we will use the local map again.
            if(bOK && !mbVO)
                bOK = TrackLocalMap();
        }

        if(bOK){
            // cout << "Tracking.cc :: Track(), Now mState is OK." << endl;
            mState = OK;
        }
        else{
            // cout << "Tracking.cc :: Track(), Now mState is LOST." << endl;
            mState=LOST;
        }

        // Update drawer
        mpFrameDrawer->Update(this);

        // If tracking were good, check if we insert a keyframe
        if(bOK)
        {
            // Update motion model
            if(!mLastFrame.mTcw.empty())
            {
                cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
                mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
                mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
                mVelocity = mCurrentFrame.mTcw*LastTwc;
            }
            else
                mVelocity = cv::Mat();

            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

            // Clean VO matches
            for(int i=0; i<mCurrentFrame.N; i++)
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(pMP)
                    if(pMP->Observations()<1)
                    {
                        mCurrentFrame.mvbOutlier[i] = false;
                        mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    }
            }

            // Delete temporal MapPoints
            for(list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend =  mlpTemporalPoints.end(); lit!=lend; lit++)
            {
                MapPoint* pMP = *lit;
                delete pMP;
            }
            mlpTemporalPoints.clear();

            // Check if we need to insert a new keyframe
            if(NeedNewKeyFrame())
                CreateNewKeyFrame();

            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame.
            for(int i=0; i<mCurrentFrame.N;i++)
            {
                if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
            }
        }

        // Reset if the camera get lost soon after initialization
        if(mState==LOST)
        {
            if(mpMap->KeyFramesInMap()<=5)
            {
                mpSystem->Reset();
                return;
            }
        }

        if(!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

        mLastFrame = Frame(mCurrentFrame);
    }
}

void Tracking::MonocularInitialization()
{

    if(!mpInitializer)
    {
        // Set Reference Frame
        if(mCurrentFrame.mvKeys.size()>100)
        {
            mInitialFrame = Frame(mCurrentFrame);
            mLastFrame = Frame(mCurrentFrame);
            mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
            for(size_t i=0; i<mCurrentFrame.mvKeysUn.size(); i++)
                mvbPrevMatched[i]=mCurrentFrame.mvKeysUn[i].pt;

            if(mpInitializer)
                delete mpInitializer;

            mpInitializer =  new Initializer(mCurrentFrame,1.0,200);

            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);

            return;
        }
    }
    else
    {
        // Try to initialize
        if((int)mCurrentFrame.mvKeys.size()<=100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
            return;
        }

        // Find correspondences
        ORBmatcher matcher(0.9,true);
        int nmatches = matcher.SearchForInitialization(mInitialFrame,mCurrentFrame,mvbPrevMatched,mvIniMatches,100);

        // Check if there are enough correspondences
        if(nmatches<100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            return;
        }

        cv::Mat Rcw; // Current Camera Rotation
        cv::Mat tcw; // Current Camera Translation
        vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

        if(mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated))
        {
            for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
            {
                if(mvIniMatches[i]>=0 && !vbTriangulated[i])
                {
                    mvIniMatches[i]=-1;
                    nmatches--;
                }
            }

            // Set Frame Poses
            mInitialFrame.SetPose(cv::Mat::eye(4,4,CV_32F));
            cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
            Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
            tcw.copyTo(Tcw.rowRange(0,3).col(3));
            mCurrentFrame.SetPose(Tcw);

            CreateInitialMapMonocular();
        }
    }
}

void Tracking::MonocularInitializationWithMap()
{

    if(mCurrentFrame.N>500)
    {
        
        // New map is loaded. First, let's relocalize the current frame.
        mCurrentFrame.SetPose(cv::Mat::eye(4,4,CV_32F));

        bool bOK;
        bOK = Relocalization();

        // Then the rest few lines in this funtion is the same as the last half part of the Tracking::Track() function.
        mCurrentFrame.mpReferenceKF = mpReferenceKF;

        // If we have an initial estimation of the camera pose and matching. Track the local map.
        if(!mbOnlyTracking)
        {
            if(bOK)
                // cout << "Tracking.cc :: Track, now into TrackLocalMap()." << endl;
                bOK = TrackLocalMap();
        }
        else
        {
            // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
            // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
            // the camera we will use the local map again.
            if(bOK && !mbVO)
                bOK = TrackLocalMap();
        }

        if(bOK){
            // cout << "Tracking.cc :: Track(), Now mState is OK." << endl;
            mState = OK;
        }
        else{
            // cout << "Tracking.cc :: Track(), Now mState is LOST." << endl;
            mState=LOST;
        }

        // Update drawer
        mpFrameDrawer->Update(this);

        // If tracking were good, check if we insert a keyframe
        if(bOK)
        {
            // Update motion model
            if(!mLastFrame.mTcw.empty())
            {
                cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
                mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
                mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
                mVelocity = mCurrentFrame.mTcw*LastTwc;
            }
            else
                mVelocity = cv::Mat();

            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

            // Clean VO matches
            for(int i=0; i<mCurrentFrame.N; i++)
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(pMP)
                    if(pMP->Observations()<1)
                    {
                        mCurrentFrame.mvbOutlier[i] = false;
                        mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    }
            }

            // Delete temporal MapPoints
            for(list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend =  mlpTemporalPoints.end(); lit!=lend; lit++)
            {
                MapPoint* pMP = *lit;
                delete pMP;
            }
            mlpTemporalPoints.clear();

            // Check if we need to insert a new keyframe
            if(NeedNewKeyFrame())
                CreateNewKeyFrame();

            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame.
            for(int i=0; i<mCurrentFrame.N;i++)
            {
                if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
            }
        }

        // Reset if the camera get lost soon after initialization
        if(mState==LOST)
        {
            if(mpMap->KeyFramesInMap()<=5)
            {
                mpSystem->Reset();
                return;
            }
        }

        if(!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

        mLastFrame = Frame(mCurrentFrame);
    }
}

void Tracking::CreateInitialMapMonocular()
{
    // Create KeyFrames
    KeyFrame* pKFini = new KeyFrame(mInitialFrame,mpMap,mpKeyFrameDB);
    KeyFrame* pKFcur = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);


    pKFini->ComputeBoW();
    pKFcur->ComputeBoW();

    // Insert KFs in the map
    mpMap->AddKeyFrame(pKFini);
    mpMap->AddKeyFrame(pKFcur);

    // Create MapPoints and asscoiate to keyframes
    for(size_t i=0; i<mvIniMatches.size();i++)
    {
        if(mvIniMatches[i]<0)
            continue;

        //Create MapPoint.
        cv::Mat worldPos(mvIniP3D[i]);

        MapPoint* pMP = new MapPoint(worldPos,pKFcur,mpMap);

        pKFini->AddMapPoint(pMP,i);
        pKFcur->AddMapPoint(pMP,mvIniMatches[i]);

        pMP->AddObservation(pKFini,i);
        pMP->AddObservation(pKFcur,mvIniMatches[i]);

        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth();

        //Fill Current Frame structure
        mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
        mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

        //Add to Map
        mpMap->AddMapPoint(pMP);
    }

    // Update Connections
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    // Bundle Adjustment
    cout << "New Map created with " << mpMap->MapPointsInMap() << " points" << endl;

    Optimizer::GlobalBundleAdjustemnt(mpMap,20);

    // Set median depth to 1
    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth = 1.0f/medianDepth;

    if(medianDepth<0 || pKFcur->TrackedMapPoints(1)<100)
    {
        cout << "Wrong initialization, reseting..." << endl;
        Reset();
        return;
    }

    // Scale initial baseline
    cv::Mat Tc2w = pKFcur->GetPose();
    Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth;
    pKFcur->SetPose(Tc2w);

    // Scale points
    vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
    for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
    {
        if(vpAllMapPoints[iMP])
        {
            MapPoint* pMP = vpAllMapPoints[iMP];
            pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
        }
    }

    mpLocalMapper->InsertKeyFrame(pKFini);
    mpLocalMapper->InsertKeyFrame(pKFcur);

    mCurrentFrame.SetPose(pKFcur->GetPose());
    mnLastKeyFrameId=mCurrentFrame.mnId;
    mpLastKeyFrame = pKFcur;

    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);
    mvpLocalMapPoints=mpMap->GetAllMapPoints();
    mpReferenceKF = pKFcur;
    mCurrentFrame.mpReferenceKF = pKFcur;

    mLastFrame = Frame(mCurrentFrame);

    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

    mpMap->mvpKeyFrameOrigins.push_back(pKFini);

    mState=OK;
}

void Tracking::CheckReplacedInLastFrame()
{   
    for(int i =0; i<mLastFrame.N; i++)
    {
        MapPoint* pMP = mLastFrame.mvpMapPoints[i];

        if(pMP)
        {
            MapPoint* pRep = pMP->GetReplaced();
            if(pRep)
            {
                mLastFrame.mvpMapPoints[i] = pRep;
                // cout << "Tracking::TrackReferenceKeyFrame(line 771) : In the last frame, mvpMapPoints[" << i <<"] got replaced." << endl;
            }
        }
    }
    if(mSensor==System::TWOSTEREO){
        for (int i=0; i<mLastFrameBack.N; i++){
            MapPoint *pMP = mLastFrameBack.mvpMapPoints[i];
            if(pMP){
                MapPoint* pRep = pMP->GetReplaced();
                if(pRep){
                    mLastFrameBack.mvpMapPoints[i]=pRep;
                    cout << "Tracking::TrackReferenceKeyFrame(line 771) : In the last frame back, mvpMapPoints[" << i <<"] got replaced." << endl;
                }
            }
        }
        
    }
}


bool Tracking::TrackReferenceKeyFrame()
{
    // Compute Bag of Words vector
    mCurrentFrame.ComputeBoW();

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.7,true);
    vector<MapPoint*> vpMapPointMatches;

    int nmatches = matcher.SearchByBoW(mpReferenceKF,mCurrentFrame,vpMapPointMatches);

    if(nmatches<15){
        return false;
    }

    vector<MapPoint *> vpMapPointMatchesBack;
    int nmatchesBack;
    if (mSensor == System::TWOSTEREO)
    {
        SetCameraBackTcw(mCurrentFrame, mCurrentFrameBack, 0.1);
        mCurrentFrameBack.ComputeBoW();
        nmatchesBack = matcher.SearchByBoW(mpReferenceKFBack, mCurrentFrameBack, vpMapPointMatchesBack);
        mCurrentFrameBack.mvpMapPoints = vpMapPointMatchesBack;
    }

    mCurrentFrame.mvpMapPoints = vpMapPointMatches;
    mCurrentFrame.SetPose(mLastFrame.mTcw);

    Optimizer::PoseOptimization(&mCurrentFrame);
    // for two stereos
    if (mSensor == System::TWOSTEREO)
        SetCameraBackTcw(mCurrentFrame, mCurrentFrameBack, 0.1);

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
                mCurrentFrame.mvbDiscarded[i]=true; // For debug use
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }

    if (nmatchesBack>=10){
        SetCameraBackTcw(mCurrentFrame, mCurrentFrameBack, 0.1);
        mCurrentFrameBack.mvpMapPoints = vpMapPointMatchesBack;
        for (int i = 0; i < mCurrentFrame.N; i++)
        {
            if (mCurrentFrameBack.mvpMapPoints[i])
            {
                if (mCurrentFrameBack.mvbOutlier[i])
                {
                    MapPoint *pMP = mCurrentFrameBack.mvpMapPoints[i];

                    mCurrentFrameBack.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                    mCurrentFrameBack.mvbOutlier[i] = false;
                    pMP->mbTrackInView = false;
                    pMP->mnLastFrameSeen = mCurrentFrameBack.mnId;
                    nmatchesBack--;
                    mCurrentFrameBack.mvbDiscarded[i] = true; // For debug use
                }
            }
        }
    }



    return nmatchesMap>=10;
}

void Tracking::UpdateLastFrame()
{
    // Update pose according to reference keyframe
    KeyFrame* pRef = mLastFrame.mpReferenceKF;
    cv::Mat Tlr = mlRelativeFramePoses.back();
    cout <<"Tracking::UpdateLastFrame(), mnLastKeyFrameId =" << mnLastKeyFrameId << endl;
    cout <<"Tracking::UpdateLastFrame(), mLastFrame.mnId =" << mLastFrame.mnId << endl;

    mLastFrame.SetPose(Tlr*pRef->GetPose());

    if(mnLastKeyFrameId==mLastFrame.mnId || mSensor==System::MONOCULAR || !mbOnlyTracking){
        if(mnLastKeyFrameId==mLastFrame.mnId){
            cout <<"Tracking::UpdateLastFrame(), mnLastKeyFrameId==mLastFrame.mnId==" << mnLastKeyFrameId << ", return." << endl;
        }
        if(mSensor==System::MONOCULAR){
            cout <<"Tracking::UpdateLastFrame(), mSensor==System::MONOCULAR, return." << endl;
        }
        if(!mbOnlyTracking){
            cout <<"Tracking::UpdateLastFrame(), This is not tracking only mode, return." << endl;
        }
        return;
    }

    // Create "visual odometry" MapPoints
    // We sort points according to their measured depth by the stereo/RGB-D sensor
    vector<pair<float,int> > vDepthIdx;
    vDepthIdx.reserve(mLastFrame.N);
    // cout <<"Tracking::UpdateLastFrame(), mLastFrame.N = " << mLastFrame.N << endl;
    for(int i=0; i<mLastFrame.N;i++)
    {
        float z = mLastFrame.mvDepth[i];
        if(z>0)
        {
            vDepthIdx.push_back(make_pair(z,i));
        }
    }

    if(vDepthIdx.empty()){
        cout <<"Tracking::UpdateLastFrame(), vDepthIdx.empty(), return." << endl;
        return;
    }

    sort(vDepthIdx.begin(),vDepthIdx.end());

    // We insert all close points (depth<mThDepth)
    // If less than 100 close points, we insert the 100 closest ones.
    int nPoints = 0;
    for(size_t j=0; j<vDepthIdx.size();j++)
    {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;

        MapPoint* pMP = mLastFrame.mvpMapPoints[i];
        if(!pMP)
            bCreateNew = true;
        else if(pMP->Observations()<1)
        {
            bCreateNew = true;
        }

        if(bCreateNew)
        {
            cv::Mat x3D = mLastFrame.UnprojectStereo(i);
            MapPoint* pNewMP = new MapPoint(x3D,mpMap,&mLastFrame,i);

            mLastFrame.mvpMapPoints[i]=pNewMP;

            mlpTemporalPoints.push_back(pNewMP);
            nPoints++;
        }
        else
        {
            nPoints++;
        }

        if (mSensor == System::TWOSTEREO)
        {
            SetCameraBackTcw(mLastFrame, mLastFrameBack, 0.1);
            vector<pair<float, int>> vDepthIdxBack;
            vDepthIdxBack.reserve(mLastFrameBack.N);
            for (int i = 0; i < mLastFrameBack.N; i++)
            {
                float z = mLastFrameBack.mvDepth[i];
                if (z > 0)
                    vDepthIdxBack.push_back(make_pair(z, i));
            }
            if (!vDepthIdxBack.empty())
            {
                sort(vDepthIdxBack.begin(), vDepthIdxBack.end());

                for (size_t j = 0; j < vDepthIdxBack.size(); j++)
                {
                    int i = vDepthIdxBack[j].second;
                    bool bCreateNew = false;
                    MapPoint *pMP = mLastFrameBack.mvpMapPoints[i];
                    if (!pMP)
                        bCreateNew = true;
                    else if (pMP->Observations() < 1)
                        bCreateNew = true;

                    if (bCreateNew)
                    {
                        cv::Mat x3D = mLastFrameBack.UnprojectStereo(i);
                        MapPoint *pNewMP = new MapPoint(x3D, mpMap, &mLastFrameBack, i);
                        mLastFrameBack.mvpMapPoints[i] = pNewMP;
                    }
                }
            }
        }

        if(vDepthIdx[j].first>mThDepth && nPoints>100)
            break;
    }
    cout <<"Tracking::UpdateLastFrame(), finished. We pushed back " << nPoints << " points into mlpTemporalPoints." << endl;
}

bool Tracking::TrackWithMotionModel()
{
    ORBmatcher matcher(0.9,true);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points if in Localization Mode
    UpdateLastFrame();

    // cout << "Tracking::TrackWithMotionModel(line901) : mVelocity = " << mVelocity << endl;
    // cout << "Tracking::TrackWithMotionModel(line902) : mLastFrame.mTcw = " << mLastFrame.mTcw << endl;

    // If UseIMU(only consider yaw angle), we add another rotation to Current Pose
    if(mbUseIMU){
        cv::Mat yawIMU = cv::Mat::eye(4,4,CV_32F);
        yawIMU.at<float>(0,0) = cos(yaw_angle_accums);
        yawIMU.at<float>(0,1) = -sin(yaw_angle_accums);
        yawIMU.at<float>(1,0) = sin(yaw_angle_accums);
        yawIMU.at<float>(1,1) = cos(yaw_angle_accums);
        cout << "Tracking::TrackWithMotionModel() : yaw_angle_accums = " << yaw_angle_accums << endl;   
        mCurrentFrame.SetPose(yawIMU*mVelocity*mLastFrame.mTcw);
    }
    else{
        mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw);
    }
    if(mSensor==System::TWOSTEREO)
        SetCameraBackTcw(mCurrentFrame, mCurrentFrameBack, 0.1);

    // mVelocity is a 4x4 SE3 matrix. cv::Mat mVelocity = mCurrentFrame.mTcw*LastTwc
    // mLastFrame.mTcw is also a 4x4 SE3 matrix

    fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
    // cout << "Tracking::TrackWithMotionModel(line908) : This frame sees " << mCurrentFrame.mvpMapPoints.size() << " map points." << endl;

    // Project points seen in previous frame
    int th;
    if(mSensor!=System::STEREO)
        th=15;
    else
        th=7;
    int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th,mSensor==System::MONOCULAR);
    cout << "Tracking::TrackWithMotionModel() : At first, nmatches = " << nmatches << endl; // Note, nmatches and nmatchesMap are different!! though they'll be equal at last.

    // If few matches, uses a wider window search
    if(nmatches<20)
    {
        cout << "Tracking::TrackWithMotionModel() : nmatches<20, uses a wider window search" << endl;
        fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
        nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,2*th,mSensor==System::MONOCULAR);
    }

    if(mSensor==System::TWOSTEREO){
        int nmatchesBack = matcher.SearchByProjection(mCurrentFrameBack,mLastFrameBack,15,mSensor==System::MONOCULAR);
        if(nmatchesBack<20)
            int nmatchesBack = matcher.SearchByProjection(mCurrentFrameBack,mLastFrameBack,30,mSensor==System::MONOCULAR);
    }

    if(nmatches<20){
        cout << "Tracking::TrackWithMotionModel() : wider window search failed, nmatches<20, return" << endl;
        return false;
    }

    // Optimize frame pose with all matches
    cout << "Tracking::TrackWithMotionModel() : Now we put these " << nmatches << " into Optimizer::PoseOptimization()." << endl;
    int nGoodMatches = Optimizer::PoseOptimization(&mCurrentFrame);
    cout << "Tracking::TrackWithMotionModel() : After PoseOptimization, we found " << nGoodMatches << " good matches and " << (nmatches-nGoodMatches) << " bad matches among those " << nmatches << " matches." << endl;

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
                mCurrentFrame.mvbDiscarded[i]=true; // For debug use
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }    
    cout << "Tracking::TrackWithMotionModel() : After picking out outliers, nmatches = " << nmatches << " out of " << mCurrentFrame.N << " points." << endl;
    cout << "Tracking::TrackWithMotionModel() : nmatchesMap = " << nmatchesMap << endl;

    if(mbOnlyTracking)
    {
        mbVO = nmatchesMap<10;
        if(mbVO){
            cout << "Tracking::TrackWithMotionModel() : Not enough nmatchesMap, we should use VO in the next iteration. "<< endl;
        }
        return nmatches>20;
    }
    return nmatchesMap>=10;
}

bool Tracking::TrackLocalMap()
{
    // We have an estimation of the camera pose and some map points tracked in the frame.
    // We retrieve the local map and try to find matches to points in the local map.
    // cout << "Tracking::TrackLocalMap() : UpdateLocalMap();" << endl;
    UpdateLocalMap();
    if(mSensor == System::TWOSTEREO)
        UpdateLocalMapBack();
    // cout << "Tracking::TrackLocalMap() : UpdateLocalMap() finished" << endl;

    SearchLocalPoints();
    if(mSensor == System::TWOSTEREO)
        SearchLocalPointsBack();
    // cout << "Tracking::TrackLocalMap() : SearchLocalPoints() finished" << endl;

    // Optimize Pose
    cout << "Tracking::TrackLocalMap() : Now we optimize the pose again." << endl;
    Optimizer::PoseOptimization(&mCurrentFrame);
    mnMatchesInliers = 0;

    // Update MapPoints Statistics
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(!mCurrentFrame.mvbOutlier[i])
            {
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                if(!mbOnlyTracking)
                {
                    if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                        mnMatchesInliers++;
                }
                else
                    mnMatchesInliers++;
            }
            else if(mSensor==System::STEREO)
                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
        }
    }

    if (mSensor == System::TWOSTEREO)
    {
        // Update MapPoints Statistics here !!!
        for (int i = 0; i < mCurrentFrameBack.N; i++)
        {
            if (mCurrentFrameBack.mvpMapPoints[i])
            {
                if (!mCurrentFrameBack.mvbOutlier[i])
                {
                    mCurrentFrameBack.mvpMapPoints[i]->IncreaseFound();
                }
            }
        }
    }

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50){
        cout << "Tracking::TrackLocalMap() : relocalization recently, and mnMatchesInliers = " << mnMatchesInliers << " < 50, return" << endl;
        return false;
    }

    if(mnMatchesInliers<30){
        cout << "Tracking::TrackLocalMap() : mnMatchesInliers = " << mnMatchesInliers << " < 30, return" << endl;
        return false;
    }
    else{
        cout << "Tracking::TrackLocalMap() : mnMatchesInliers = " << mnMatchesInliers << ", great!" << endl;
        return true;
    }
}


bool Tracking::NeedNewKeyFrame()
{
    if(mbOnlyTracking)
        return false;

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
        return false;

    const int nKFs = mpMap->KeyFramesInMap();

    // Do not insert keyframes if not enough frames have passed from last relocalisation
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && nKFs>mMaxFrames)
        return false;

    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3;
    if(nKFs<=2)
        nMinObs=2;
    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

    // Local Mapping accept keyframes?
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // Check how many "close" points are being tracked and how many could be potentially created.
    int nNonTrackedClose = 0;
    int nTrackedClose= 0;
    if(mSensor!=System::MONOCULAR)
    {
        for(int i =0; i<mCurrentFrame.N; i++)
        {
            if(mCurrentFrame.mvDepth[i]>0 && mCurrentFrame.mvDepth[i]<mThDepth)
            {
                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                    nTrackedClose++;
                else
                    nNonTrackedClose++;
            }
        }
    }

    bool bNeedToInsertClose = (nTrackedClose<100) && (nNonTrackedClose>70);

    // Thresholds
    float thRefRatio = 0.75f;
    if(nKFs<2)
        thRefRatio = 0.4f;

    if(mSensor==System::MONOCULAR)
        thRefRatio = 0.9f;

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = (mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames && bLocalMappingIdle);
    //Condition 1c: tracking is weak
    const bool c1c =  mSensor!=System::MONOCULAR && (mnMatchesInliers<nRefMatches*0.25 || bNeedToInsertClose) ;
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    const bool c2 = ((mnMatchesInliers<nRefMatches*thRefRatio|| bNeedToInsertClose) && mnMatchesInliers>15);

    if((c1a||c1b||c1c)&&c2)
    {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        if(bLocalMappingIdle)
        {
            return true;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            if(mSensor!=System::MONOCULAR)
            {
                if(mpLocalMapper->KeyframesInQueue()<3)
                    return true;
                else
                    return false;
            }
            else
                return false;
        }
    }
    else
        return false;
}

void Tracking::CreateNewKeyFrame()
{
    if(!mpLocalMapper->SetNotStop(true))
        return;

    KeyFrame* pKF = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

    // for two stereos
    if(mSensor==System::TWOSTEREO){
        KeyFrame *pKFBack = new KeyFrame(mCurrentFrameBack, mpMap, mpKeyFrameDB);
        pKF->AddConnection(pKFBack, 50);
    }        

    mpReferenceKF = pKF;
    mCurrentFrame.mpReferenceKF = pKF;

    if(mSensor!=System::MONOCULAR)
    {
        mCurrentFrame.UpdatePoseMatrices();

        // We sort points by the measured depth by the stereo/RGBD sensor.
        // We create all those MapPoints whose depth < mThDepth.
        // If there are less than 100 close points we create the 100 closest.
        vector<pair<float,int> > vDepthIdx;
        vDepthIdx.reserve(mCurrentFrame.N);
        for(int i=0; i<mCurrentFrame.N; i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                vDepthIdx.push_back(make_pair(z,i));
            }
        }

        if(!vDepthIdx.empty())
        {
            sort(vDepthIdx.begin(),vDepthIdx.end());

            int nPoints = 0;
            for(size_t j=0; j<vDepthIdx.size();j++)
            {
                int i = vDepthIdx[j].second;

                bool bCreateNew = false;

                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(!pMP)
                    bCreateNew = true;
                else if(pMP->Observations()<1)
                {
                    bCreateNew = true;
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
                }

                if(bCreateNew)
                {
                    cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                    MapPoint* pNewMP = new MapPoint(x3D,pKF,mpMap);
                    pNewMP->AddObservation(pKF,i);
                    pKF->AddMapPoint(pNewMP,i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpMap->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                    nPoints++;
                }
                else
                {
                    nPoints++;
                }

                if(vDepthIdx[j].first>mThDepth && nPoints>100)
                    break;
            }
        }
    }

    mpLocalMapper->InsertKeyFrame(pKF);

    mpLocalMapper->SetNotStop(false);

    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;
}

void Tracking::CreateNewKeyFrameBack()
{
    if (!mpLocalMapperBack->SetNotStop(true))
        return;

    KeyFrame *pKFBack = new KeyFrame(mCurrentFrameBack, mpMap, mpKeyFrameDB);

     // for two stereos
    if(mSensor==System::TWOSTEREO){
        KeyFrame *pKF = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB);
        pKFBack->AddConnection(pKF, 50);
    } 
    mpReferenceKFBack = pKFBack;
    mCurrentFrameBack.mpReferenceKF = pKFBack;

    if (mSensor != System::MONOCULAR)
    {
        mCurrentFrameBack.UpdatePoseMatrices();

        // We sort points by the measured depth by the stereo/RGBD sensor.
        // We create all those MapPoints whose depth < mThDepth.
        // If there are less than 100 close points we create the 100 closest.
        vector<pair<float, int>> vDepthIdx;
        vDepthIdx.reserve(mCurrentFrameBack.N);
        for (int i = 0; i < mCurrentFrameBack.N; i++)
        {
            float z = mCurrentFrameBack.mvDepth[i];
            if (z > 0)
            {
                vDepthIdx.push_back(make_pair(z, i));
            }
        }

        if (!vDepthIdx.empty())
        {
            sort(vDepthIdx.begin(), vDepthIdx.end());

            int nPoints = 0;
            for (size_t j = 0; j < vDepthIdx.size(); j++)
            {
                int i = vDepthIdx[j].second;

                bool bCreateNew = false;

                MapPoint *pMP = mCurrentFrameBack.mvpMapPoints[i];
                if (!pMP)
                    bCreateNew = true;
                else if (pMP->Observations() < 1)
                {
                    bCreateNew = true;
                    mCurrentFrameBack.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                }

                if (bCreateNew)
                {
                    cv::Mat x3D = mCurrentFrameBack.UnprojectStereo(i);
                    MapPoint *pNewMP = new MapPoint(x3D, pKFBack, mpMap);
                    pNewMP->AddObservation(pKFBack, i);
                    pKFBack->AddMapPoint(pNewMP, i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpMap->AddMapPoint(pNewMP);

                    mCurrentFrameBack.mvpMapPoints[i] = pNewMP;
                    nPoints++;
                }
                else
                {
                    nPoints++;
                }

                if (vDepthIdx[j].first > mThDepth && nPoints > 100)
                    break;
            }
        }
    }

    mpLocalMapperBack->InsertKeyFrame(pKFBack);

    mpLocalMapperBack->SetNotStop(false);

    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKFBack;
}

void Tracking::SearchLocalPoints()
{
    // Do not search map points already matched
    for(vector<MapPoint*>::iterator vit=mCurrentFrame.mvpMapPoints.begin(), vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP)
        {
            if(pMP->isBad())
            {
                *vit = static_cast<MapPoint*>(NULL);
            }
            else
            {
                pMP->IncreaseVisible();
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                pMP->mbTrackInView = false;
            }
        }
    }

    int nToMatch=0;

    // Project points in frame and check its visibility
    for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        if(pMP->isBad())
            continue;
        // Project (this fills MapPoint variables for matching)
        if(mCurrentFrame.isInFrustum(pMP,0.5))
        {
            pMP->IncreaseVisible();
            nToMatch++;
        }
    }

    if(nToMatch>0)
    {
        ORBmatcher matcher(0.8);
        int th = 1;
        if(mSensor==System::RGBD)
            th=3;
        // If the camera has been relocalised recently, perform a coarser search
        if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
            th=5;
        matcher.SearchByProjection(mCurrentFrame,mvpLocalMapPoints,th);
    }
}

void Tracking::SearchLocalPointsBack()
{
    SetCameraBackTcw(mCurrentFrame,mCurrentFrameBack,0.1);
    // Do not search map points already matched
    for(vector<MapPoint*>::iterator vit=mCurrentFrameBack.mvpMapPoints.begin(), vend=mCurrentFrameBack.mvpMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP)
        {
            if(pMP->isBad())
            {
                *vit = static_cast<MapPoint*>(NULL);
            }
            else
            {
                pMP->IncreaseVisible();
                pMP->mnLastFrameSeen = mCurrentFrameBack.mnId;
                pMP->mbTrackInView = false;
            }
        }
    }

    int nToMatch=0;

    // Project points in frame and check its visibility
    for(vector<MapPoint*>::iterator vit=mvpLocalMapPointsBack.begin(), vend=mvpLocalMapPointsBack.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP->mnLastFrameSeen == mCurrentFrameBack.mnId)
            continue;
        if(pMP->isBad())
            continue;
        // Project (this fills MapPoint variables for matching)
        if(mCurrentFrameBack.isInFrustum(pMP,0.5))
        {
            pMP->IncreaseVisible();
            nToMatch++;
        }
    }

    if(nToMatch>0)
    {
        ORBmatcher matcher(0.8);
        int th = 1;
        // If the camera has been relocalised recently, perform a coarser search
        if(mCurrentFrameBack.mnId<mnLastRelocFrameId+2)
            th=5;
        matcher.SearchByProjection(mCurrentFrameBack,mvpLocalMapPointsBack,th);
    }
}

void Tracking::UpdateLocalMap()
{
    // This is for visualization
    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);
    // Update
    UpdateLocalKeyFrames();
    UpdateLocalPoints();
}

void Tracking::UpdateLocalMapBack()
{
    // mpMap->SetReferenceMapPointsBack(mvpLocalMapPointsBack);
    UpdateLocalKeyFramesBack();
    UpdateLocalPointsBack();
}

void Tracking::UpdateLocalPoints()
{
    mvpLocalMapPoints.clear();

    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        KeyFrame* pKF = *itKF;
        const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

        for(vector<MapPoint*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {
            MapPoint* pMP = *itMP;
            if(!pMP)
                continue;
            if(pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId)
                continue;
            if(!pMP->isBad())
            {
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;
            }
        }
    }
}

void Tracking::UpdateLocalKeyFrames()
{
    // Each map point vote for the keyframes in which it has been observed
    map<KeyFrame*,int> keyframeCounter;
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
            if(!pMP->isBad())
            {
                const map<KeyFrame*,size_t> observations = pMP->GetObservations();
                for(map<KeyFrame*,size_t>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                    keyframeCounter[it->first]++;
            }
            else
            {
                mCurrentFrame.mvpMapPoints[i]=NULL;
            }
        }
    }

    if(keyframeCounter.empty())
        return;

    int max=0;
    KeyFrame* pKFmax= static_cast<KeyFrame*>(NULL);

    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    for(map<KeyFrame*,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
    {
        KeyFrame* pKF = it->first;

        if(pKF->isBad())
            continue;

        if(it->second>max)
        {
            max=it->second;
            pKFmax=pKF;
        }

        mvpLocalKeyFrames.push_back(it->first);
        pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
    }


    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        // Limit the number of keyframes
        if(mvpLocalKeyFrames.size()>80)
            break;

        KeyFrame* pKF = *itKF;

        const vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

        for(vector<KeyFrame*>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
        {
            KeyFrame* pNeighKF = *itNeighKF;
            if(!pNeighKF->isBad())
            {
                if(pNeighKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        const set<KeyFrame*> spChilds = pKF->GetChilds();
        for(set<KeyFrame*>::const_iterator sit=spChilds.begin(), send=spChilds.end(); sit!=send; sit++)
        {
            KeyFrame* pChildKF = *sit;
            if(!pChildKF->isBad())
            {
                if(pChildKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pChildKF);
                    pChildKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        KeyFrame* pParent = pKF->GetParent();
        if(pParent)
        {
            if(pParent->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
            {
                mvpLocalKeyFrames.push_back(pParent);
                pParent->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                break;
            }
        }

    }

    if(pKFmax)
    {
        mpReferenceKF = pKFmax;
        mCurrentFrame.mpReferenceKF = mpReferenceKF;
    }
}

void Tracking::UpdateLocalPointsBack()
{
    mvpLocalMapPointsBack.clear();

    for (vector<KeyFrame *>::const_iterator itKF = mvpLocalKeyFramesBack.begin(), itEndKF = mvpLocalKeyFramesBack.end(); itKF != itEndKF; itKF++)
    {
        KeyFrame *pKF = *itKF;
        const vector<MapPoint *> vpMPs = pKF->GetMapPointMatches();

        for (vector<MapPoint *>::const_iterator itMP = vpMPs.begin(), itEndMP = vpMPs.end(); itMP != itEndMP; itMP++)
        {
            MapPoint *pMP = *itMP;
            if (!pMP)
                continue;
            if (pMP->mnTrackReferenceForFrame == mCurrentFrameBack.mnId)
                continue;
            if (!pMP->isBad())
            {
                mvpLocalMapPointsBack.push_back(pMP);
                pMP->mnTrackReferenceForFrame = mCurrentFrameBack.mnId;
            }
        }
    }
}

void Tracking::UpdateLocalKeyFramesBack()
{
    // Each map point vote for the keyframes in which it has been observed
    map<KeyFrame *, int> keyframeCounter;
    for (int i = 0; i < mCurrentFrameBack.N; i++)
    {
        if (mCurrentFrameBack.mvpMapPoints[i])
        {
            MapPoint *pMP = mCurrentFrameBack.mvpMapPoints[i];
            if (!pMP->isBad())
            {
                const map<KeyFrame *, size_t> observations = pMP->GetObservations();
                for (map<KeyFrame *, size_t>::const_iterator it = observations.begin(), itend = observations.end(); it != itend; it++)
                    keyframeCounter[it->first]++;
            }
            else
            {
                mCurrentFrameBack.mvpMapPoints[i] = NULL;
            }
        }
    }

    if (keyframeCounter.empty())
        return;

    int max = 0;
    KeyFrame *pKFmax = static_cast<KeyFrame *>(NULL);

    mvpLocalKeyFramesBack.clear();
    mvpLocalKeyFramesBack.reserve(3 * keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    for (map<KeyFrame *, int>::const_iterator it = keyframeCounter.begin(), itEnd = keyframeCounter.end(); it != itEnd; it++)
    {
        KeyFrame *pKF = it->first;

        if (pKF->isBad())
            continue;

        if (it->second > max)
        {
            max = it->second;
            pKFmax = pKF;
        }

        mvpLocalKeyFramesBack.push_back(it->first);
        pKF->mnTrackReferenceForFrame = mCurrentFrameBack.mnId;
    }

    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    for (vector<KeyFrame *>::const_iterator itKF = mvpLocalKeyFramesBack.begin(), itEndKF = mvpLocalKeyFramesBack.end(); itKF != itEndKF; itKF++)
    {
        // Limit the number of keyframes
        if (mvpLocalKeyFramesBack.size() > 80)
            break;

        KeyFrame *pKF = *itKF;

        const vector<KeyFrame *> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

        for (vector<KeyFrame *>::const_iterator itNeighKF = vNeighs.begin(), itEndNeighKF = vNeighs.end(); itNeighKF != itEndNeighKF; itNeighKF++)
        {
            KeyFrame *pNeighKF = *itNeighKF;
            if (!pNeighKF->isBad())
            {
                if (pNeighKF->mnTrackReferenceForFrame != mCurrentFrameBack.mnId)
                {
                    mvpLocalKeyFramesBack.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame = mCurrentFrameBack.mnId;
                    break;
                }
            }
        }

        const set<KeyFrame *> spChilds = pKF->GetChilds();
        for (set<KeyFrame *>::const_iterator sit = spChilds.begin(), send = spChilds.end(); sit != send; sit++)
        {
            KeyFrame *pChildKF = *sit;
            if (!pChildKF->isBad())
            {
                if (pChildKF->mnTrackReferenceForFrame != mCurrentFrameBack.mnId)
                {
                    mvpLocalKeyFramesBack.push_back(pChildKF);
                    pChildKF->mnTrackReferenceForFrame = mCurrentFrameBack.mnId;
                    break;
                }
            }
        }

        KeyFrame *pParent = pKF->GetParent();
        if (pParent)
        {
            if (pParent->mnTrackReferenceForFrame != mCurrentFrameBack.mnId)
            {
                mvpLocalKeyFramesBack.push_back(pParent);
                pParent->mnTrackReferenceForFrame = mCurrentFrameBack.mnId;
                break;
            }
        }
    }

    if (pKFmax)
    {
        mpReferenceKFBack = pKFmax;
        mCurrentFrameBack.mpReferenceKF = mpReferenceKFBack;
    }
}

bool Tracking::Relocalization()
{
    // Compute Bag of Words Vector
    mCurrentFrame.ComputeBoW();

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    // cout <<"Tracking::Relocalization() : checkpoint1" << endl;
    vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);
    if(vpCandidateKFs.empty()){
        return false;
    }
    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    // cout <<"Tracking::Relocalization() : checkpoint2, nKFs = " << nKFs << endl;
    ORBmatcher matcher(0.75,true);

    vector<PnPsolver*> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates=0;

    cout <<"Tracking::Relocalization() : Try to localize in " << nKFs << " KeyFrames." << endl;
    for(int i=0; i<nKFs; i++)
    {
        KeyFrame* pKF = vpCandidateKFs[i];
        if(pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            int nmatches = matcher.SearchByBoW(pKF,mCurrentFrame,vvpMapPointMatches[i]);
            cout << "nmatches=" << nmatches << "********************" << endl;
            if(nmatches<15) // 15
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                PnPsolver* pSolver = new PnPsolver(mCurrentFrame,vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(0.99,10,300,4,0.5,5.991);
                vpPnPsolvers[i] = pSolver;
                nCandidates++;
                // cout <<"Tracking::Relocalization() : checkpoint2.1, nCandidates = " << nCandidates << " added to the pSolver." << endl;
            }
        }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    // cout <<"Tracking::Relocalization() : checkpoint3" << endl;
    bool bMatch = false;
    ORBmatcher matcher2(0.9,true);

    while(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nKFs; i++)
        {
            if(vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            PnPsolver* pSolver = vpPnPsolvers[i];
            cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if(!Tcw.empty())
            {
                Tcw.copyTo(mCurrentFrame.mTcw);

                set<MapPoint*> sFound;

                const int np = vbInliers.size();

                for(int j=0; j<np; j++)
                {
                    if(vbInliers[j])
                    {
                        mCurrentFrame.mvpMapPoints[j]=vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else
                        mCurrentFrame.mvpMapPoints[j]=NULL;
                }

                int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                if(nGood<10)
                    continue;

                for(int io =0; io<mCurrentFrame.N; io++)
                    if(mCurrentFrame.mvbOutlier[io])
                        mCurrentFrame.mvpMapPoints[io]=static_cast<MapPoint*>(NULL);

                // If few inliers, search by projection in a coarse window and optimize again
                if(nGood<50)
                {
                    int nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,10,100);

                    if(nadditional+nGood>=50)
                    {
                        nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        if(nGood>30 && nGood<50)
                        {
                            sFound.clear();
                            for(int ip =0; ip<mCurrentFrame.N; ip++)
                                if(mCurrentFrame.mvpMapPoints[ip])
                                    sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                            nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,3,64);

                            // Final optimization
                            if(nGood+nadditional>=50)
                            {
                                nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                for(int io =0; io<mCurrentFrame.N; io++)
                                    if(mCurrentFrame.mvbOutlier[io])
                                        mCurrentFrame.mvpMapPoints[io]=NULL;
                            }
                        }
                    }
                }


                // If the pose is supported by enough inliers stop ransacs and continue
                if(nGood>=50)
                {
                    bMatch = true;
                    break;
                }
            }
        }
    }
    // cout <<"Tracking::Relocalization() : checkpoint4 Solver finished, current pose mCurrentFrame.mTcw = " << mCurrentFrame.mTcw << endl;

    if(!bMatch)
    {
        // cout <<"Tracking::Relocalization() : checkpoint5 !bMatch" << endl;
        return false;
    }
    else
    {
        // cout <<"Tracking::Relocalization() : checkpoint5 true, mnLastRelocFrameId = " << mnLastRelocFrameId << endl;
        mnLastRelocFrameId = mCurrentFrame.mnId;
        return true;
    }

}

void Tracking::Reset()
{

    cout << "System Reseting" << endl;
    if(mpViewer)
    {
        mpViewer->RequestStop();
        while(!mpViewer->isStopped())
            usleep(3000);
    }

    // Reset Local Mapping
    cout << "Reseting Local Mapper...";
    mpLocalMapper->RequestReset();
    cout << " done" << endl;

    if(mSensor==System::TWOSTEREO){
        // Reset Local Mapping Back
        cout << "Reseting Local Mapper...";
        mpLocalMapperBack->RequestReset();
        cout << " done" << endl;
    }

    // Reset Loop Closing
    cout << "Reseting Loop Closing...";
    mpLoopClosing->RequestReset();
    cout << " done" << endl;

    // Clear BoW Database
    cout << "Reseting Database...";
    mpKeyFrameDB->clear();
    cout << " done" << endl;

    // Clear Map (this erase MapPoints and KeyFrames)
    mpMap->clear();

    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    mState = NO_IMAGES_YET;

    if(mpInitializer)
    {
        delete mpInitializer;
        mpInitializer = static_cast<Initializer*>(NULL);
    }

    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();

    if(mpViewer)
        mpViewer->Release();
}

void Tracking::ChangeCalibration(const string &strSettingPath)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    Frame::mbInitialComputations = true;
}

void Tracking::InformOnlyTracking(const bool &flag)
{
    mbOnlyTracking = flag;
}

void Tracking::SetCameraBackTcw(Frame &mCurrentFrame, Frame &mCurrentFrameBack, const float CamSpacing)
{
    const cv::Mat FrontTcw = mCurrentFrame.mTcw.clone();
    if (FrontTcw.cols == 0){
        return;
    }

    // For Two Stereo camera back
    const cv::Mat Rotate180X = (cv::Mat_<float>(4, 4) <<          //绕x轴旋转180
                                        1, 0, 0, 0,
                                        0,-1, 0, 0,
                                        0, 0,-1, 0,
                                        0, 0, 0, 1);
    const cv::Mat Rotate180Y = (cv::Mat_<float>(4, 4) <<          //绕y轴旋转180
                                       -1, 0, 0, 0,
                                        0, 1, 0, 0,
                                        0, 0, -1, 0,
                                        0, 0, 0, 1);
    const cv::Mat Rotate180Z = (cv::Mat_<float>(4, 4) <<          //绕z轴旋转180
                                       -1, 0, 0, 0,
                                        0,-1, 0, 0,
                                        0, 0, 1, 0,
                                        0, 0, 0, 1);
    const cv::Mat MoveZ = (cv::Mat_<float>(4, 4) << 
                                        1, 0, 0, 0,
                                        0, 1, 0, 0,
                                        0, 0, 1,-CamSpacing,
                                        0, 0, 0, 1);
    const cv::Mat BackTcw = MoveZ * Rotate180Y * FrontTcw;
    mCurrentFrameBack.SetPose(BackTcw);
}


} //namespace ORB_SLAM