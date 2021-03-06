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


#ifndef VIEWER_H
#define VIEWER_H

#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Tracking.h"
#include "System.h"

#include <mutex>



namespace ORB_SLAM2
{

class Tracking;
class FrameDrawer;
class MapDrawer;
class System;

class Viewer
{
public:
    Viewer(System* pSystem, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Tracking *pTracking, const string &strSettingPath, bool mbReuseMap);

    // Main thread function. Draw points, keyframes, the current camera pose and the last processed
    // frame. Drawing is refreshed according to the camera fps. We use Pangolin.
    void Run();

    void RequestFinish();

    void RequestStop();

    bool isFinished();

    bool isStopped();

    void Release();

//    static bool test; //Test variable that we accessed
    static char test; //Test variable that we accessed
    static vector<bool> ButtonState;
    static bool RecordData; //Test variable that we accessed
    static bool RecordCameraData; //Test variable that we accessed

    static double SlowSpeed;
    static int mFrameMax;
    static int mCurrentFrame;
    static int mFrameSlider;
    static bool ChangeVideo;
    static bool SaveMap;
    static bool SaveMapTag;
    static bool LoadMap;
    static bool LoadMapTag;
    static string mapfileC1;
    static string mapfileC2;


	//was static
//    static bool get(); //Getter method
//    static char get(); //Getter method

    static vector<bool> get(); //Getter method
    static bool getRecordData(); //Getter method
    static bool getRecordCameraData(); //Getter method

    static double getSlowSpeed();
    static void putFrameMax(int FrameMax);
    static void putCurrentFrame(int CurrentFrame);
    static int getFrameSlider();
    static bool getChangeVideo();
    static bool getSaveMap();
    static bool getLoadMap();
    static string passfSettings1();
    static string passfSettings2();



private:

    bool Stop();

    System* mpSystem;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;
    Tracking* mpTracker;

    // 1/fps in ms
    double mT;
    float mImageWidth, mImageHeight;

    float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    bool mbStopped;
    bool mbStopRequested;
    bool mbReuseMap;
    std::mutex mMutexStop;

};

}


#endif // VIEWER_H
	

