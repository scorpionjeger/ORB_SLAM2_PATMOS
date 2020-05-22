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

#ifndef FRAMEDRAWER_H
#define FRAMEDRAWER_H

#include "Tracking.h"
#include "MapPoint.h"
#include "Map.h"
#include<pangolin/pangolin.h>//qwq

#include <iostream>//qwq
#include <fstream>



#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include<mutex>


namespace ORB_SLAM2
{

class Tracking;
class Viewer;

class FrameDrawer
{
public:
    FrameDrawer(Map* pMap, bool bReuseMap=false);

    // Update info from the last processed frame.
    void Update(Tracking *pTracker);
    Tracking *pTrackerCrayCray;///qwq
    // Draw last processed frame.
    cv::Mat DrawFrame();
    pangolin::OpenGlMatrix TwcClone; //qwq  
    //cv::vector<
    std::ofstream myfile;//qwq
    bool  WriteDataFile;
    bool  WriteDataFile2; 

    void MessWithMap();//qwq
    int mnTracked;//qwq



    static double timeStampFollow1;//qwq
    static double timeStampFollow2;//qwq


protected:

    void DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText);

    // Info of the frame to be drawn
    cv::Mat mIm;
    cv::Mat fmRGB;//qwq
    double timestampHere;//qwq
    int N;
    vector<cv::KeyPoint> mvCurrentKeys;
    vector<bool> mvbMap, mvbVO;
    vector<int> CountVect;//qwq
    vector<MapPoint*> MapPointVec;//qwq
    //vector<6> vectorBundel; //qwq
    bool mbOnlyTracking;
    int mnTrackedVO;//qwq
    //int mnTracked, mnTrackedVO;//qwq
    vector<cv::KeyPoint> mvIniKeys;
    vector<int> mvIniMatches;
    int mState;
    ///qwq
    cv::Mat mmK;
    cv::Mat mmDistCoef;
    float mmbf;


    Map* mpMap;

    std::mutex mMutex;
	





};

} //namespace ORB_SLAM

#endif // FRAMEDRAWER_H
