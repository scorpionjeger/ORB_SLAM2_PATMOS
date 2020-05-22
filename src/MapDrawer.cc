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

#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>
#include <mutex>

namespace ORB_SLAM2
{


MapDrawer::MapDrawer(Map* pMap, const string &strSettingPath):mpMap(pMap)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
    mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
    mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
    mPointSize = fSettings["Viewer.PointSize"];
    mCameraSize = fSettings["Viewer.CameraSize"];
    mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];

}

void MapDrawer::DrawMapPoints()
{
    const vector<MapPoint*> &vpMPs = mpMap->GetAllMapPoints();
    const vector<MapPoint*> &vpRefMPs = mpMap->GetReferenceMapPoints();

    bool UseColor=false;//qwq


///////////////////////////////////////////////////qwq

//    if(!myfile.is_open()) 
//{
//     myfile.open("Data/example.txt");
//     string myFileheader="Time;";
     //myFileheader+="index seed;true index;";
     //myfile<<myFileheader<<endl;
//}


///////////////////////////////////////////qwq


    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());



    if(1==1)
{

    glPointSize(mPointSize);
    glBegin(GL_LINES);//qwq  can we put the normals on the map
    glColor3f(1.0,0.0,0.0);

    glVertex3f(1.01,0.0,0.0);
    glVertex3f(-1.01,0.0,0.0);


    glColor3f(0.0,1.0,0.0);


    glVertex3f(0.0,1.01,0.0);
    glVertex3f(0.0,-1.01,0.0);


    glColor3f(0.0,0.0,1.0);

    glVertex3f(0.0,0.0,1.01);
    glVertex3f(0.0,0.0,-1.01);



//    glColor3f(0.0,0.0,1.0);

//    glVertex3f(0.0,0.0,1.01);
//    glVertex3f(0.0,0.0,-1.01);


if(1==2)
{
    glColor3f(0.0,1.0,1.0);

 
    float norm=0.2/sqrt(pow(TwcClone.m[8],2)+pow(TwcClone.m[9],2)+pow(TwcClone.m[10],2));
    glVertex3f(TwcClone.m[12],TwcClone.m[13],TwcClone.m[14]);
//    glVertex3f(TwcClone.m[12]+.2,TwcClone.m[13]+.2,TwcClone.m[14]+.2);
    glVertex3f(TwcClone.m[12]+TwcClone.m[8]*norm,TwcClone.m[13]+TwcClone.m[9]*norm,TwcClone.m[14]+TwcClone.m[10]*norm);

}


    glEnd();



}




    if(1==1)
{

    if(vpMPs.empty())
        return;

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);

    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {

//	if(i>1000 && i<1100)////qwq
//   	   glColor3f(0.0,1.0,0.0);
//	else if(i>2000 && i<2100)
//   	   glColor3f(0.0,1.0,1.0);
//	else if(i>3000 && i<3100)
//   	   glColor3f(0.0,0.0,1.0);
//	else
   	   //glColor3f(0.0,0.0,0.0);
   	   glColor3f(0.0,0.0,0.0);
////////////////////////////////////qwq
	

        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
//        if(vpMPs[i]->isBad())////qwq
//            continue;

        cv::Mat pos = vpMPs[i]->GetWorldPos();



if(UseColor)
{
	cv::Mat Col= vpMPs[i]->GetRgbVal();//qwq	
	if(!Col.empty())
{
	glColor3f(Col.at<float>(0,0),Col.at<float>(1,0),Col.at<float>(2,0));
}
	else
{
	
	glColor3f(1.0,0.0,0.0);
}

}//UseColor

        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
	//myfile<<i<<";"<<pos.at<float>(0)<<";"<<pos.at<float>(1)<<";"<<pos.at<float>(2)<<";"<<vpMPs.size()<<";"<<vpMPs[i]->Observations()<<";"<<vpMPs[i]->mnId<<";"<<vpMPs[i]->nNextId<<endl;
    }
    //myfile<<vpMPs.size()<<endl;//qwq

    //cout <<"vpMPs.size() "<<vpMPs.size()<<"vpRefMPs.size() "<<vpRefMPs.size()<<" ";
    glEnd();
}


	if(1==1)//qwq
{
    glPointSize(mPointSize);
    glBegin(GL_POINTS); //qwq cut
    //glBegin(GL_LINES);//qwq  can we put the normals on the map
    glColor3f(1.0,0.0,0.0);


    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {

   



        if((*sit)->isBad())
            continue;
        cv::Mat pos = (*sit)->GetWorldPos();
////qwq

	//cv::Mat normPos= (*sit)->GetNormal();//qwq




	if(UseColor)
{
	cv::Mat Col= (*sit)->GetRgbVal();//qwq	
	if(!Col.empty())
{


	if(Col.at<float>(0,0)==0.0 && Col.at<float>(0,0)==0.0 && Col.at<float>(0,0)==0.0)
  {

	glColor3f(0.0,1.0,0.0);
  }

	else
  {
	glColor3f(Col.at<float>(0,0),Col.at<float>(1,0),Col.at<float>(2,0));
  }

}
	else
{
	
	glColor3f(1.0,0.0,0.0);
}

}///UseColor
/////qwq



        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
        //glVertex3f(pos.at<float>(0)-normPos.at<float>(0)/10,pos.at<float>(1)-normPos.at<float>(1)/10,pos.at<float>(2)-normPos.at<float>(2)/10);


    }



    glEnd();

}

}

void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph)
{
    const float &w = mKeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;

    const vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();

    if(bDrawKF)
    {
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame* pKF = vpKFs[i];
            cv::Mat Twc = pKF->GetPoseInverse().t();

            glPushMatrix();

            glMultMatrixf(Twc.ptr<GLfloat>(0));

            glLineWidth(mKeyFrameLineWidth);
            glColor3f(0.0f,0.0f,1.0f);
            glBegin(GL_LINES);
            glVertex3f(0,0,0);
            glVertex3f(w,h,z);
            glVertex3f(0,0,0);
            glVertex3f(w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,h,z);

            glVertex3f(w,h,z);
            glVertex3f(w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(-w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(w,h,z);

            glVertex3f(-w,-h,z);
            glVertex3f(w,-h,z);
            glEnd();

            glPopMatrix();
        }
    }

    if(bDrawGraph)
    {
        glLineWidth(mGraphLineWidth);
        glColor4f(0.0f,1.0f,0.0f,0.6f);
        glBegin(GL_LINES);

        for(size_t i=0; i<vpKFs.size(); i++)
        {
            // Covisibility Graph
            const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
            cv::Mat Ow = vpKFs[i]->GetCameraCenter();


/*
            if(!vCovKFs.empty())
            {
                for(vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
                {
                    if((*vit)->mnId<vpKFs[i]->mnId)
                        continue;
                    cv::Mat Ow2 = (*vit)->GetCameraCenter();
                    glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                    glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
                }
            }

*/



            // Spanning tree
            KeyFrame* pParent = vpKFs[i]->GetParent();
            if(pParent)
            {
                cv::Mat Owp = pParent->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owp.at<float>(0),Owp.at<float>(1),Owp.at<float>(2));
            }



/*
            // Loops
            set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
            for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
            {
                if((*sit)->mnId<vpKFs[i]->mnId)
                    continue;
                cv::Mat Owl = (*sit)->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owl.at<float>(0),Owl.at<float>(1),Owl.at<float>(2));
            }

*/

        }

        glEnd();
    }
}

void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    const float &w = mCameraSize;
    const float h = w*0.75;
    const float z = w*0.6;

	//////////////////////////qwq
    //cout<<"TWC "<<Twc.m[12]<<" "<<Twc.m[13]<<" "<<Twc.m[14]<<" "<<endl;


    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

    glLineWidth(mCameraLineWidth);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}


void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone();
}

void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
{
    if(!mCameraPose.empty())
    {
        cv::Mat Rwc(3,3,CV_32F);
        cv::Mat twc(3,1,CV_32F);
        {
            unique_lock<mutex> lock(mMutexCamera);
            Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
            twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
        }

        M.m[0] = Rwc.at<float>(0,0);
        M.m[1] = Rwc.at<float>(1,0);
        M.m[2] = Rwc.at<float>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Rwc.at<float>(0,1);
        M.m[5] = Rwc.at<float>(1,1);
        M.m[6] = Rwc.at<float>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Rwc.at<float>(0,2);
        M.m[9] = Rwc.at<float>(1,2);
        M.m[10] = Rwc.at<float>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = twc.at<float>(0);
        M.m[13] = twc.at<float>(1);
        M.m[14] = twc.at<float>(2);
        M.m[15]  = 1.0;
    }
    else
        M.SetIdentity();
}

} //namespace ORB_SLAM
