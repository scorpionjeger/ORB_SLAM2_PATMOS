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


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<stdio.h>
#include<stdlib.h>

#include<opencv2/core/core.hpp>

//#include<opencv2/core.hpp>



#include<System.h> //Included System.h to access Viewer methods

#include <unistd.h>//qwq added 
#include <boost/algorithm/string.hpp>


using namespace std;
//using namespace boost;


////////////////////  THis should just be loading a single straight movie





int main(int argc, char **argv)
{
cout<<"program started";
  //  ORB_SLAM2::Viewer v1();



 

    if(argc < 5 || argc >6)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }




    string strFile = string(argv[3]);///straigth file name e 
    string strFile2=""; 
    cout<<"argc "<<argc<<endl;

    cout<<"argv[2] "<<string(argv[2])<<endl;//YAML
    cv::FileStorage fSettings(string(argv[2]), cv::FileStorage::READ);
    cout<<"fsetings trial "<<string(fSettings["Map.mapfile"])<<endl;
    string kftfile = fSettings["Map.KFTfile"];
    int FrameInital = fSettings["Map.FirstFrame"];
    if(argc == 6){

    strFile2 = string(argv[4]);///straigth file name


}



    std::cout << "video file "<<strFile << std::endl;


    // Create SLAM system. It initializes all system threads and gets ready to process frames.
//    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true, (bool)atoi(argv[5]));


    // Main loop
    cv::Mat im;
    cv::Mat imr;

   // assumeing there are two videos.   Should make it so can use one or two
    cv::VideoCapture cap0(strFile);

//    cv::VideoCapture* cap1=NULL;

//    cv::VideoCapture* cap2=NULL;
 
//    if(argc ==6){

    cv::VideoCapture cap1(strFile);

    cv::VideoCapture cap2(strFile2);

//}

    if (!cap0.isOpened())
    {
        std::cout << "!!! Failed to open file: " << argv[1] << std::endl;
        return -1;
    }
    else
    {
        std::cout << "succeded in opening video " << std::endl;
    }   

//    if (argc==5){

//    cv::VideoCapture cap2(strFile2);
 


    if (!cap2.isOpened())
    {
        std::cout << "!!! Failed to open file: " << argv[1] << std::endl;
        return -1;
    }
    else
    {
        std::cout << "succeded in opening video " << std::endl;
    }  


    cap0=cap1;


//}



    //the width and basic parameters for cropping.
    
    cap0.read(imr);
    cout<<"we made it here yes"<<endl;
    cout << "Width : " << imr.size().width << endl;
    cout << "Height: " << imr.size().height << endl;    
    float imWidth=imr.size().width;
    float imHeight=imr.size().height;

/////////////////////////////

    //float baseWidth=640;///regular
    //float baseHeight=480;


    float baseWidth=320;//the peacocks
    float baseHeight=212;

/////////////////////////////

    string KeyInput;
    string SkippingFileFlag;
    bool forward=true;
    bool skipforward=false;
    int frameskip=5;
    bool CutOutJunk=false;
    vector<int> SkipBeginVector;
    vector<int> SkipEndVector;
    bool CropVideo=false;
    bool CropVideoH=false;
    

    //Rect(int x, int y, int width, int height) 
    int cropx;
    int cropy;
    int cropWidth;
    int cropHeight;
    ofstream SkipFile;
    ifstream SkipFileR;
    int MatchingNum=0;
    int MatchingNumMin=40;
    int MatchingNumOk=130;
    bool MatchingStatePanic=false;
    bool rememberskipforward=false;
    int MatchingStatePanicind=0;
    int MatchingStatePanicindT=250;
    bool repeatMode=false;
    int  repeatModeStep=4;
    int repeatModeInc=0;
    float TimetoRest=0;
    int FrameSlider=0;
    int FrameSliderMemory=0;
    bool ChangeVideo=false;
    bool SaveMap=false;
    bool LoadMap=false;
    int FrameNew=0;
    bool ManChoosingFrameFlag=false;










    //initial frame number

    //int FrameInital=2;

    //cout<<"Initial frame"<<endl;

    //cin >> FrameInital;////                                                  /// //inputting the inital frame
    
    cout <<" Initial frame="<<FrameInital<<endl;





///                                                                         /// using a utitlity to skip bad parts of the video


 //   cout <<"Use skipping file or make skipping file. y use skipping file, m make it, n ignore"<< endl;

 //   cin >> SkippingFileFlag;



    SkippingFileFlag="n";
    if(SkippingFileFlag=="y") {

    cout<<"using file"<<endl;

        

    string delimiter1=".";
    vector<string> filedirs;
    boost::split(filedirs,strFile,boost::is_any_of("/"));    
    size_t found = filedirs[filedirs.size()-1].find_last_of(delimiter1);
    string trakingFileName=filedirs[filedirs.size()-2]+"_"+filedirs[filedirs.size()-1].substr(0,found);
    cout<<trakingFileName<<endl;





    SkipFileR.open("Data/"+trakingFileName+".txt");
    if(SkipFileR.good()){

    string s0;
    getline(SkipFileR,s0);
    getline(SkipFileR,s0);
    stringstream ss;    
    ss << s0;
    string garbage;
    ss >> garbage;
    ss >> FrameInital;
    cout <<"The initial frame is now "<< FrameInital<<endl;
    


    while(!SkipFileR.eof())
    {
        string s;
        getline(SkipFileR,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            int t1;
            ss >> t1;
            SkipBeginVector.push_back(t1);
            ss >> t1;
            SkipEndVector.push_back(t1);

        }
    }


    }



}

    else if(SkippingFileFlag=="m") {
    cout<<"making file "<<endl;


    //save this where???
    //strFile
    
                                                                              ////// example of parsing strFile....

    


    string delimiter1=".";

    //size_t found = strFile.find_last_of(delimiter1);

    //cout<< found <<" found"<<endl;

    //string token1=strFile.substr(found,strFile.find(delimiter1));

    //string token2=strFile.substr(found-1,strFile.find(delimiter1));

    vector<string> filedirs;
    boost::split(filedirs,strFile,boost::is_any_of("/"));
    
    size_t found = filedirs[filedirs.size()-1].find_last_of(delimiter1);

    //string token1=strFile.substr(found-1);

    //string token2=strFile.substr(found);

    string trakingFileName=filedirs[filedirs.size()-2]+"_"+filedirs[filedirs.size()-1].substr(0,found);
    cout<<trakingFileName<<endl;



    SkipFile.open("Data/"+trakingFileName+".txt");
    SkipFile<<"Skipping bad frame segments file header"<<endl;
    SkipFile<<"FrameInital "<<FrameInital<<endl;




}

    else {
    cout<<"ignoring file"<<endl;
}



////////                                                               /////////////   fetting the file here
    cap0.set(CV_CAP_PROP_POS_FRAMES,FrameInital-1);
    






    if (baseWidth*(imHeight/baseHeight)<imWidth)
	{
    
	cropy=0;
	cropHeight=imHeight;
	cropx=int((imWidth-baseWidth*(imHeight/baseHeight))/2);
	cropWidth=int(baseWidth*(imHeight/baseHeight));

	}


    double fps = cap0.get(CV_CAP_PROP_FPS);
    cout << "Frames per second using video.get(CV_CAP_PROP_FPS) : " << fps << endl;


    double TotalFrames=cap0.get(CV_CAP_PROP_FRAME_COUNT);
    cout << "Total number of frames : " << TotalFrames << endl;
    



    double CurrentFrame=cap0.get(CV_CAP_PROP_POS_FRAMES);
    cout << " Current Frame: " << CurrentFrame << endl;



    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << TotalFrames << endl << endl;



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//////                                          The main iteration





    for(int ni=FrameInital; ni<TotalFrames*200; ni++)// making this issentially infinite
    {



	ChangeVideo=ORB_SLAM2::Viewer::getChangeVideo();

//        if(argc == 6){



    cap0=cap1;



	if(ChangeVideo){

	cap0 = cap2;

	}

	else{

	cap0 = cap1;

	}
//}




	////  load specific frame and movie:
	CurrentFrame=cap0.get(CV_CAP_PROP_POS_FRAMES);


	if(ni-FrameInital==4){

	FrameSliderMemory=ORB_SLAM2::Viewer::getFrameSlider();
	
	

	}

	else if (ni-FrameInital>4){


	FrameSlider=ORB_SLAM2::Viewer::getFrameSlider();

	if(FrameSlider != FrameSliderMemory){


	cap0.set(CV_CAP_PROP_POS_FRAMES,FrameSlider);
	CurrentFrame=cap0.get(CV_CAP_PROP_POS_FRAMES);
	
	}


	FrameSliderMemory=FrameSlider;

    	}








	if(forward==false){

	//CurrentFrame=cap0.get(CV_CAP_PROP_POS_FRAMES);

	if(skipforward==false){

	cap0.set(CV_CAP_PROP_POS_FRAMES,CurrentFrame-2);


}
	else if (skipforward==true){


	cap0.set(CV_CAP_PROP_POS_FRAMES,CurrentFrame-frameskip-1);
}
	
	

}


	else{

	if(skipforward){

	//CurrentFrame=cap0.get(CV_CAP_PROP_POS_FRAMES);
        cap0.set(CV_CAP_PROP_POS_FRAMES,CurrentFrame+frameskip);



}

}




	if(repeatMode==true){

		if(repeatModeInc % repeatModeStep!=0){
	
	        cap0.set(CV_CAP_PROP_POS_FRAMES,CurrentFrame-1);

	        repeatModeInc+=1;

		}

		else {

	        repeatModeInc+=1;

		}

	}





    SaveMap=ORB_SLAM2::Viewer::getSaveMap();


    if(SaveMap){


     	usleep(1*1e6);	

    cout<<"Save Map Pressed"<<endl;
        SLAM.SaveMapFunction();
     	usleep(1*1e6);	

}



    LoadMap=ORB_SLAM2::Viewer::getLoadMap();


    if(LoadMap){

    cout<<"Load Map Pressed"<<endl;
//        SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
       SLAM.LoadMapFunction();


}






    ORB_SLAM2::Viewer::putFrameMax(TotalFrames);
    ORB_SLAM2::Viewer::putCurrentFrame(CurrentFrame);
















#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif


	for(int SFI=0;SFI<SkipBeginVector.size();SFI+=1){

		if(CurrentFrame>SkipBeginVector[SFI] && CurrentFrame <SkipEndVector[SFI]){	

		CropVideo=true;
		break;
		}
		CropVideo=false;

	}

	if(CropVideo==true && CropVideoH==false){


	SLAM.ActivateLocalizationMode();

	cout<<"turn map on"<<endl;
	CutOutJunk=true;
	CropVideoH=true;

	}
	else if(CropVideo==false && CropVideoH==true){

	SLAM.DeactivateLocalizationMode();
	cout<<"turn map off"<<endl;
 	CutOutJunk=false;
	CropVideoH=false;
	}


        if (!cap0.read(imr)) {   
           break;
}

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif


	
        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
 	
	//cout<<ORB_SLAM2::Viewer::getSlowSpeed()<<endl;

	//vector< mButtonState=ORB_SLAM2::Viewer::getSlowSpeed();
     	usleep((1-ORB_SLAM2::Viewer::getSlowSpeed())*1e6);	



 	cout << "going forward "<< forward<<", Frame number "<<CurrentFrame<<" mapping on "<< CutOutJunk <<endl;
	

	//cin.ignore();
	//KeyInput=cin.get();
        //KeyInput="";
	//getline(cin,KeyInput);
	//cout<< KeyInput<<endl;

        char c=1;

        //cout<<ORB_SLAM2::Viewer::get()<<" THIS IS WORKING"; //This is where we accessed the variable

	//c=ORB_SLAM2::Viewer::get();

	vector<bool> mButtonState=ORB_SLAM2::Viewer::get();
                                                    /////////////////////////////   wait keys or other inputs
	//c=KeyInput;

        //c=cv::waitKey(10);


/*
        if(c==32){// hit the space bar
        cout<<c<< " Waitkey is space"<<endl;
        //c=cv::waitKey(100);
        cout<<"press any key to continue"<<endl;
	cin.ignore();
	cin>> KeyInput;
//	KeyInput=cin.get();
//	getline(cin,KeyInput);



		if(KeyInput=="z"){

			if(CutOutJunk==false){

			SLAM.ActivateLocalizationMode();
			SkipFile<<CurrentFrame;
			CutOutJunk=!CutOutJunk;
			cout<<"deactivate map"<<endl;
			}
			else{

			SLAM.DeactivateLocalizationMode();						
			SkipFile<<" "<<CurrentFrame<<endl;
			CutOutJunk=!CutOutJunk;
			cout<<"activate map"<<endl;	
			}	

		}


	
	}


*/


/*
	if(KeyInput=="m" || c==109){//109

	skipforward=!skipforward;
	cout<<"changed to faster gait"<<endl;

	}


	
	if(KeyInput=="r" || c==114){//114

	forward=!forward;
	cout<<"has changed directions"<<endl;
	}


*/



////////////// reverse

	if(mButtonState[2]==true){

	forward=false;
	}
	else{

	forward=true;

	}




//////////// skip forward

	if(mButtonState[3]==true){

	skipforward=true;
	}
	else{

	skipforward=false;

	}



//////////// choose another frame

	if(mButtonState[5]==true && ManChoosingFrameFlag==false){

    	cout<<"Type another frame number and press enter"<<endl;

    	cin >>FrameNew;////  

	cap0.set(CV_CAP_PROP_POS_FRAMES,FrameNew);
	CurrentFrame=cap0.get(CV_CAP_PROP_POS_FRAMES);
	ManChoosingFrameFlag=true;

	}
	else if(mButtonState[5]==false){
	ManChoosingFrameFlag=false;
	}

////////////////// pause

	if(mButtonState[1]==true){

	while(mButtonState[1]==true){

	mButtonState=ORB_SLAM2::Viewer::get();

	}

	}



/*
	if(KeyInput=="d" || c==100){//100
	SLAM.DeactivateLocalizationMode();
	cout<<"turn map on"<<endl;

	}

	if(KeyInput=="a" || c==97){//97
	SLAM.ActivateLocalizationMode();
	cout<<"turn map off"<<endl;
	}	


*/


	if(KeyInput=="g" || KeyInput=="h"){//103  104

	
//	SLAM.ParrMessWithMap(KeyInput);
	cout<<"Delete things"<<endl;

	}	

	if(c==103){//103  104

	KeyInput="g";
//	SLAM.ParrMessWithMap(KeyInput);
	KeyInput="";

	}



	if(c==104){//103  104

	KeyInput="h";
	
//	SLAM.ParrMessWithMap(KeyInput);
	KeyInput="";

	}





/*
	if(KeyInput=="q" || c==113){//113
	break;

	}

	if(KeyInput=="e" || c==101){//101
	SLAM.Reset();
	cout<<"reset"<<endl;

	}

*/



	if(mButtonState[0]==true){
	break;

	}





	if(mButtonState[4]==true){

	repeatMode=true;
	}
	else{

	repeatMode=false;
	}







	



	if(KeyInput=="i" || c==105){


    	CurrentFrame=cap0.get(CV_CAP_PROP_POS_FRAMES);
        cap0.set(CV_CAP_PROP_POS_FRAMES,CurrentFrame+100);
	cout<<"skip forward 100"<<endl;

	}	

	if(KeyInput=="o" || c==111){


    	CurrentFrame=cap0.get(CV_CAP_PROP_POS_FRAMES);
        cap0.set(CV_CAP_PROP_POS_FRAMES,CurrentFrame+500);
	cout<<"skip forward 500"<<endl;

	}


	if(KeyInput=="u" || c==117){


    	CurrentFrame=cap0.get(CV_CAP_PROP_POS_FRAMES);
        cap0.set(CV_CAP_PROP_POS_FRAMES,CurrentFrame-200);
	cout<<"skip backward 200"<<endl;

	}	

	if(KeyInput=="y" || c==121){


    	CurrentFrame=cap0.get(CV_CAP_PROP_POS_FRAMES);
        cap0.set(CV_CAP_PROP_POS_FRAMES,CurrentFrame-700);
	cout<<"skip backward 800"<<endl;

	}










	if(skipforward){

	CurrentFrame=cap0.get(CV_CAP_PROP_POS_FRAMES);
        cap0.set(CV_CAP_PROP_POS_FRAMES,CurrentFrame+frameskip);



}

	
	//CurrentFrame=cap.get(CV_CAP_PROP_POS_FRAMES);                                         //cv
        //cout << " Current Frame before: " << CurrentFrame << endl;
        // Read image from file
        //if (!cap.read(imr))             
        //    break;

	//CurrentFrame=cap.get(CV_CAP_PROP_POS_FRAMES);                                         //cv
        //cout << " Current Frame after: " << CurrentFrame << endl;

	//  this sets an roi that the image can be cropped from 
        //Rect(int x, int y, int width, int height) 
        //cv::Rect myROI(10, 10, 100, 100);
	if(1==2){


        cv::Rect myROI(cropx, cropy, cropWidth, cropHeight);                                         //cv

        //  using the ROI we can form a new image.
	cv::Mat croppedImage = imr(myROI);                                                        //cv

	//  this resizing will morph the image
	cv::resize(croppedImage, im,cv::Size(320,212));//(640,480));                               //cv


        //cv::imshow("window", im);
	}
	else
	{


	}
        //cv::imshow("window hi", imr);                                                           //cv

//        double tframe = CurrentFrame/fps;
        double tframe = CurrentFrame;

        if(imr.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << endl;
            return 1;
        }

        // Pass the image to the SLAM system


	//cout<<tframe<<endl;
        SLAM.TrackMonocular(imr,tframe);


    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    //SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM(kftfile);

    return 0;
} 
