#include <iostream>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <chrono>

#include <opencv2/core/core.hpp>

#include "orb_slam/include/System.h"
#include "orb_slam/include/FPGA.h"


#if USE_RECONOS == 1

extern "C" {
    #include "reconos.h"
    #include "reconos_app.h"
}

#endif

#define COMPILEDWITHC11



extern "C" THREAD_ENTRY(); // this is required because of the mixture of c and c++

THREAD_ENTRY()
{

    const string strVocFile = "hierundda";
    const string strSettingsFile = "daundhier";
  
    reconos_thread_create_hwt_fast((void*)0);
    reconos_thread_create_hwt_fast((void*)1);
    FPGA::FPGA_Init();


    ORB_SLAM2::System SLAM(strVocFile,strSettingsFile, ORB_SLAM2::System::STEREO,false);



    // Main loop
    cv::Mat imLeft, imRight;
    while(true)


        // Read left and right images from file
        //imLeft = cv::imread(strImageLeft,CV_LOAD_IMAGE_UNCHANGED);
        //imRight = cv::imread(strImageRight,CV_LOAD_IMAGE_UNCHANGED);
        double tframe;
        // Pass the images to the SLAM system
        SLAM.TrackStereo(imLeft,imRight,tframe);



    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");

    return 0;
}
