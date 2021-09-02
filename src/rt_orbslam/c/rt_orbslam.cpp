#include <iostream>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <chrono>

#include <opencv2/core/core.hpp>

#include "orb_slam/include/System.h"
#include "orb_slam/include/FPGA.h"

extern "C" {
    #include "reconos.h"
    #include "reconos_thread.h"
}




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
    {


        ros_subscriber_message_take(rorbslam_sub_left,  rorbslam_image_msg_left);
        ros_subscriber_message_take(rorbslam_sub_right, rorbslam_image_msg_right);
        // Read left and right images from file
        
        imLeft  = cv::Mat(100,100,CV_8S,rorbslam_image_msg_left->data.data,1);
        imRight = cv::Mat(100,100,CV_8S,rorbslam_image_msg_right->data.data,1);
        double tframe = 0.0;
        // Pass the images to the SLAM system
        SLAM.TrackStereo(imLeft,imRight,tframe);



    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");

    return;
}
