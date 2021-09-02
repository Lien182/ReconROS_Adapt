#include <iostream>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <chrono>

#include <opencv2/core/core.hpp>

#include "orb_slam/include/System.h"
#include "orb_slam/include/FPGA.h"
#include "orb_slam/include/Converter.h"

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
        
        //rows, cols, type, data, step
        imLeft  = cv::Mat(rorbslam_image_msg_left->height,  rorbslam_image_msg_left->width, CV_8S,  rorbslam_image_msg_left->data.data, 1);
        imRight = cv::Mat(rorbslam_image_msg_right->height, rorbslam_image_msg_right->width,CV_8S,  rorbslam_image_msg_right->data.data,1);
        double tframe = 0.0;
        // Pass the images to the SLAM system
        cv::Mat Tcw = SLAM.TrackStereo(imLeft,imRight,tframe);


        //https://github.com/raulmur/ORB_SLAM2/issues/597

        
        //geometry_msg_out->header.stamp = ros::Time::now();
        //geometry_msg_out->header.frame_id ="map";

        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t(); // Rotation information
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3); // translation information
        vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);




        //tf::Transform new_transform;
        //new_transform.setOrigin(tf::Vector3(twc.at<float>(0, 0), twc.at<float>(0, 1), twc.at<float>(0, 2)));

        //tf::Quaternion quaternion(q[0], q[1], q[2], q[3]);
        //new_transform.setRotation(quaternion);

        //tf::poseTFToMsg(new_transform, pose.pose);
        
        rorbslam_geometry_msg_out->pose.position.x = twc.at<float>(0, 0);
        rorbslam_geometry_msg_out->pose.position.y = twc.at<float>(0, 1);
        rorbslam_geometry_msg_out->pose.position.z = twc.at<float>(0, 2);

        rorbslam_geometry_msg_out->pose.orientation.x = q[0];
        rorbslam_geometry_msg_out->pose.orientation.y = q[1];
        rorbslam_geometry_msg_out->pose.orientation.z = q[2];
        rorbslam_geometry_msg_out->pose.orientation.w = q[3];
        
        ros_publisher_publish(rorbslam_pub_out, rorbslam_geometry_msg_out);

        //geometry_msgs__msg__PoseStamped
            //std_msgs__msg__Header header;
            //geometry_msgs__msg__Pose pose;
                //geometry_msgs__msg__Point position;
                    //double x;
                    //double y;
                    //double z;
                //geometry_msgs__msg__Quaternion orientation;
                    //double x;
                    //double y;
                    //double z;
                    //double w;
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");

    return;
}
