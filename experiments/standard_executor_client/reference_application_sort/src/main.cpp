#include <cinttypes>
#include <memory>
#include <iostream>
#include <fstream>
#include <ratio>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp> 

#include "sorter_msgs/srv/sort.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "sensor_msgs/msg/image.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using Sort = sorter_msgs::srv::Sort;


timespec diff(timespec start, timespec end)
{
    timespec temp;
    if ((end.tv_nsec-start.tv_nsec)<0) {
        temp.tv_sec = end.tv_sec-start.tv_sec-1;
        temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
    } else {
        temp.tv_sec = end.tv_sec-start.tv_sec;
        temp.tv_nsec = end.tv_nsec-start.tv_nsec;
    }
    return temp;
}


 
using namespace std::chrono_literals;
 
int main(int argc, char **argv)
{
  timespec time1, time2, tstart, tend;
  std::ofstream myfile;
  myfile.open ("sort.csv");
  myfile << "Sort" << ";" <<std::endl;
  rclcpp::init(argc, argv);
    
  // Create the node
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("sort_client");
   
  // Create the client for the node
  rclcpp::Client<sorter_msgs::srv::Sort>::SharedPtr client =
    node->create_client<sorter_msgs::srv::Sort>("sorter");
 
  // Make the request
  auto request = std::make_shared<sorter_msgs::srv::Sort::Request>();
  for(int i = 0; i < 2048; i++)
    request->unsorted.push_back(rand());
 
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      // Show an error if the user types CTRL + C
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    // Search for service nodes in the network
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }
 
  clock_gettime(CLOCK_MONOTONIC, (timespec*)&tstart);
  for(int cnt = 0; cnt < 1000; cnt++)
  {
    // Send a request
    clock_gettime(CLOCK_MONOTONIC, (timespec*)&time1);
    auto result = client->async_send_request(request);
    
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result) ==
      rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      clock_gettime(CLOCK_MONOTONIC, (timespec*)&time2);
      myfile << (double)diff(time1,time2).tv_nsec / 1000000  << ";" <<std::endl;
      uint32_t sorted = 1;
      for(int i = 1; i < 2048; i++)
      {
        if(result.get()->sorted[i-1] > result.get()->sorted[i])
          sorted = 0;
      }
    } 
    else 
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service sorter");
    }
  }
  clock_gettime(CLOCK_MONOTONIC, (timespec*)&tend);
  myfile << ((double)diff(tstart,tend).tv_sec * 1000.0) + (double)diff(tstart,tend).tv_nsec / 1000000  << ";" <<std::endl;
  myfile.close();
  rclcpp::shutdown();
  return 0;
}
