#include <cinttypes>
#include <memory>
#include <iostream>
#include <fstream>
#include <ratio>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/u_int32_multi_array.hpp"


using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;


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

class InverseNode : public rclcpp::Node
{
  public:
    InverseNode()
    : Node("periodic_node")
    {
        
      myfile.open ("periodic.csv");
      myfile << "Periodic" << ";" <<std::endl;
      cnt = 0;
      RCLCPP_INFO(this->get_logger(), "PeriodicNode started");

      subscription_ = this->create_subscription<std_msgs::msg::UInt32MultiArray>("hash", 10, std::bind(&InverseNode::topic_callback, this, _1));
      clock_gettime(CLOCK_MONOTONIC, (timespec*)&tstart);
    }

  private:
    void topic_callback(const std_msgs::msg::UInt32MultiArray::SharedPtr msg)
    {
      msg;
      clock_gettime(CLOCK_MONOTONIC, (timespec*)&time2);
      myfile << (double)diff(time1,time2).tv_nsec / 1000000  << ";" <<std::endl;  
      RCLCPP_INFO(this->get_logger(), "Periodic node callback");    
      clock_gettime(CLOCK_MONOTONIC, (timespec*)&time1);      
      
      if(cnt < 1000)
      {
        cnt++;
      }
      else
      {
        clock_gettime(CLOCK_MONOTONIC, (timespec*)&tend);
        myfile << ((double)diff(tstart,tend).tv_sec * 1000.0) + (double)diff(tstart,tend).tv_nsec / 1000000  << ";" <<std::endl;
        myfile.close();
        rclcpp::shutdown();
      }      
    }

    rclcpp::Subscription<std_msgs::msg::UInt32MultiArray>::SharedPtr subscription_;

    timespec time1, time2, tstart, tend;
    uint32_t cnt;
    std::ofstream myfile;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InverseNode>());
  rclcpp::shutdown();
  return 0;
}
