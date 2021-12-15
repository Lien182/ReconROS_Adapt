#include <cinttypes>
#include <memory>
#include <iostream>
#include <fstream>
#include <ratio>
#include <chrono>

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

class InverseNode : public rclcpp::Node
{
  public:
    InverseNode()
    : Node("inverse_node")
    {
        
      myfile.open ("inverse.csv");
      myfile << "Inverse" << ";" <<std::endl;
      cnt = 0;
      RCLCPP_INFO(this->get_logger(), "InverseNode started");
      publisher_ = this->create_publisher<std_msgs::msg::UInt32>("angle", 10);
      subscription_ = this->create_subscription<std_msgs::msg::UInt32>("legangle", 10, std::bind(&InverseNode::topic_callback, this, _1));
      timer_ = this->create_wall_timer( std::chrono::milliseconds(2000), std::bind(&InverseNode::timer_callback, this));
    }

  private:
    void topic_callback(const std_msgs::msg::UInt32::SharedPtr msg)
    {
      msg;
      clock_gettime(CLOCK_MONOTONIC, (timespec*)&time2);
      myfile << (double)diff(time1,time2).tv_nsec / 1000000  << ";" <<std::endl;      
      auto output_msg = std_msgs::msg::UInt32();
      clock_gettime(CLOCK_MONOTONIC, (timespec*)&time1);
      
      
      if(cnt < 1000)
      {
        publisher_->publish(output_msg);
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

    void timer_callback(void) const
    {
      auto output_msg = std_msgs::msg::UInt32();
      output_msg.data = 120;
      clock_gettime(CLOCK_MONOTONIC, (timespec*)&tstart);
      publisher_->publish(output_msg);
      timer_->cancel();
    }


    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr publisher_;

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
