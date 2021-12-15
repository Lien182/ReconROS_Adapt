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

class MnistNode : public rclcpp::Node
{
  public:
    MnistNode()
    : Node("mnist_node")
    {

      myfile.open ("mnist.csv");
      myfile << "Mnist" << ";" <<std::endl;
      RCLCPP_INFO(this->get_logger(), "MnistNode started");
      publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/image_classification", 10);
      subscription_ = this->create_subscription<std_msgs::msg::UInt32>("class", 10, std::bind(&MnistNode::topic_callback, this, _1));
      timer_ = this->create_wall_timer( std::chrono::milliseconds(2000), std::bind(&MnistNode::timer_callback, this));


      RCLCPP_INFO(this->get_logger(), "File name for publishing image is : %s", filename_.c_str());

      image_ = cv::imread("/home/christian/mnist.png", 0);
      if (image_.empty()) {  // if filename not exist, open video device
        try 
        {  // if filename is number
          int num = std::stoi(filename_);  // num is 1234798797
          cap_.open(num);
        } 
        catch (const std::invalid_argument &) 
        {  // if file name is string
          cap_.open(filename_);
        }
      cap_.read(image_);
      cap_.set(1, 0);
      }
  
  }    
  
  private:
    void doWork() const
    {
      sensor_msgs::msg::Image::SharedPtr out_img =
        cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", image_).toImageMsg();
      out_img->header.frame_id = frame_id_;
      out_img->header.stamp = rclcpp::Clock().now();
      clock_gettime(CLOCK_MONOTONIC, (timespec*)&time1);
      publisher_->publish(*out_img); 
    }

    void topic_callback(const std_msgs::msg::UInt32::SharedPtr msg)
    {
      msg;
      clock_gettime(CLOCK_MONOTONIC, (timespec*)&time2);
      myfile << (double)diff(time1,time2).tv_nsec / 1000000  << ";" <<std::endl;

      if(cnt < 1000)
      {
        doWork();
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
      clock_gettime(CLOCK_MONOTONIC, (timespec*)&tstart);
      doWork(); 
      timer_->cancel();
    }

    cv::VideoCapture cap_;
    cv::Mat image_;

    std::string filename_;
    bool flip_horizontal_;
    bool flip_vertical_;

    std::string frame_id_;
    double publish_rate_;
    std::string camera_info_url_;
    bool flip_image_;
    int flip_value_;
    sensor_msgs::msg::CameraInfo camera_info_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

    timespec time1, time2, tstart, tend;
    uint32_t cnt;
    std::ofstream myfile;
};




int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MnistNode>());
  rclcpp::shutdown();

  return 0;
}
