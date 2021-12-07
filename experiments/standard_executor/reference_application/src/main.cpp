#include <cinttypes>
#include <memory>

#include "sorter_msgs/srv/sort.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "sensor_msgs/msg/image.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using Sort = sorter_msgs::srv::Sort;


//Some functions are written in C
extern "C"
{
  uint32_t  calc_inverse(uint32_t input);
  void      sort_bubble(uint32_t * ram);
  void      calc_sobel(uint8_t * input, uint8_t * output);
}


uint32_t calc_mnist(uint8_t * data);

class InverseNode : public rclcpp::Node
{
  public:
    InverseNode()
    : Node("inverse_node")
    {
      RCLCPP_INFO(this->get_logger(), "InverseNode started");
      publisher_ = this->create_publisher<std_msgs::msg::UInt32>("legangle", 10);
      subscription_ = this->create_subscription<std_msgs::msg::UInt32>("/angle", 10, std::bind(&InverseNode::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::UInt32::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "Inverse Client with input data %d", msg->data);
      auto output_msg = std_msgs::msg::UInt32();
      output_msg.data = calc_inverse(msg->data);
      publisher_->publish(output_msg);
    }

    rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr publisher_;
};

class MnistNode : public rclcpp::Node
{
  public:
    MnistNode()
    : Node("mnist_node")
    {
      RCLCPP_INFO(this->get_logger(), "MnistNode started");
      publisher_ = this->create_publisher<std_msgs::msg::UInt32>("class", 10);
      subscription_ = this->create_subscription<sensor_msgs::msg::Image>("/image_classification", 10, std::bind(&MnistNode::topic_callback, this, _1));
    }

  private:
    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "Mnist  with input data %d", msg->data[0]);
      auto output_msg = std_msgs::msg::UInt32();
      output_msg.data = calc_mnist(&msg->data[0]);
      publisher_->publish(output_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr publisher_;
};


class SobelNode : public rclcpp::Node
{
  public:
    SobelNode()
    : Node("SobelNode")
    {
      RCLCPP_INFO(this->get_logger(), "SobelNode started");
      publisher_ = this->create_publisher<sensor_msgs::msg::Image>("filtered", 10);
      subscription_ = this->create_subscription<sensor_msgs::msg::Image>("/image_raw", 10, std::bind(&SobelNode::topic_callback, this, _1));
    }

  private:
    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "Sobel with input data ");
      sensor_msgs::msg::Image output_msg = *msg;
      //calc_sobel(&msg->data[0], &output_msg.data[0]);
      publisher_->publish(output_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

class SortNode : public rclcpp::Node
{
  public:
    SortNode()
    : Node("SortNode")
    {
      RCLCPP_INFO(this->get_logger(), "SortNode started");
      service_ = this->create_service<Sort>("sorter",std::bind(&SortNode::handle_service, this, _1, _2, _3));
    }

  private:
    void handle_service(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<Sort::Request> request,
      const std::shared_ptr<Sort::Response> response)
      {
        (void)request_header;
        RCLCPP_INFO(this->get_logger(), "request: ");
        
        sort_bubble((uint32_t*)&request->unsorted[0]);
        
        for(int i = 0; i < 2048; i++)
          response->sorted.push_back(request->unsorted[i]);
      
      }

    rclcpp::Service<Sort>::SharedPtr service_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;

  auto sortnode = std::make_shared<SortNode>();
  auto inversenode = std::make_shared<InverseNode>();
  auto mnistnode   = std::make_shared<MnistNode>();
  auto sobelnode   = std::make_shared<SobelNode>();

  executor.add_node(sortnode);
  executor.add_node(inversenode);
  executor.add_node(mnistnode);
  executor.add_node(sobelnode);

  executor.spin();
  rclcpp::shutdown();
  return 0;
}
