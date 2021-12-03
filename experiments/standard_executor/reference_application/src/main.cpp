#include <cinttypes>
#include <memory>

#include "sorter_msgs/srv/sort.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "sensor_msgs/msg/image.hpp"

using std::placeholders::_1;
using Sort = sorter_msgs::srv::Sort;
rclcpp::Node::SharedPtr g_node = nullptr;


//Some functions are written in C
extern "C"
{
  uint32_t calc_inverse(uint32_t input);
  void    sort_bubble(uint32_t * ram);
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


void handle_service(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<Sort::Request> request,
  const std::shared_ptr<Sort::Response> response)
{
  (void)request_header;
  RCLCPP_INFO(g_node->get_logger(), "request: ");
  
  sort_bubble((uint32_t*)&request->unsorted[0]);
  
  for(int i = 0; i < 2048; i++)
    response->sorted.push_back(request->unsorted[i]);
  
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;

  g_node = rclcpp::Node::make_shared("sorter_node");
  auto server = g_node->create_service<Sort>("sorter", handle_service);
  
  auto inversenode = std::make_shared<InverseNode>();
  auto mnistenode  = std::make_shared<MnistNode>();

  executor.add_node(g_node);
  executor.add_node(inversenode);
  executor.add_node(mnistenode);

  executor.spin();
  rclcpp::shutdown();
  g_node = nullptr;
  return 0;
}
