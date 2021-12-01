#include <cinttypes>
#include <memory>

#include "sorter_msgs/srv/sort.hpp"
#include "rclcpp/rclcpp.hpp"

using Sort = sorter_msgs::srv::Sort;
rclcpp::Node::SharedPtr g_node = nullptr;

void handle_service(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<Sort::Request> request,
  const std::shared_ptr<Sort::Response> response)
{
  (void)request_header;
  RCLCPP_INFO(g_node->get_logger(), "request: %" PRId64 " + %" PRId64, request->a, request->b);
  response->sum = request->a + request->b;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  g_node = rclcpp::Node::make_shared("sorter_node");
  auto server = g_node->create_service<Sort>("sort", handle_service);
  rclcpp::spin(g_node);
  rclcpp::shutdown();
  g_node = nullptr;
  return 0;
}
