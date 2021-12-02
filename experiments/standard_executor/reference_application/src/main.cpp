#include <cinttypes>
#include <memory>

#include "sorter_msgs/srv/sort.hpp"
#include "rclcpp/rclcpp.hpp"

using Sort = sorter_msgs::srv::Sort;
rclcpp::Node::SharedPtr g_node = nullptr;

#define BLOCK_SIZE 2048

void sort_bubble(uint32_t ram[BLOCK_SIZE]) {
	unsigned int i, j;
	uint32_t tmp;
	for (i = 0; i < BLOCK_SIZE; i++) {
		for (j = 0; j < BLOCK_SIZE - 1; j++) {
			if (ram[j] > ram[j + 1]) {
				tmp = ram[j];
				ram[j] = ram[j + 1];
				ram[j + 1] = tmp;
			}
		}
	}
}

void handle_service(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<Sort::Request> request,
  const std::shared_ptr<Sort::Response> response)
{
  (void)request_header;
  RCLCPP_INFO(g_node->get_logger(), "request: %" PRId64 " + %" PRId64, request->unsorted[0], request->unsorted[1]);
  
  sort_bubble((uint32_t*)&request->unsorted[0]);
  
  for(int i = 0; i < BLOCK_SIZE; i++)
    response->sorted.push_back(request->unsorted[i]);
  //sort_bubble());
  
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  g_node = rclcpp::Node::make_shared("sorter_node");
  auto server = g_node->create_service<Sort>("sorter", handle_service);
  rclcpp::spin(g_node);
  rclcpp::shutdown();
  g_node = nullptr;
  return 0;
}
