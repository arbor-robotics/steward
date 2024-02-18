#include "route_generator/RouteGenerationNode.hpp"

using namespace std::chrono_literals;
using arbor::route_generator::RouteGenerationNode;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

RouteGenerationNode::RouteGenerationNode()
    : Node("route_generation_node")
{
  subscriber_ = this->create_subscription<std_msgs::msg::String>("chatter", 10, std::bind(&RouteGenerationNode::topic_cb, this, _1));
}

RouteGenerationNode::~RouteGenerationNode() {}

void RouteGenerationNode::topic_cb(const String::SharedPtr msg) const
{
  RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
}