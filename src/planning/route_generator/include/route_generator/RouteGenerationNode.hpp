/*
 * -- Arbor Robotics --
 * Package:   route_generator
 * Filename:  RouteGenerationNode.hpp
 * Author:    Will Heitman
 * Email:     w@heit.mn
 * License:   MIT License
 */

#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "rclcpp/rclcpp.hpp"

using std_msgs::msg::String;

namespace arbor
{
    namespace route_generator
    {

        class RouteGenerationNode : public rclcpp::Node
        {
        public:
            RouteGenerationNode();
            virtual ~RouteGenerationNode();

        private:
            void topic_cb(const String::SharedPtr msg) const;

            rclcpp::Subscription<String>::SharedPtr subscriber_;
        };

    }
}