#pragma once

#include "legbot_exploration/shared_layer_map.hpp"

#include <rclcpp/rclcpp.hpp>
#include <memory>

std::shared_ptr<rclcpp::Node> make_online_layer_map_node(
    std::shared_ptr<legbot_exploration::SharedLayerMap> shared);
std::shared_ptr<rclcpp::Node> make_explorer_node(
    std::shared_ptr<legbot_exploration::SharedLayerMap> shared);
