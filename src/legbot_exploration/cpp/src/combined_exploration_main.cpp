#include "legbot_exploration/exploration_node_factories.hpp"

#include <rclcpp/rclcpp.hpp>

#include <exception>
#include <memory>
#include <thread>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    auto shared = std::make_shared<legbot_exploration::SharedLayerMap>();
    auto map_node = make_online_layer_map_node(shared);
    auto explorer_node = make_explorer_node(shared);
    rclcpp::executors::SingleThreadedExecutor map_executor;
    rclcpp::executors::SingleThreadedExecutor explorer_executor;
    map_executor.add_node(map_node);
    explorer_executor.add_node(explorer_node);
    std::exception_ptr map_error;
    std::thread map_thread([&]() {
      try { map_executor.spin(); }
      catch (...) {
        map_error = std::current_exception();
        rclcpp::shutdown();
      }
    });
    try { explorer_executor.spin(); }
    catch (...) {
      map_executor.cancel();
      map_thread.join();
      throw;
    }
    map_executor.cancel();
    map_thread.join();
    if (map_error) std::rethrow_exception(map_error);
  } catch (const std::exception &error) {
    RCLCPP_ERROR(rclcpp::get_logger("combined_exploration"), "%s", error.what());
    exit_code = 1;
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  return exit_code;
}
