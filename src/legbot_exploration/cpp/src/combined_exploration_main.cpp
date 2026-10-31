#include "legbot_exploration/exploration_node_factories.hpp"

#include <rclcpp/rclcpp.hpp>

#include <exception>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

int main(int argc, char **argv) {
  // Compatibility names choose a mode, while the launch files pass it
  // explicitly. Remove this application argument before ROS parses argv.
  const std::string executable = argv[0] ? argv[0] : "";
  std::string mode = executable.find("online_layer_map_cpp") != std::string::npos ?
      "map" : executable.find("explorer_cpp") != std::string::npos ?
      "flat" : "combined";
  std::vector<char *> ros_args{argv[0]};
  for (int i = 1; i < argc; ++i) {
    const std::string argument = argv[i];
    constexpr char prefix[] = "--exploration-mode=";
    if (argument.rfind(prefix, 0) == 0)
      mode = argument.substr(sizeof(prefix) - 1);
    else
      ros_args.push_back(argv[i]);
  }
  if (mode != "combined" && mode != "flat" && mode != "map") {
    std::cerr << "Invalid exploration mode: " << mode << '\n';
    return 2;
  }
  rclcpp::init(static_cast<int>(ros_args.size()), ros_args.data());
  int exit_code = 0;
  try {
    if (mode == "map") {
      rclcpp::spin(make_online_layer_map_node(nullptr));
    } else if (mode == "flat") {
      rclcpp::spin(make_explorer_node(nullptr));
    } else {
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
    }
  } catch (const std::exception &error) {
    RCLCPP_ERROR(rclcpp::get_logger("combined_exploration"), "%s", error.what());
    exit_code = 1;
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  return exit_code;
}
