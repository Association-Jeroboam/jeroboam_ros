#include "jrb_webots/WebotsRos2PluginExample.hpp"

namespace webots_ros2_plugin_example
{
  void WebotsRos2PluginExample::init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters)
  {
    // This method is executed once the plugin is loaded by the `webots_ros2_driver` package.
    // The `webots_ros2_driver::WebotsNode` inherits the `rclcpp::Node`, so you have all methods available from there.
    // In addition, from the `webots_ros2_driver::WebotsNode` instance you can also get a `webots::Robot` reference (`node.robot()`).
  }
  void WebotsRos2PluginExample::step()
  {
    // This method is executed on each Webots step
  }
}

// The class has to be exported with `PLUGINLIB_EXPORT_CLASS` macro.
// The first argument is the name of your class, while the second is always `webots_ros2_driver::PluginInterface`
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(webots_ros2_plugin_example::WebotsRos2PluginExample, webots_ros2_driver::PluginInterface)