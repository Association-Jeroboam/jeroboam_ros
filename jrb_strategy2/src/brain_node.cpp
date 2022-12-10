#include "rclcpp/rclcpp.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_minitrace_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_file_logger.h"
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"

#define DEFAULT_BT_XML "./eurobot.xml"
using namespace std::chrono_literals;

void bind_actions(BT::BehaviorTreeFactory &factory) {
    const std::vector<std::string> plugin_libs = {
        "is_match_started_condition",
        "is_equal_condition",
        "is_timer_over_condition"
    };

    BT::SharedLibrary loader;
    for (const auto & p : plugin_libs) {
        std::cout << loader.getOSName(p) << std::endl;
        factory.registerFromPlugin(loader.getOSName(p));
    }
}

int main(int argc, char **argv)
{
    // Force flush of the stdout buffer
    // setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Init ROS
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("brain");
    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // Parameters
    node->declare_parameter("bt_xml", rclcpp::ParameterValue(std::string(DEFAULT_BT_XML)));
    std::string bt_xml;
    node->get_parameter("bt_xml", bt_xml);


    // Init blackboard
    auto blackboard = BT::Blackboard::create(); 
    blackboard->set<rclcpp::Node::SharedPtr>("node", node);
    blackboard->set<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", tf_buffer);
    blackboard->set<std::chrono::milliseconds>("bt_loop_duration", 1000ms);

    // Init behavior tree
    BT::BehaviorTreeFactory factory;
    bind_actions(factory);
    auto tree = factory.createTreeFromFile(bt_xml, blackboard);

    // Loggers
    BT::StdCoutLogger logger_cout(tree);
    BT::PublisherZMQ publisher_zmq(tree);
    BT::FileLogger logger_file(tree, "bt_trace.fbl");

    // Initial print
    BT::printTreeRecursively(tree.rootNode());

    rclcpp::WallRate loopRate(blackboard->get<std::chrono::milliseconds>("bt_loop_duration"));

    try {
        while (rclcpp::ok()) {
            tree.tickRoot();

            loopRate.sleep();
        }
    } catch (const std::exception & ex) {
        RCLCPP_ERROR(
            rclcpp::get_logger(""),
            "Behavior tree threw exception: %s. Exiting with failure.", ex.what()
        );
    }

    // Shut down ROS
    rclcpp::shutdown();

    return 0;
}