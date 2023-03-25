#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <unordered_map>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class ParameterException : public std::runtime_error
{
public:
  explicit ParameterException(const std::string & what)
  : std::runtime_error(what)
  {
  }
};

class MarkerPublisher : public rclcpp::Node
{
public:
    MarkerPublisher()
        : Node("marker_publisher", "",
               rclcpp::NodeOptions().allow_undeclared_parameters(
                                        true)
                   .automatically_declare_parameters_from_overrides(true))
    {
        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.transient_local(); // Latched QoS

        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("markers", qos);

        std::unordered_map<std::string, int> type_map{
            {"arrow", visualization_msgs::msg::Marker::ARROW},
            {"cube", visualization_msgs::msg::Marker::CUBE},
            {"sphere", visualization_msgs::msg::Marker::SPHERE},
            {"cylinder", visualization_msgs::msg::Marker::CYLINDER},
            {"line_strip", visualization_msgs::msg::Marker::LINE_STRIP},
            {"line_list", visualization_msgs::msg::Marker::LINE_LIST},
            {"cube_list", visualization_msgs::msg::Marker::CUBE_LIST},
            {"sphere_list", visualization_msgs::msg::Marker::SPHERE_LIST},
            {"points", visualization_msgs::msg::Marker::POINTS},
            {"text_view_facing", visualization_msgs::msg::Marker::TEXT_VIEW_FACING},
            {"mesh", visualization_msgs::msg::Marker::MESH_RESOURCE},
        };

        visualization_msgs::msg::MarkerArray marker_array;
        int marker_id = 0;
        rcl_interfaces::msg::ListParametersResult list = list_parameters({"markers"}, 10);
        
        for (auto prefix : list.prefixes) {
            RCLCPP_DEBUG(get_logger(), "Prefix: %s", prefix.c_str());
            
            visualization_msgs::msg::Marker marker;

            marker.header.frame_id = fetch_param<std::string>(prefix + ".frame_id");
            marker.ns = fetch_param_or<std::string>(prefix + ".ns", "");
            marker.id = fetch_param_or<int>(prefix + ".id", marker_id);
            marker.type = type_map.at(fetch_param<std::string>(prefix + ".type"));
            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.pose.position.x = fetch_param_or<double>(prefix + ".pose_x", 0.0);
            marker.pose.position.y = fetch_param_or<double>(prefix + ".pose_y", 0.0);
            marker.pose.position.z = fetch_param_or<double>(prefix + ".pose_z", 0.0);

            tf2::Quaternion quaternion;
            quaternion.setRPY(fetch_param_or<double>(prefix + ".pose_roll", 0.0), fetch_param_or<double>(prefix + ".pose_pitch", 0.0), fetch_param_or<double>(prefix + ".pose_yaw", 0.0));
            marker.pose.orientation = tf2::toMsg(quaternion);

            marker.scale.x = fetch_param_or<double>(prefix + ".scale_x", 1.0);
            marker.scale.y = fetch_param_or<double>(prefix + ".scale_y", 1.0);
            marker.scale.z = fetch_param_or<double>(prefix + ".scale_z", 1.0);

            marker.color.r = fetch_param_or<double>(prefix + ".color_r", 1.0);
            marker.color.g = fetch_param_or<double>(prefix + ".color_g", 0.0);
            marker.color.b = fetch_param_or<double>(prefix + ".color_b", 0.0);
            marker.color.a = fetch_param_or<double>(prefix + ".color_a", 1.0);

            if (marker.type == visualization_msgs::msg::Marker::MESH_RESOURCE)
            {
                marker.mesh_resource = "file://" + ament_index_cpp::get_package_share_directory(fetch_param<std::string>(prefix + ".package_name")) + "/" + fetch_param<std::string>(prefix + ".mesh_file");
            }

            marker_array.markers.push_back(marker);
            ++marker_id;
        }

        marker_pub_->publish(marker_array);
    }

private:
    template <class T>
    T fetch_param_or(const std::string& param_name, const T& default_value) const
    {
        rclcpp::Parameter parameter;
        T value;

        if (!get_parameter(param_name, parameter)) 
        {
            value = default_value;
        } else 
        {
            value = parameter.get_value<T>();
        }

        return value;
    }

    template <class T>
    T fetch_param(const std::string& param_name) const
    {
        rclcpp::Parameter parameter;

        if (!get_parameter(param_name, parameter)) 
        {
            std::ostringstream err_msg;
            err_msg << "Could not load parameter '" << param_name << "'. (namespace: " << get_namespace() << ")";
            throw ParameterException(err_msg.str());
        }

        return parameter.get_value<T>();
    }

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MarkerPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}   