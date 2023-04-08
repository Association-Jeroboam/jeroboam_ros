#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "jrb_msgs/msg/sample_detected.hpp"
#include "jrb_msgs/msg/sample_detected_array.hpp"

const auto DATA_PATH = ament_index_cpp::get_package_share_directory("jrb_sensors");

class SampleDetector : public rclcpp::Node
{
public:
    SampleDetector(rclcpp::NodeOptions const &options) : Node("sample_detector", options)
    {
        this->declare_parameter("big_marker_size", 0.050);
        this->declare_parameter("small_marker_size", 0.015);
        this->declare_parameter("length_of_axis", 0.02);
        this->declare_parameter("publish_debug", true);

        big_marker_size_ = this->get_parameter("big_marker_size").as_double();
        small_marker_size_ = this->get_parameter("small_marker_size").as_double();
        length_of_axis_ = this->get_parameter("length_of_axis").as_double();
        publish_debug_ = this->get_parameter("publish_debug").as_bool();

        auto sensor_qos = rclcpp::QoS(rclcpp::SensorDataQoS());

        publisher_ = this->create_publisher<jrb_msgs::msg::SampleDetectedArray>(
            "sample_detected", 10);
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "debug/sample_detected", 10);
        image_publisher_ = image_transport::create_publisher(this, "debug/image", sensor_qos.get_rmw_qos_profile());

        camera_info_subscriber_ = create_subscription<sensor_msgs::msg::CameraInfo>("/camera/camera_info", 10, std::bind(&SampleDetector::on_camera_info, this, std::placeholders::_1));

        if (options.use_intra_process_comms())
        {
            image_subscriber_ = create_subscription<sensor_msgs::msg::Image>("/camera/image_raw", 10, std::bind(&SampleDetector::on_image_raw, this, std::placeholders::_1));
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "no intra coms");
            it_image_subscriber_ = image_transport::create_subscription(this, "/camera/image_raw", std::bind(&SampleDetector::on_it_image_raw, this, std::placeholders::_1), "raw", sensor_qos.get_rmw_qos_profile());
        }

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        aruco_dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_1000);
        aruco_params_ = cv::aruco::DetectorParameters::create();
    }

private:
    void on_it_image_raw(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
    {
        if (camera_matrix_.empty() || dist_coeffs_.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No CameraInfo yet, discard frame");
            return;
        }

        cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(gray, aruco_dict_, corners, ids, aruco_params_);

        if (!ids.empty())
        {
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(corners, big_marker_size_, camera_matrix_,
                                                 dist_coeffs_, rvecs, tvecs);

            jrb_msgs::msg::SampleDetectedArray msg;
            msg.header.stamp = this->now();
            msg.header.frame_id = "camera_link_optical";

            visualization_msgs::msg::MarkerArray markers_msg;

            for (size_t i = 0; i < ids.size(); ++i)
            {
                jrb_msgs::msg::SampleDetected sample_msg;

                if (ids[i] == 47)
                {
                    sample_msg.type = jrb_msgs::msg::SampleDetected::RED;
                }
                else if (ids[i] == 13)
                {
                    sample_msg.type = jrb_msgs::msg::SampleDetected::BLUE;
                }
                else if (ids[i] == 36)
                {
                    sample_msg.type = jrb_msgs::msg::SampleDetected::GREEN;
                }
                else if (ids[i] == 17)
                {
                    sample_msg.type = jrb_msgs::msg::SampleDetected::ROCK;
                }
                else if (ids[i] == 91 || ids[i] == 92)
                {
                    continue;
                }
                else
                {
                    continue;
                }

                sample_msg.pose.position.x = tvecs[i][0];
                sample_msg.pose.position.y = tvecs[i][1];
                sample_msg.pose.position.z = 0.355;

                cv::Mat rotationMatrix;
                cv::Rodrigues(rvecs[i], rotationMatrix);

                // tf2::Matrix3x3 tf2Matrix(
                //     rotationMatrix.at<double>(0, 0), rotationMatrix.at<double>(0, 1), rotationMatrix.at<double>(0, 2),
                //     rotationMatrix.at<double>(1, 0), rotationMatrix.at<double>(1, 1), rotationMatrix.at<double>(1, 2),
                //     rotationMatrix.at<double>(2, 0), rotationMatrix.at<double>(2, 1), rotationMatrix.at<double>(2, 2));

                // tf2::Quaternion q;
                // tf2Matrix.getRotation(q);

                tf2::Matrix3x3 rotation_matrix;
                double angle_z = atan2(rotationMatrix.at<double>(1, 0), rotationMatrix.at<double>(0, 0));
                rotation_matrix.setEulerYPR(angle_z, 0, 0);
                tf2::Quaternion q;
                rotation_matrix.getRotation(q);

                sample_msg.pose.orientation.x = q.x();
                sample_msg.pose.orientation.y = q.y();
                sample_msg.pose.orientation.z = q.z();
                sample_msg.pose.orientation.w = q.w();

                msg.samples.push_back(sample_msg);

                visualization_msgs::msg::Marker marker_msg = make_marker_msg(
                    markers_msg.markers.size(), this->now(), sample_msg.pose, std::string(sample_msg.type));
                markers_msg.markers.push_back(marker_msg);
            }

            if (publish_debug_)
            {

                cv::aruco::drawAxis(frame, camera_matrix_, dist_coeffs_, cv::Vec3d({0.0, 0.0, 0.0}), cv::Vec3d({0.0, 0.0, 0.0}), length_of_axis_);
                cv::aruco::drawDetectedMarkers(frame, corners, ids);

                sensor_msgs::msg::Image::SharedPtr img_msg =
                    cv_bridge::CvImage(msg.header, "bgr8", frame).toImageMsg();
                img_msg->header.frame_id = "camera_link_optical";
                image_publisher_.publish(img_msg);
            }

            publisher_->publish(msg);
            marker_publisher_->publish(markers_msg);
        }
    }

    void on_image_raw(sensor_msgs::msg::Image::UniquePtr msg)
    {
        const sensor_msgs::msg::Image::ConstSharedPtr msg_shared(msg.get());
        on_it_image_raw(msg_shared);
    }

    void on_camera_info(sensor_msgs::msg::CameraInfo::UniquePtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Got camera info, unsubscribing");
        camera_matrix_ = cv::Mat(3, 3, CV_64F, const_cast<double *>(msg->k.data())).clone();
        dist_coeffs_ = cv::Mat(1, 5, CV_64F, const_cast<double *>(msg->d.data())).clone();
        camera_info_subscriber_.reset();
    }

    visualization_msgs::msg::Marker make_marker_msg(
        size_t id, const rclcpp::Time &stamp,
        const geometry_msgs::msg::Pose &pose,
        std::string color)
    {
        visualization_msgs::msg::Marker marker;

        marker.header.frame_id = "camera_link_optical";
        marker.header.stamp = stamp;

        marker.lifetime = rclcpp::Duration(2, 0);

        marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.id = id;

        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;

        marker.pose = pose;

        if (color == jrb_msgs::msg::SampleDetected::RED)
        {
            marker.mesh_resource =
                "file://" + DATA_PATH + "/meshes/echantillon_rouge.dae";
        }
        else if (color == jrb_msgs::msg::SampleDetected::GREEN)
        {
            marker.mesh_resource =
                "file://" + DATA_PATH + "/meshes/echantillon_vert.dae";
        }
        else if (color == jrb_msgs::msg::SampleDetected::BLUE)
        {
            marker.mesh_resource =
                "file://" + DATA_PATH + "/meshes/echantillon_bleu.dae";
        }
        else if (color == jrb_msgs::msg::SampleDetected::ROCK)
        {
            marker.mesh_resource =
                "file://" + DATA_PATH + "/meshes/echantillon_surprise.dae";
        }

        marker.mesh_use_embedded_materials = true;

        return marker;
    }
    rclcpp::Publisher<jrb_msgs::msg::SampleDetectedArray>::SharedPtr publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
    image_transport::Publisher image_publisher_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscriber_;
    image_transport::Subscriber it_image_subscriber_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    cv::Ptr<cv::aruco::Dictionary> aruco_dict_;
    cv::Ptr<cv::aruco::DetectorParameters> aruco_params_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;

    double big_marker_size_;
    double small_marker_size_;
    double length_of_axis_;
    bool publish_debug_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SampleDetector>(rclcpp::NodeOptions{}));
    rclcpp::shutdown();
    return 0;
}