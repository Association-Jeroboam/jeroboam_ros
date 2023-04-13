from re import I
import traceback
import rclpy
from rclpy.node import Node

from ament_index_python import get_package_share_directory
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster
from builtin_interfaces.msg import Duration
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, FloatingPointRange
from rcl_interfaces.msg import SetParametersResult

import os
from ament_index_python import get_package_share_directory


DATA_PATH = get_package_share_directory("jrb_localization")


class MapManager(Node):
    def __init__(self):
        super().__init__("map_manager")
        self.get_logger().info("init")

        self.declare_parameter(
            "tf_publish_rate",
            50.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[
                    FloatingPointRange(from_value=1.0, to_value=100.0, step=0.1)
                ],
            ),
        )

        # Subscribers
        self.sub_odometry = self.create_subscription(
            Odometry, "/odom", self.on_odometry, 10
        )
        # Publishers
        latchedQoS = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=1,
        )
        self.map_marker_publisher = self.create_publisher(
            Marker, "/debug/table_mesh", latchedQoS
        )

        # Tf publisher
        self.tf_broadcaster = TransformBroadcaster(self)
        # self.tf_publish_rate = self.get_parameter("tf_publish_rate").value  # Hz
        # self.tf_publish_timer = self.create_timer(
        #     1 / self.tf_publish_rate, self.on_publish_tf_timer
        # )

        # Map marker msg
        self.map_marker_msg = Marker()

        self.map_marker_msg.header.frame_id = "odom"
        self.map_marker_msg.header.stamp = self.get_clock().now().to_msg()

        self.map_marker_msg.lifetime = Duration()

        self.map_marker_msg.type = Marker.MESH_RESOURCE
        self.map_marker_msg.action = Marker.ADD
        self.map_marker_msg.id = 0

        self.map_marker_msg.pose.position.x = 0.0
        self.map_marker_msg.pose.position.y = 0.0
        self.map_marker_msg.pose.position.z = -0.01

        self.map_marker_msg.scale.x = 1.0
        self.map_marker_msg.scale.y = 1.0
        self.map_marker_msg.scale.z = 1.0

        self.map_marker_msg.mesh_resource = "file://" + os.path.join(
            DATA_PATH, "meshes/table2023.dae"
        )
        self.map_marker_msg.mesh_use_embedded_materials = True

        self.map_marker_publisher.publish(self.map_marker_msg)

        # Transform msg
        self.tf_msg = TransformStamped()

        self.add_on_set_parameters_callback(self.on_update_parameters)

    def on_update_parameters(self, params):
        # for param in params:
        #     if param.name == "tf_publish_rate":
        #         self.tf_publish_rate = param.value
        #         self.tf_publish_timer.timer_period_ns = 1 / self.tf_publish_rate * 1e9

        self.get_logger().info("Params updated")

        return SetParametersResult(successful=True)

    def on_odometry(self, msg: Odometry):
        self.tf_msg.header = msg.header
        self.tf_msg.child_frame_id = msg.child_frame_id

        position = msg.pose.pose.position
        self.tf_msg.transform.translation.x = position.x
        self.tf_msg.transform.translation.y = position.y
        self.tf_msg.transform.translation.z = position.z
        self.tf_msg.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(self.tf_msg)


def main(args=None):
    rclpy.init(args=args)

    node = MapManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped cleanly")
    except Exception:
        print("Error while stopping the node")
        print(traceback.format_exc())
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
