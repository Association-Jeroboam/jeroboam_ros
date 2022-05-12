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

import os
from ament_index_python import get_package_share_directory


DATA_PATH = get_package_share_directory("jrb_localization")


class MapManager(Node):
    def __init__(self):
        super().__init__("map_manager")
        self.get_logger().info("init")

        # Subscribers
        self.sub_odometry = self.create_subscription(
            Odometry, "odometry", self.on_odometry, 10
        )
        # Publishers
        latchedQoS = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=1,
        )
        self.map_marker_publisher = self.create_publisher(
            Marker, "debug/table_mesh", latchedQoS
        )

        # map_marker_publish_rate = 1
        # self.publish_map_marker_timer = self.create_timer(
        #     map_marker_publish_rate, self.on_map_publish_map_marker_timer
        # )

        # Tf publisher
        self.tf_broadcaster = TransformBroadcaster(self)
        tf_publish_rate = 1 / 50  # Hz
        self.publish_tf_timer = self.create_timer(
            tf_publish_rate, self.on_publish_tf_timer
        )

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
        self.map_marker_msg.pose.position.z = 0.0

        self.map_marker_msg.scale.x = 1.0
        self.map_marker_msg.scale.y = 1.0
        self.map_marker_msg.scale.z = 1.0

        self.map_marker_msg.mesh_resource = "file://" + os.path.join(
            DATA_PATH, "meshes/table.dae"
        )

        self.map_marker_msg.mesh_use_embedded_materials = True
        self.on_map_publish_map_marker_timer()

        # Transform msg
        self.tf_msg = TransformStamped()
        self.tf_msg.header.frame_id = "odom"
        self.tf_msg.child_frame_id = "base_footprint"

        self.on_publish_tf_timer()

    def on_map_publish_map_marker_timer(self):
        self.map_marker_publisher.publish(self.map_marker_msg)

    def on_publish_tf_timer(self):
        self.tf_msg.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(self.tf_msg)

    def on_odometry(self, msg: Odometry):
        self.tf_msg.transform.translation = msg.pose.pose.position
        self.tf_msg.transform.rotation = msg.pose.pose.orientation
        self.on_publish_tf_timer()


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
