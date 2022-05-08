from re import I
import traceback
import rclpy
from rclpy.node import Node

from ament_index_python import get_package_share_directory
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster
from builtin_interfaces.msg import Duration
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

import os
from ament_index_python import get_package_share_directory


DATA_PATH = get_package_share_directory("jrb_localization")


class MapManager(Node):
    def __init__(self):
        super().__init__("map_manager")
        self.get_logger().info("init")

        # Marker publishers
        latchedQoS = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=1,
        )
        self.map_marker_publisher = self.create_publisher(
            Marker, "debug/table_mesh", latchedQoS
        )

        # Latched markers doen't show up in rviz, so we use a timer :(
        publish_rate = 1
        self.publish_map_marker_timer = self.create_timer(
            publish_rate, self.on_map_publish_map_marker_timer
        )

        # Tf publisher
        self.tf_broadcaster = TransformBroadcaster(self)

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

    def on_map_publish_map_marker_timer(self):
        self.map_marker_publisher.publish(self.map_marker_msg)


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
