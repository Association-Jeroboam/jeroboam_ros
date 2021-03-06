from queue import Queue
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from message_filters import SimpleFilter, Subscriber
from typing import TypeVar
from rclpy.node import Node
from builtin_interfaces.msg import Time

StampedMsgType = TypeVar("StampedMsgType")


# Source: https://answers.ros.org/question/49069/is-there-a-tfmessagefilter-in-the-tf-python-api/
class TfMessageFilter(SimpleFilter):
    """Stores a message unless corresponding transforms is
    available
    """

    def __init__(
        self,
        input_filter: Subscriber,
        target_frame: str,
        source_frame: str,
        queue_size: int = 500,
    ):
        super().__init__()
        self.connectInput(input_filter)
        self.node: Node = input_filter.node
        self.target_frame = target_frame
        self.source_frame = source_frame
        # TODO: Use a better data structure
        self.message_queue: Queue[StampedMsgType] = Queue(maxsize=queue_size)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        self.max_queue_size = queue_size
        self._max_queue_size_so_far = 0

    def connectInput(self, input_filter: Subscriber) -> None:
        self.incoming_connection = input_filter.registerCallback(self.input_callback)

    def poll_transforms(self, latest_msg_tstamp: Time) -> None:
        """
        Poll transforms corresponding to all messages. If found throw older
        messages than the timestamp of transform just found
        and if not found keep all the messages.
        """
        # Check all the messages for transform availability
        tmp_queue = Queue(self.max_queue_size)
        first_iter = True

        # Loop from old to new
        while not self.message_queue.empty():
            msg = self.message_queue.get()
            tstamp = msg.header.stamp
            if first_iter and self.message_queue.qsize() > self._max_queue_size_so_far:
                first_iter = False
                self._max_queue_size_so_far = self.message_queue.qsize()
                self.node.get_logger().debug(
                    "Queue(%d) time range: %f - %f"
                    % (self.message_queue.qsize(), tstamp.sec, latest_msg_tstamp.sec)
                )
                self.node.get_logger().info(
                    "Maximum queue size used: %d" % self._max_queue_size_so_far
                )
            if self.tf_buffer.can_transform(
                self.target_frame, self.source_frame, tstamp
            ):
                transform_msg = self.tf_buffer.lookup_transform(
                    self.target_frame, self.source_frame, tstamp
                )
                self.signalMessage(msg, transform_msg)
                # Note that we are deliberately throwing away the messages
                # older than transform we just received
                return
            else:
                # if we don't find any transform we will have to recycle all
                # the messages
                tmp_queue.put(msg)
        self.message_queue = tmp_queue

    def input_callback(self, msg: StampedMsgType) -> None:
        """Handles incoming message"""
        if self.message_queue.full():
            # throw away the oldest message
            self.node.get_logger().warn(
                "Queue too small. If you this message too often consider increasing queue_size"
            )
            self.message_queue.get()

        self.message_queue.put(msg)
        # This can be part of another timer thread
        # TODO: call this only when a new/changed transform
        self.poll_transforms(msg.header.stamp)
