from std_msgs.msg import Float32
from rclpy.node import Node


class BasicNode(Node):
    def __init__(self, name, in_topic, out_topic, kind=Float32, qos_profile=1):
        super().__init__(name)
        self.kind = kind

    def receive(self, value):
        raise NotImplementedError

