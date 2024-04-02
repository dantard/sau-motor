from std_msgs.msg import Float32
from rclpy.node import Node


class BasicNode(Node):
    def __init__(self, name, in_topic, out_topic):
        super().__init__(name)
        self.publisher = self.create_publisher(Float32, out_topic, 10)
        self.subscription = self.create_subscription(Float32, in_topic, self.receive, 10)

    def receive(self, value):
        raise NotImplementedError

    def send(self, value):
        msg = Float32()
        msg.data = value
        self.publisher.publish(msg)