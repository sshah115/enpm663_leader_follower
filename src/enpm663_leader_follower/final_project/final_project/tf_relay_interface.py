
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy


class TFRelay(Node):
    def __init__(self, namespace):
        super().__init__("tf_relay" + "_" + str(namespace))
        tf_topic = "/" + str(namespace) + "/tf"
        self.frame_prefix = str(namespace) + "/"
        self.subscription = self.create_subscription(
            msg_type=TFMessage,
            topic=tf_topic,
            callback=self.tf_callback,
            qos_profile=QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.VOLATILE,
                depth=10,
            ),
        )
        self.publisher = self.create_publisher(
            msg_type=TFMessage,
            topic="/tf",
            qos_profile=QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.VOLATILE,
                depth=10,
            ),
        )

    def tf_callback(self, msg):
        for transform in msg.transforms:
            transform.header.frame_id = self.frame_prefix + transform.header.frame_id
            transform.child_frame_id = self.frame_prefix + transform.child_frame_id
            
            # output ="====================\n"
            # output += "Parent: " + str(transform.header.frame_id)
            # output += "Child: " + str(transform.child_frame_id)
            # output += "====================\n"
            # self.get_logger().info(output)

        self.publisher.publish(msg)
        

class TFStaticRelay(Node):
    def __init__(self, namespace):
        super().__init__("tf_static_relay" + "_" + str(namespace))
        tf_static_topic = "/" + str(namespace) +  "/tf_static"
        self.frame_prefix = str(namespace) + "/"
        self.static_subscription = self.create_subscription(
            msg_type=TFMessage,
            topic=tf_static_topic,
            callback=self.static_tf_callback,
            qos_profile=QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                depth=10,
            ),
        )

        self.static_publisher = self.create_publisher(
            msg_type=TFMessage,
            topic="/tf_static",
            qos_profile=QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                depth=10,
            ),
        )

    def static_tf_callback(self, msg):
        for transform in msg.transforms:
            transform.header.frame_id = self.frame_prefix + transform.header.frame_id
            transform.child_frame_id = self.frame_prefix + transform.child_frame_id
        self.static_publisher.publish(msg)
        
        
