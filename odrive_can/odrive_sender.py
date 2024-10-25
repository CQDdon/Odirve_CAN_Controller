import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray  # Use Int32MultiArray to receive integer arrays
import can

class CANUSBNode(Node):
    def __init__(self):
        super().__init__('odrive_sender_node')

        # Initialize CAN bus
        self.bus = can.Bus(interface='socketcan', channel='can0', bitrate=500000)

        # Create a subscriber to listen on the `encoded_data` topic
        self.subscriptions = self.create_subscription(
            Int32MultiArray,     # Expecting an array of integers
            'encoded_data',      # Topic name
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        # Ensure we have exactly 5 elements: 1 ID and 4 data bytes
        if len(msg.data) != 5:
            self.get_logger().error("Invalid data length; expected 5 integers (1 ID and 4 data bytes)")
            return

        # Extract ID and data bytes from the message
        arbitration_id = msg.data[0]
        data_bytes = msg.data[1:5]

        # Create CAN message with extracted ID and data bytes
        can_message = can.Message(
            arbitration_id=arbitration_id,
            data=data_bytes,
            is_extended_id=False
        )

        # Send CAN message
        try:
            self.bus.send(can_message)
            self.get_logger().info(f"Message sent on {self.bus.channel_info}")
        except can.CanError:
            self.get_logger().error("Message NOT sent")

def main(args=None):
    rclpy.init(args=args)
    node = CANUSBNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()