import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray  # The topic use UInt8MultiArray to receive integer arrays
import can
import array

def pad_list(data_bytes, target_length=8, pad_value=0):
    # Calculate how many elements to pad
    padding_needed = target_length - len(data_bytes)
    
    if padding_needed > 0:
        # Pad the list with the specified pad_value
        data_bytes += [pad_value] * padding_needed  # Append the padding values
    elif padding_needed < 0:
        # Trim the list if it's longer than the target_length
        data_bytes = data_bytes[:target_length]
    
    return data_bytes

def pad_array(data_bytes, target_length=8, pad_value=0):
    # Calculate how many elements to pad
    padding_needed = target_length - len(data_bytes)
    
    if padding_needed > 0:
        # Create an array of the same type as data_bytes, filled with the pad_value
        padding = array.array(data_bytes.typecode, [pad_value] * padding_needed)
        data_bytes.extend(padding)  # Append padding array
    elif padding_needed < 0:
        # Trim the array if it's longer than the target_length
        data_bytes = data_bytes[:target_length]
    
    return data_bytes

class CANUSBNode(Node):
    def __init__(self):
        super().__init__('odrive_sender_node')

        # Initialize CAN bus
        self.bus = can.Bus(interface='socketcan', channel='can0', bitrate=500000)

        # Create a subscriber to listen on the `encoded_data` topic
        self.subscriber = self.create_subscription(
            UInt8MultiArray,     # Expecting an array of integers
            'encoded_data',      # Topic name
            self.listener_callback,
            10
        )
        
    def listener_callback(self, msg):
        # Ensure we have exactly 5 elements: 2 ID and 4 data bytes
        if len(msg.data) != 6:
            self.get_logger().error("Invalid data length; expected 6 integers (2 ID and 4 data bytes)")
            return

        # Extract ID and data bytes from the message
        arbitration_id = msg.data[0] << 8 | msg.data[1]
        data_bytes = msg.data[2:6]
        data_bytes = pad_array(data_bytes)

        # Create CAN message with extracted ID and data bytes
        can_message = can.Message(
            arbitration_id=arbitration_id,
            data=data_bytes,
            is_extended_id=False
        )

        hex_data = ''.join(f'{b:02X}' for b in data_bytes)
        print(f"{arbitration_id:02X}#{hex_data}")

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