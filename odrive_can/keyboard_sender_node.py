import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
import struct

def float_to_hex(f):
    return struct.pack('<f', f)
    #return hex(struct.unpack('<I', struct.pack('<f', f))[0])

class KeyboardInputNode(Node):
    def __init__(self):
        super().__init__('keyboard_sender_node')
        self.publisher_ = self.create_publisher(UInt8MultiArray, 'encoded_data', 10)
        self.timer = self.create_timer(0.1, self.publish_encoded_data)

    def publish_encoded_data(self):
        # Ask user for input in format [ID(hex)],[MODE(struct)],[DATA(float)]
        user_input = input("Enter input: ")

        # Split the input into components
        id_str, mode_str, data_str = user_input.split(',')

        # Convert ID from string to integer (assuming hex)
        input_id = int(id_str, 16)

        if input_id > 0x3F:  # 6-bit limit
            self.get_logger().error("Input ID exceeds 6-bit limit for 11-bit encoding")
            return

        # Determine mode and corresponding hexadecimal
        mode_hex = 0x0D if mode_str == 'vel' else 0x0C if mode_str == 'ang' else None
        if mode_hex is None:
            self.get_logger().error(f"Invalid mode: {mode_str}")
            return

        # Encode ID: ID_input << 5 | mode_hex
        encoded_id = ((input_id << 5) | mode_hex) & 0x7FF

        # Convert data from float to IEEE 754 (little-endian)
        data_float = float(data_str)
        data_bytes = float_to_hex(data_float)

        # Convert bytes to array of integers for ROS message
        data_array = list(data_bytes)

        # Prepare message
        message = UInt8MultiArray()
        # Extract the 8 least significant bits and the upper 3 bits
        encoded_id_low = encoded_id & 0xFF
        encoded_id_high = (encoded_id >> 8) & 0x07  # 3 bits

        message.data = [encoded_id_high, encoded_id_low] + [int(b) for b in data_array[:4]]


        # Log and publish the message
        hex_data_array = [hex(b) for b in data_array]
        self.get_logger().info(f"Encoded ID: {hex(encoded_id)}, Data: {hex_data_array}")
        self.publisher_.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardInputNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
