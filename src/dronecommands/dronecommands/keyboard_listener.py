import rclpy
from rclpy.node import Node
from sshkeyboard import listen_keyboard
from std_msgs.msg import String
from threading import Thread

class KeyboardListener(Node):
    def __init__(self):
        super().__init__('keyboard_listener')
        self.publisher_ = self.create_publisher(String, '/dronecontrol_commands', 10)
        Thread(target=self.listen_for_keys, daemon=True).start()

    def publish_key(self, key):
        msg = String()
        msg.data = key
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{key}"')

    def press_handler(self, key):
        self.publish_key(key)

    def listen_for_keys(self):
        listen_keyboard(on_press=self.press_handler)

def main(args=None):
    rclpy.init(args=args)
    keyboard_listener = KeyboardListener()

    try:
        rclpy.spin(keyboard_listener)  # Spin the node in the main thread
    except KeyboardInterrupt:
        pass
    
    finally:
        keyboard_listener.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()