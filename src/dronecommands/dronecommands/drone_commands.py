# arm, takeoff, and set mode commands for the drone using mavros for ros2

import time
import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from geometry_msgs.msg import TwistStamped
import geometry_msgs.msg
from std_msgs.msg import String

class DroneCommands(Node):
    def __init__(self):
        super().__init__('drone_commands')
        self.arm_service = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.takeoff_service = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.set_mode_service = self.create_client(SetMode, '/mavros/set_mode') 

        self.publisher_ = self.create_publisher(TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)
        self.frame_id = 'base_link'

        while not self.arm_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        while not self.takeoff_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        while not self.set_mode_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.get_logger().info('services are available')

        self.cmd_vel_subscription = self.create_subscription(
            geometry_msgs.msg.Twist,
            '/desd_vel',
            self.cmd_vel_callback,
            10
        )
        self.cmd_vel_subscription

        self.subscription = self.create_subscription(
            String,
            '/drone_control_commands',
            self.keyboard_callback,
            10)
        self.subscription


    def arm(self):
        request = CommandBool.Request()
        request.value = True
        future = self.arm_service.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Arming successful')
        else:
            self.get_logger().info('Arming failed')

    def takeoff(self):
        request = CommandTOL.Request()
        request.altitude = 3.0
        future = self.takeoff_service.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Takeoff successful')
        else:
            self.get_logger().info('Takeoff failed')

    def set_mode(self, mode):
        request = SetMode.Request()
        request.custom_mode = mode
        future = self.set_mode_service.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Mode change successful')
        else:
            self.get_logger().info('Mode change failed')

    def land(self):
        self.set_mode("LAND")

    def cmd_vel_callback(self, pidMsg):
        if self.started == True:
            msg = TwistStamped()
            msg.twist.linear.x = pidMsg.linear.x
            msg.twist.linear.y = pidMsg.linear.y
            msg.twist.linear.z = pidMsg.linear.z
            msg.twist.angular.x = pidMsg.angular.x
            msg.twist.angular.y = pidMsg.angular.x
            msg.twist.angular.z = pidMsg.angular.x
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' %msg)

    def keyboard_callback(self, msg):
        if msg.data == "a":
            self.arm()

        elif msg.data == "g":
            self.set_mode("GUIDED")

        elif msg.data == "t":
            self.takeoff()
            
        elif msg.data == "l":
            self.started == False
            self.land()

        elif msg.data == "s":
            self.started == True

        else:
            self.get_logger().info('Invalid command')

    
   

def main(args=None):
    rclpy.init(args=args)
    drone_commands = DroneCommands()
    try:
        rclpy.spin(drone_commands)
    except KeyboardInterrupt:
        drone_commands.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        drone_commands.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
