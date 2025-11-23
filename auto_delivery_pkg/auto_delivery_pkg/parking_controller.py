import rclpy
from rclpy.node import Node

class ParkingController(Node):
    def __init__(self):
        super().__init__('parking_controller')
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.get_logger().info('Parking Controller Node Initialized')

    def timer_callback(self):
        # Placeholder: This is where you would calculate steering to back up
        self.get_logger().info('Parking Controller: Waiting for target pose...', throttle_duration_sec=2.0)

def main(args=None):
    rclpy.init(args=args)
    node = ParkingController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()