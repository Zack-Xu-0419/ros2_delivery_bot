import rclpy
from rclpy.node import Node
from enum import IntEnum, auto

from geometry_msgs.msg import Twist

class MissionState(IntEnum):
    IDLE = auto()
    SEARCHING = auto()
    PARKING = auto()
    DELIVERING = auto()
    RETURNING = auto()

class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')
        
        # Create Publisher for cmd_vel
        # Queue size of 10 is standard
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Change timer to 0.1 seconds (10Hz) for smoother control
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # Variables to track logic
        self.moving = True
        self.counter = 0
        self.switch_time = 30 # 30 ticks * 0.1s = 3 seconds

        self.get_logger().info('Mission Controller: Move/Stop Cycle Started')

    def control_loop(self):
        # Create the message object
        msg = Twist()

        # LOGIC: Toggle between moving and stopping based on the counter
        if self.moving:
            msg.linear.x = 0.5  # Move forward at 0.5 m/s
            msg.angular.z = 0.0 # No rotation
            self.get_logger().info(f'Moving... ({self.counter}/{self.switch_time})')
        else:
            msg.linear.x = 0.0  # Stop
            msg.angular.z = 0.0
            self.get_logger().info(f'Stopped... ({self.counter}/{self.switch_time})')

        # Publish the message
        self.publisher_.publish(msg)
        
        # Increment counter
        self.counter += 1

        # Check if it is time to switch states
        if self.counter >= self.switch_time:
            self.moving = not self.moving # Toggle the boolean
            self.counter = 0              # Reset counter

    # def control_loop(self):
    #     self.get_logger().info(f'CURRENT STATE: [{self.state.name}]')

    #     if self.state == MissionState.IDLE:
    #         self.get_logger().info("Waiting for mission start...")
    #         self.state = MissionState.SEARCHING

    #     elif self.state == MissionState.SEARCHING:
    #         self.get_logger().info("Locating apriltagS")
    #         self.state = MissionState.PARKING

    #     elif self.state == MissionState.PARKING:
    #         self.get_logger().info("Backing up")
    #         self.state = MissionState.DELIVERING

    #     elif self.state == MissionState.DELIVERING:
    #         self.get_logger().info("Dropping Package")
    #         self.state = MissionState.RETURNING

    #     elif self.state == MissionState.RETURNING:
    #         self.get_logger().info("Mission Complete.")

def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()