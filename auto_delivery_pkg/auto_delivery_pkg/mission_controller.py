import rclpy
from rclpy.node import Node
from enum import IntEnum, auto
import time

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
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # State variables
        self.moving = True
        self.counter = 0
        self.switch_time = 30 
        
        # Safe Start Logic
        self.start_delay = 100 # 100 ticks * 0.1s = 10 seconds
        self.boot_counter = 0
        self.is_initialized = False

        self.get_logger().info('Mission Controller: Initialized (Waiting 10s warmup)')

    def control_loop(self):
        # 1. Warmup Phase (Non-blocking replacement for time.sleep)
        if not self.is_initialized:
            self.boot_counter += 1
            if self.boot_counter % 10 == 0:
                self.get_logger().info(f"Warming up... {self.boot_counter/10}s")
            
            if self.boot_counter >= self.start_delay:
                self.is_initialized = True
                self.get_logger().info("Warmup Complete. Starting Motion.")
            return # Exit loop here, don't publish yet

        # 2. Main Logic
        msg = Twist()
        
        if self.moving:
            msg.linear.x = 0.5
            msg.angular.z = -0.5
            # Throttle logs to every 1 second (every 10 ticks)
            if self.counter % 10 == 0: 
                self.get_logger().info(f'Moving... ({self.counter}/{self.switch_time})')
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            if self.counter % 10 == 0:
                self.get_logger().info(f'Stopped... ({self.counter}/{self.switch_time})')

        self.publisher_.publish(msg)
        self.counter += 1

        if self.counter >= self.switch_time:
            self.moving = not self.moving
            self.counter = 0

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
