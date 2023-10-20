import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile
import math

LINEAR_VEL = 0.21  # New linear velocity (0.3 m/s)
STOP_DISTANCE = 0.2
LIDAR_ERROR = 0.05
LIDAR_AVOID_DISTANCE = 0.7
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
RIGHT_SIDE_INDEX = 270
RIGHT_FRONT_INDEX = 210
LEFT_FRONT_INDEX = 150
LEFT_SIDE_INDEX = 90
TARGET_DISTANCE_1M = 1.0  # Target distance to move 1 meter
TARGET_DISTANCE_5M = 5.0  # Target distance to move 5 meters

LINEAR_VEL_FAST = 0.21  # Linear velocity for fast movement (0.3 m/s)
LINEAR_VEL_SLOW = 0.08  # Linear velocity for slow movement (0.08 m/s)


class RandomWalk(Node):
    def __init__(self):
        super().__init__('random_walk_node')
        self.scan_cleaned = []
        self.stall = False
        self.turtlebot_moving = False
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber1 = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback1,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.subscriber2 = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback2,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.laser_forward = 0
        self.odom_data = 0
        timer_period = 0.5
        self.pose_saved = ''
        self.cmd = Twist()
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.start_time = self.get_clock().now()
        self.target_distance = TARGET_DISTANCE_1M  # Start with 1 meter movement
        self.linear_velocity = LINEAR_VEL_FAST  # Start with fast velocity


    def listener_callback1(self, msg1):
        scan = msg1.ranges
        self.scan_cleaned = []
        for reading in scan:
            if reading == float('Inf'):
                self.scan_cleaned.append(3.5)
            elif math.isnan(reading):
                self.scan_cleaned.append(0.0)
            else:
                self.scan_cleaned.append(reading)

    def listener_callback2(self, msg2):
        position = msg2.pose.pose.position
        self.pose_saved = position

    def timer_callback(self):
        if len(self.scan_cleaned) == 0:
            self.turtlebot_moving = False
            return

        if self.pose_saved:
            distance_traveled = math.sqrt(self.pose_saved.x ** 2 + self.pose_saved.y ** 2)
        else:
            distance_traveled = 0.0

        if distance_traveled < self.target_distance:
            # Continue moving
            self.cmd.linear.x = self.linear_velocity
            self.cmd.angular.z = 0.0
        else:
            # Target distance reached, stop
            self.cmd.linear.x = 0.0
            self.get_logger().info('Target distance reached')

    def run(self):
        while rclpy.ok():
            choice = input("Enter a number (1/2/3/4) to select the movement:\n1. 1 meter at 0.3 m/s\n2. 1 meter at 0.08 m/s\n3. 5 meters at 0.3 m/s\n4. 5 meters at 0.08 m/s\n")
            if choice == '1':
                self.target_distance = TARGET_DISTANCE_1M
                self.cmd.linear.x = LINEAR_VEL_FAST
                self.linear_velocity = LINEAR_VEL_FAST
                self.start_moving()
            elif choice == '2':
                self.target_distance = TARGET_DISTANCE_1M
                self.cmd.linear.x = LINEAR_VEL_SLOW
                self.start_moving()
            elif choice == '3':
                self.target_distance = TARGET_DISTANCE_5M
                self.cmd.linear.x = LINEAR_VEL_FAST
                self.start_moving()
            elif choice == '4':
                self.target_distance = TARGET_DISTANCE_5M
                self.cmd.linear.x = LINEAR_VEL_SLOW
                self.start_moving()
            else:
                self.get_logger().info('Invalid choice. Please enter a valid number.')

    def start_moving(self):
        self.get_logger().info('Moving {} meters at {} m/s'.format(self.target_distance, self.linear_velocity))
        self.get_logger().info('Press Ctrl+C to stop the movement.')
        self.timer = self.create_timer(0.1, self.timer_callback)
        rclpy.spin(self)


def main(args=None):
    rclpy.init(args=args)
    random_walk_node = RandomWalk()
    random_walk_node.run()
    random_walk_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
