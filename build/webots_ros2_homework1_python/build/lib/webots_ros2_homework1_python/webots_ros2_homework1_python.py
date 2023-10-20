import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile
import math

LINEAR_VEL_FAST = 0.3  # Linear velocity for fast movement (0.3 m/s)
LINEAR_VEL_SLOW = 0.08  # Linear velocity for slow movement (0.08 m/s)
ANGULAR_VEL_SLOW = 0.523  # Angular velocity for slow rotation (30 degrees/s)
ANGULAR_VEL_FAST = 2.094  # Angular velocity for fast rotation (120 degrees/s)

class RandomWalk(Node):
    def __init__(self):
        super().__init__('random_walk_node')
        self.scan_cleaned = []
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
        self.turtlebot_moving = False
        self.cmd = Twist()
        self.linear_velocity = LINEAR_VEL_FAST
        self.angular_velocity = ANGULAR_VEL_FAST
        self.start_time = None
        self.target_distance = 1.0  # Default: 1 meter for linear movement
        self.target_rotation = math.radians(10)  # Default: 10 degrees for rotation
        self.pose_saved = Odometry()  # Initialize pose_saved attribute with an Odometry message

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
        orientation = msg2.pose.pose.orientation
        (posx, posy, posz) = (position.x, position.y, position.z)
        (qx, qy, qz, qw) = (orientation.x, orientation.y, orientation.z, orientation.w)
        self.pose_saved = msg2  # Update pose_saved with the received Odometry message

        if self.start_time is not None:
            time_elapsed = self.get_clock().now() - self.start_time
            if time_elapsed.nanoseconds / 1e9 >= 1.0:
                self.start_time = self.get_clock().now()
                self.control_robot()
        else:
            self.start_time = self.get_clock().now()
            self.control_robot()

    def control_robot(self):
        if self.pose_saved is not None:
            position = self.pose_saved.pose.pose.position
            distance_traveled = math.sqrt(position.x ** 2 + position.y ** 2)
            rotation_angle = 2.0 * math.acos(self.pose_saved.pose.pose.orientation.w)

            if distance_traveled < self.target_distance:
                self.move_robot(self.linear_velocity, 0.0)
            elif rotation_angle < self.target_rotation:
                self.move_robot(0.0, self.angular_velocity)
            else:
                self.stop_robot()


    def move_robot(self, linear_vel, angular_vel):
        self.cmd.linear.x = linear_vel
        self.cmd.angular.z = angular_vel
        self.publisher_.publish(self.cmd)
        self.turtlebot_moving = True

    def stop_robot(self):
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publisher_.publish(self.cmd)
        self.turtlebot_moving = False

    def run(self):
        while rclpy.ok():
            choice = input("Enter a number to select the movement:\n"
                           "1. 1 meter at 0.3 m/s\n"
                           "2. 1 meter at 0.08 m/s\n"
                           "3. 5 meters at 0.3 m/s\n"
                           "4. 5 meters at 0.08 m/s\n"
                           "5. Rotate 10 degrees at 30 degrees/s\n"
                           "6. Rotate 180 degrees at 120 degrees/s\n"
                           "7. Rotate 360 degrees at 120 degrees/s\n")

            if choice == '1':
                self.target_distance = 1.0
                self.target_rotation = 0.0
                self.linear_velocity = LINEAR_VEL_FAST
                self.angular_velocity = 0.0
                self.control_robot()
            elif choice == '2':
                self.target_distance = 1.0
                self.target_rotation = 0.0
                self.linear_velocity = LINEAR_VEL_SLOW
                self.angular_velocity = 0.0
                self.control_robot()
            elif choice == '3':
                self.target_distance = 5.0
                self.target_rotation = 0.0
                self.linear_velocity = LINEAR_VEL_FAST
                self.angular_velocity = 0.0
                self.control_robot()
            elif choice == '4':
                self.target_distance = 5.0
                self.target_rotation = 0.0
                self.linear_velocity = LINEAR_VEL_SLOW
                self.angular_velocity = 0.0
                self.control_robot()
            elif choice == '5':
                self.target_distance = 0.0
                self.target_rotation = math.radians(10)
                self.linear_velocity = 0.0
                self.angular_velocity = ANGULAR_VEL_SLOW
                self.control_robot()
            elif choice == '6':
                self.target_distance = 0.0
                self.target_rotation = math.radians(180)
                self.linear_velocity = 0.0
                self.angular_velocity = ANGULAR_VEL_FAST
                self.control_robot()
            elif choice == '7':
                self.target_distance = 0.0
                self.target_rotation = math.radians(360)
                self.linear_velocity = 0.0
                self.angular_velocity = ANGULAR_VEL_FAST
                self.control_robot()
            else:
                self.get_logger().info('Invalid choice. Please enter a valid number.')

def main(args=None):
    rclpy.init(args=args)
    random_walk_node = RandomWalk()
    random_walk_node.run()
    random_walk_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
