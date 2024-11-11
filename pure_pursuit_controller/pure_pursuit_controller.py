import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped
import math

# Constants for pure pursuit algorithm
lookahead_distance = 0.3
speed = 0.2

# Helper function to convert quaternion orientation to yaw (heading)
def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z

# Core logic of the Pure Pursuit algorithm to determine the robot's motion
def pure_pursuit(current_x, current_y, current_heading, path, index):
    closest_point = None
    v = speed

    # Search for the closest point on the path ahead of the robot within the lookahead distance
    for i in range(index, len(path)):
        x = path[i][0]
        y = path[i][1]
        distance = math.hypot(current_x - x, current_y - y)
        if lookahead_distance < distance:
            closest_point = (x, y)
            index = i
            break

    # If a closest point is found, calculate desired heading
    if closest_point is not None:
        target_heading = math.atan2(closest_point[1] - current_y, closest_point[0] - current_x)
        desired_steering_angle = target_heading - current_heading
    else:
        # If no point is found, aim towards the last point on the path
        target_heading = math.atan2(path[-1][1] - current_y, path[-1][0] - current_x)
        desired_steering_angle = target_heading - current_heading
        index = len(path) - 1

    # Normalize the steering angle to the range of [-π, π]
    if desired_steering_angle > math.pi:
        desired_steering_angle -= 2 * math.pi
    elif desired_steering_angle < -math.pi:
        desired_steering_angle += 2 * math.pi

    # If steering angle is too large, reduce speed and limit steering to a maximum
    if desired_steering_angle > math.pi / 6 or desired_steering_angle < -math.pi / 6:
        sign = 1 if desired_steering_angle > 0 else -1
        desired_steering_angle = sign * math.pi / 4
        v = 0.0

    return v, desired_steering_angle, index

# ROS2 Node for Pure Pursuit algorithm
class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')

        # Subscriptions to receive planned path and odometry updates
        self.subscription_path = self.create_subscription(Path, 'planned_path', self.path_callback, 10)
        self.subscription_odom = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        # Publisher to send control commands (velocity) to robot
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer to periodically compute and publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)

        # State variables
        self.path = None  # Holds the planned path
        self.index = 0  # Index of the current closest point on the path
        self.current_x = 0.0  # Robot's current x-coordinate
        self.current_y = 0.0  # Robot's current y-coordinate
        self.current_yaw = 0.0  # Robot's current orientation (yaw)
        self.goal_reached = False  # Flag to check if the goal is reached
        self.previous_first_point = None  # To detect if the path has changed
        self.previous_last_point = None  # To detect if the path has changed

    # Callback function for the path update (when a new planned path is received)
    def path_callback(self, msg):
        # Convert the received path into a list of coordinates
        new_path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        new_first_point = new_path[0] if new_path else None
        new_last_point = new_path[-1] if new_path else None

        # If the path has changed, update the path and find the nearest point
        if new_first_point != self.previous_first_point or new_last_point != self.previous_last_point:
            self.path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
            # self.index = 0
            self.index = self.find_nearest_point_index()
            self.goal_reached = False
            self.previous_first_point = new_first_point
            self.previous_last_point = new_last_point
            self.get_logger().info(f"Received new path with {len(self.path)} points.")

    # Function to find the nearest point on the path based on current position
    def find_nearest_point_index(self):
        min_distance = float('inf')
        nearest_index = 0
        for i, (x, y) in enumerate(self.path):
            distance = math.hypot(self.current_x - x, self.current_y - y)
            if distance < min_distance:
                min_distance = distance
                nearest_index = i
        return nearest_index

    # Callback function for the odometry update (to get robot's position and orientation)
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = euler_from_quaternion(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )

    # Periodic function to compute and send control commands (twist message)
    def timer_callback(self):
        if self.path and not self.goal_reached:
            twist = Twist()
            # Calculate linear and angular velocities based on the pure pursuit algorithm
            twist.linear.x, twist.angular.z, self.index = pure_pursuit(
                self.current_x, self.current_y, self.current_yaw, self.path, self.index
            )
            self.get_logger().info(f"Linear velocity: {twist.linear.x}, Angular velocity: {twist.angular.z}, Index: {self.index}")

            # Check if the robot has reached the goal
            if self.index >= len(self.path) - 1 and math.hypot(self.current_x - self.path[-1][0], self.current_y - self.path[-1][1]) < 0.05:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.goal_reached = True
                self.get_logger().info("Goal reached.")

            # Publish the computed control command
            self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    pure_pursuit_node = PurePursuitNode()
    rclpy.spin(pure_pursuit_node)
    pure_pursuit_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
