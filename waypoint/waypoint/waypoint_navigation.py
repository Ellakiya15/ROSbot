import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
import numpy as np

class WaypointNavigation(Node):
    def __init__(self):
        super().__init__('waypoint_navigation')

        # Declare ROS 2 parameters
        self.declare_parameter('waypoint_1_x', 1.0)
        self.declare_parameter('waypoint_1_y', 2.0)
        self.declare_parameter('waypoint_2_x', 3.0)
        self.declare_parameter('waypoint_2_y', 2.0)
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.01)
        self.declare_parameter('kd', 0.0)

        # Get parameters
        self.waypoints = [
            (self.get_parameter('waypoint_1_x').value, self.get_parameter('waypoint_1_y').value),
            (self.get_parameter('waypoint_2_x').value, self.get_parameter('waypoint_2_y').value)
        ]
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value

        # Subscribe to odometry
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # PID Error terms
        self.prev_error = 0.0
        self.integral = 0.0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.current_waypoint_index = 0

        # Timer to run control loop
        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

        # Debugging print
        # print(f"Received Odometry Data: x = {self.robot_x}, y = {self.robot_y}")  # Debugging
        # self.get_logger().info(f"Robot Position: x = {self.robot_x:.2f}, y = {self.robot_y:.2f}")

    def control_loop(self):
        if not hasattr(self, 'stop_executed'):
            self.stop_executed = False  # Ensure flag exists

        if self.current_waypoint_index >= len(self.waypoints):
            if not self.stop_executed:  # Check if stop action has already been executed
                self.get_logger().info("Waypoints reached. Stopping robot.")
                self.stop_robot()
                self.get_logger().info("Successfully reached all waypoints.")
                self.stop_executed = True  # Set flag to prevent multiple stops
            return

        target_x, target_y = self.waypoints[self.current_waypoint_index]

        # Compute Euclidean distance to target
        distance = math.sqrt((target_x - self.robot_x) ** 2 + (target_y - self.robot_y) ** 2)

        if distance < 0.1:  # If close to waypoint, switch to next
            self.get_logger().info(f"Waypoint {self.current_waypoint_index + 1} reached: ({target_x}, {target_y})")
            self.current_waypoint_index += 1
            if self.current_waypoint_index >= len(self.waypoints):
                if not self.stop_executed:
                    self.get_logger().info("Waypoints reached. Stopping robot.")
                    self.stop_robot()
                    self.get_logger().info("Successfully reached all waypoints.")
                    self.stop_executed = True  # Ensure it runs only once
            return

        # Compute angle to waypoint
        target_angle = math.atan2(target_y - self.robot_y, target_x - self.robot_x)
        angle_error = target_angle - self.robot_yaw

        # Normalize angle error
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        # PID Controller for angular velocity
        self.integral += angle_error * 0.1
        derivative = (angle_error - self.prev_error) / 0.1
        angular_vel = self.kp * angle_error + self.ki * self.integral + self.kd * derivative
        self.prev_error = angle_error

        # Proportional speed based on distance
        linear_vel = min(0.5, distance * 0.5)

        # Publish velocity
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        self.cmd_vel_pub.publish(twist)


    # def control_loop(self):
    #     if self.current_waypoint_index >= len(self.waypoints):
    #         self.stop_robot()
    #         return

    #     target_x, target_y = self.waypoints[self.current_waypoint_index]

    #     # Compute Euclidean distance to target
    #     distance = math.sqrt((target_x - self.robot_x) ** 2 + (target_y - self.robot_y) ** 2)

    #     if distance < 0.1:  # If close to waypoint, switch to next
    #         self.get_logger().info(f"Waypoint {self.current_waypoint_index + 1} reached: ({target_x}, {target_y})")
    #         self.current_waypoint_index += 1
    #         if self.current_waypoint_index >= len(self.waypoints):
    #             self.stop_robot()
    #         return

    #     # Compute angle to waypoint
    #     target_angle = math.atan2(target_y - self.robot_y, target_x - self.robot_x)
    #     angle_error = target_angle - self.robot_yaw

    #     # Normalize angle error
    #     angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

    #     # PID Controller for angular velocity
    #     self.integral += angle_error * 0.1
    #     derivative = (angle_error - self.prev_error) / 0.1
    #     angular_vel = self.kp * angle_error + self.ki * self.integral + self.kd * derivative
    #     self.prev_error = angle_error

    #     # Proportional speed based on distance
    #     linear_vel = min(0.5, distance * 0.5)

    #     # Publish velocity
    #     twist = Twist()
    #     twist.linear.x = linear_vel
    #     twist.angular.z = angular_vel
    #     self.cmd_vel_pub.publish(twist)

    # def control_loop(self):
    #     if self.current_waypoint_index >= len(self.waypoints):
    #         self.stop_robot()

    #         # Stop logging after reaching all waypoints
    #         if not hasattr(self, 'logging_stopped'):
    #             self.get_logger().info("All waypoints reached. Stopping position logging.")
    #             self.logging_stopped = True  # Set flag to prevent further prints
    #         return

    #     target_x, target_y = self.waypoints[self.current_waypoint_index]

    #     # Compute Euclidean distance to target
    #     distance = math.sqrt((target_x - self.robot_x) ** 2 + (target_y - self.robot_y) ** 2)

    #     if distance < 0.1:  # If close to waypoint, switch to next
    #         self.get_logger().info(f"Waypoint {self.current_waypoint_index + 1} reached: ({target_x}, {target_y})")
    #         self.current_waypoint_index += 1
    #         return

    #     # Compute angle to waypoint
    #     target_angle = math.atan2(target_y - self.robot_y, target_x - self.robot_x)
    #     angle_error = target_angle - self.robot_yaw

    #     # Normalize angle error
    #     angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

    #     # PID Controller for angular velocity
    #     self.integral += angle_error * 0.1
    #     derivative = (angle_error - self.prev_error) / 0.1
    #     angular_vel = self.kp * angle_error + self.ki * self.integral + self.kd * derivative
    #     self.prev_error = angle_error

    #     # Proportional speed based on distance
    #     linear_vel = min(0.5, distance * 0.5)

    #     # Publish velocity
    #     twist = Twist()
    #     twist.linear.x = linear_vel
    #     twist.angular.z = angular_vel
    #     self.cmd_vel_pub.publish(twist)



    def stop_robot(self):
        """Stop the robot when waypoints are completed"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        # self.get_logger().info
        # print("Waypoints reached. Stopping robot")

def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
