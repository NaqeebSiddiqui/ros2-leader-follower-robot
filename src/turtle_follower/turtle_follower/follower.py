import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math


class Follower(Node):

    def __init__(self):
        super().__init__('turtle_follower')

        self.cmd_pub = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)

        # leader pose
        self.create_subscription(Pose, '/turtle1/pose', self.leader_pose_callback, 10)

        # follower pose
        self.create_subscription(Pose, '/turtle2/pose', self.follower_pose_callback, 10)

        self.leader_x = 0.0
        self.leader_y = 0.0

        self.my_x = 0.0
        self.my_y = 0.0
        self.my_theta = 0.0

        self.timer = self.create_timer(0.1, self.control_loop)

    def leader_pose_callback(self, msg):
        self.leader_x = msg.x
        self.leader_y = msg.y

    def follower_pose_callback(self, msg):
        self.my_x = msg.x
        self.my_y = msg.y
        self.my_theta = msg.theta

    def control_loop(self):
        twist = Twist()

        dx = self.leader_x - self.my_x
        dy = self.leader_y - self.my_y

        distance = math.sqrt(dx**2 + dy**2)
        # stop if close enough
        if distance < 0.5:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            return
        target_angle = math.atan2(dy, dx)
        angle_error = target_angle - self.my_theta

        # normalize angle
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        twist.linear.x = 2.0 * distance
        twist.angular.z = 6.0 * angle_error

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = Follower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
