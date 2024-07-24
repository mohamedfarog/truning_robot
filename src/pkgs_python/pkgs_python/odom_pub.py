import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, Quaternion, Point, TransformStamped
from nav_msgs.msg import Odometry
import math
from rclpy.clock import Clock
from tf2_ros import TransformBroadcaster

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')
        self.publisher_ = self.create_publisher(Odometry, 'odom', 10)
        self.timer_ = self.create_timer(0.1, self.publish_odom)
        self.tf_broadcaster_ = TransformBroadcaster(self)
        
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_theta = 0.0
        self.wheelbase = 0.35  # Example value, adjust based on your robot's configuration
        # self.steer_ratio = 0.1  # Example value, adjust based on your robot's configuration
        self.linear_velocity = 0.0  # Initial linear velocity in m/s
        self.steering_angle = 0.5  # Initial steering angle in radians
        
        # Subscribe to the '/odom' topic to receive linear velocity updates
        self.subscription = self.create_subscription(
            Twist,
            'vel_raw',
            self.odom_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.cmd_vel_subscription  # prevent unused variable warning

    def cmd_vel_callback(self, msg):
        # Update steering angle based on the received Twist message
        self.steering_angle = msg.angular.z
        
    def odom_callback(self, msg):
        # Update linear velocity based on the received message
        self.linear_velocity = msg.linear.x

    def publish_odom(self):
        odom_msg = Odometry()

        current_time = Clock().now()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'

        # Update the robot's pose based on Ackermann steering odometry equations
        delta_time = 0.1
        delta_distance = self.linear_velocity * delta_time
        delta_theta = (delta_distance / self.wheelbase) * math.tan(self.steering_angle)

        self.pose_x += delta_distance * math.cos(delta_theta / 2)
        self.pose_y += delta_distance * math.sin(delta_theta / 2)
        self.pose_theta += delta_theta

        # Populate the odometry message
        odom_msg.pose.pose.position = Point(x=self.pose_x, y=self.pose_y, z=0.0)
        odom_msg.pose.pose.orientation = self.get_quaternion(self.pose_theta)
        odom_msg.twist.twist.linear.x = self.linear_velocity
        odom_msg.twist.twist.angular.z = (self.linear_velocity / self.wheelbase) * math.tan(self.steering_angle)
        
        self.publisher_.publish(odom_msg)

        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = self.pose_x
        t.transform.translation.y = self.pose_y
        t.transform.translation.z = 0.0
        
        t.transform.rotation = self.get_quaternion(self.pose_theta)
        
        self.tf_broadcaster_.sendTransform(t)

        print("**********************")
        print(self.linear_velocity)
        

    def get_quaternion(self, theta):
        return Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(theta / 2.0),
            w=math.cos(theta / 2.0)
        )

def main(args=None):
    rclpy.init(args=args)

    odom_publisher = OdomPublisher()

    rclpy.spin(odom_publisher)

    odom_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
