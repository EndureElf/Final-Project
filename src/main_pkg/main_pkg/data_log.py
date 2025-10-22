import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, BatteryState
from geometry_msgs.msg import Point, Quaternion, Twist, Vector3
import math

class VoltageVelImuNode(Node):
    def __init__(self):
        super().__init__('voltage_vel_imu_node')
        self.create_subscription(Float32, '/voltage', self.voltage_callback, 1)
        self.create_subscription(Twist, '/vel_raw', self.velocity_callback, 1)
        self.create_subscription(Imu, '/imu/data_raw', self.imu_callback, 1)

        self.pub = self.create_publisher(Odometry, '/odom', 1)

        self.timer = self.create_timer(0.5, self.timer_callback)

        self.pose_x = 0
        self.pose_y = 0
        self.pose_theta_imu = 0
        self.pose_theta_odom = 0

        self.pose_last_x = 0
        self.pose_last_y = 0
        self.pose_last_theta = 0
        
        self.velx = 0
        self.vely = 0
        self.veltheta = 0

        self.vel_imu_x = 0
        self.vel_imu_y = 0
        self.vel_imu_theta = 0

        print("time_stamp x y theta_odom theta_imu vel_x vel_y vel_theta_odom vel_x_imu vel_y_ vel_theta_imu")
    
    def timer_callback(self):
        current_time = self.get_clock().now().nanoseconds / 1e9  # Convert to seconds
        
        print(f"{current_time} {self.pose_x} {self.pose_y} {self.pose_theta_odom} {self.pose_theta_imu} {self.velx} {self.vely} {self.veltheta} {self.vel_imu_x} {self.vel_imu_y} {self.vel_imu_theta}")
        
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        # filter 
        self.pose_theta = self.pose_theta_imu # output filter

        qx, qy, qz, qw = self.euler_to_quaternion(0,0,self.pose_theta)

        msg.pose.pose.position = Point(x=self.pose_x,y=self.pose_y, z=0.0)
        msg.pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        vel_x = self.pose_x - self.pose_last_x
        self.pose_last_x = self.pose_x
        vel_y = self.pose_y - self.pose_last_y
        self.pose_last_y = self.pose_y
        vel_theta = self.pose_theta - self.pose_last_theta
        self.pose_last_theta = self.pose_theta

        msg.twist.twist = Twist(linear=Vector3(x=vel_x, y=vel_y, z=0.0),
                                angular=Vector3(x=0.0, y=0.0, z=vel_theta))

        self.pub.publish(msg)

    def voltage_callback(self, msg):
        return

    def velocity_callback(self, msg):
        self.pose_x += msg.linear.x/10
        self.pose_y += msg.linear.y/10
        self.pose_theta_odom += msg.angular.z/10
        self.velx = msg.linear.x
        self.vely = msg.linear.y
        self.veltheta = msg.angular.z
        return

    def imu_callback(self, msg):
        self.pose_theta_imu += msg.angular_velocity.z/10
        self.vel_imu_x = msg.angular_velocity.x
        self.vel_imu_y = msg.angular_velocity.y
        self.vel_imu_theta = msg.angular_velocity.z

        return
    
    def euler_to_quaternion(self,roll: float, pitch: float, yaw: float):
        half_roll = roll * 0.5
        half_pitch = pitch * 0.5
        half_yaw = yaw * 0.5

        cr = math.cos(half_roll)
        sr = math.sin(half_roll)
        cp = math.cos(half_pitch)
        sp = math.sin(half_pitch)
        cy = math.cos(half_yaw)
        sy = math.sin(half_yaw)

        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy

        return (qx, qy, qz, qw)

def main(args=None):
    rclpy.init(args=args)
    node = VoltageVelImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down node (KeyboardInterrupt)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()