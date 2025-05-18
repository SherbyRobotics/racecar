#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, MagneticField, JointState
import geometry_msgs.msg

import tf2_ros
import tf_transformations
import math

class ArduinoSensors(Node):
    def __init__(self):
        super().__init__('arduino_sensors')

        self._l = self.declare_parameter('wheelbase', 0.34).value
        self._w = self.declare_parameter('axle_track', 0.20).value
        self._radius = self.declare_parameter('wheel_radius', 0.05).value
        self._child_frame_id = self.declare_parameter('child_frame_id', 'base_footprint').value
        self._publish_tf = self.declare_parameter('publish_tf', False).value
        self._tf_prefix = self.declare_parameter('tf_prefix', 'racecar').value

        # State space variables
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._total_distance = 0.0
        self.q = tf_transformations.quaternion_from_euler(0, 0, self._theta)

        self.raw_odom_sub = self.create_subscription(Float32MultiArray, 'raw_odom', self._raw_odom_cb, 1000)

        self._odom_pub = self.create_publisher(Odometry, 'odom', 5)
        self._odom_tf = tf2_ros.TransformBroadcaster(self)
        self._imu_pub = self.create_publisher(Imu, 'imu/data_raw', 5)
        self._mag_pub = self.create_publisher(MagneticField, 'imu/mag', 5)
        self._joints_pub = self.create_publisher(JointState, 'joint_states', 5)

    def _raw_odom_cb(self, raw_odom):
        if len(raw_odom.data) != 19:
            self.get_logger().error("Received data from arduino should have a length of 19! current length=%d, make sure you have the latest arduino firmware installed.", len(raw_odom.data))
            return
        
        elapsed_seconds = raw_odom.data[8]/1000.0
        totalEncDistance = raw_odom.data[0]
        speed = raw_odom.data[9]/elapsed_seconds
        distance = speed*elapsed_seconds
        steering_angle = -raw_odom.data[6]
        
        # IMU
        linear_acceleration_x = raw_odom.data[10]
        linear_acceleration_y = raw_odom.data[11]
        linear_acceleration_z = raw_odom.data[12]
        angular_velocity_x = raw_odom.data[13]
        angular_velocity_y = raw_odom.data[14]
        angular_velocity_z = raw_odom.data[15]
        magnetic_x = raw_odom.data[16]
        magnetic_y = raw_odom.data[17]
        magnetic_z = raw_odom.data[18]

        if elapsed_seconds <= 0:
            self.get_logger().warn("elapsed_seconds is 0.")
            return
        
        v_x = speed * math.cos(self._theta)
        v_y = speed * math.sin(self._theta)
        v_theta = speed * math.tan(steering_angle) / self._l

        self._x = self._x + v_x * elapsed_seconds
        self._y = self._y + v_y * elapsed_seconds
        self._theta = self._theta + v_theta * elapsed_seconds

        now = self.get_clock().now().to_msg()
        self._send_odometry(now, self._x, self._y, self._theta, speed, 0, v_theta)
        self._send_imu(now, linear_acceleration_x, linear_acceleration_y, linear_acceleration_z,
                       angular_velocity_x, angular_velocity_y, angular_velocity_z)
        self._send_mag(now, magnetic_x, magnetic_y, magnetic_z)
        self._send_wheel_joints(now, steering_angle, distance)

    def _send_odometry(self, now, x, y, theta, v_x, v_y, v_theta):
        odometry_msg = Odometry()
        odometry_msg.header.stamp = now
        odometry_msg.header.frame_id = self._tf_prefix + '/odom'
        odometry_msg.child_frame_id = self._tf_prefix + '/' + self._child_frame_id

        odometry_msg.pose.pose.position.x = float(x)
        odometry_msg.pose.pose.position.y = float(y)

        self.q = tf_transformations.quaternion_from_euler(0.0, 0.0, theta)
        

        # Check for NaN values in the quaternion
        if any(math.isnan(i) for i in self.q):
            self.get_logger().error("Quaternion contains NaN values: (%f, %f, %f, %f)", self.q[0], self.q[1], self.q[2], self.q[3])
            return

        # Normalize the quaternion
        norm = math.sqrt(self.q[0]**2 + self.q[1]**2 + self.q[2]**2 + self.q[3]**2)
        quaternion = [q / norm for q in self.q]
        
        odometry_msg.pose.pose.orientation.x = quaternion[0]
        odometry_msg.pose.pose.orientation.y = quaternion[1]
        odometry_msg.pose.pose.orientation.z = quaternion[2]
        odometry_msg.pose.pose.orientation.w = quaternion[3]

        odometry_msg.twist.twist.linear.x = float(v_x)
        odometry_msg.twist.twist.linear.y = float(v_y)
        odometry_msg.twist.twist.angular.z = float(v_theta)

        odometry_msg.pose.covariance = [0.01 if i % 7 == 0 else 0.0 for i in range(36)]
        odometry_msg.twist.covariance = [0.01 if i % 7 == 0 else 0.0 for i in range(36)]

        self._odom_pub.publish(odometry_msg)

        if self._publish_tf:
            t = geometry_msgs.msg.TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self._tf_prefix + '/odom'
            t.child_frame_id = odometry_msg.child_frame_id
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = quaternion[0]
            t.transform.rotation.y = quaternion[1]
            t.transform.rotation.z = quaternion[2]
            t.transform.rotation.w = quaternion[3]
            self._odom_tf.sendTransform(t)

    def _send_imu(self, now, linear_acceleration_x, linear_acceleration_y, linear_acceleration_z,
                  angular_velocity_x, angular_velocity_y, angular_velocity_z):
        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = self._tf_prefix + '/imu_link'

        imu_msg.orientation_covariance = [0.1, 0.0, 0.0, 0., 0.1, 0., 0., 0., 0.1]

        imu_msg.linear_acceleration.x = linear_acceleration_x
        imu_msg.linear_acceleration.y = linear_acceleration_y
        imu_msg.linear_acceleration.z = linear_acceleration_z

        imu_msg.linear_acceleration_covariance = [0.1, 0.0, 0.0, 0., 0.1, 0., 0., 0., 0.1]

        imu_msg.angular_velocity.x = angular_velocity_x
        imu_msg.angular_velocity.y = angular_velocity_y
        imu_msg.angular_velocity.z = angular_velocity_z

        imu_msg.angular_velocity_covariance = [0.1, 0.0, 0.0, 0., 0.1, 0., 0., 0., 0.1]

        self._imu_pub.publish(imu_msg)

    def _send_mag(self, now, x, y, z):
        mag_msg = MagneticField()
        mag_msg.header.stamp = now
        mag_msg.header.frame_id = self._tf_prefix + '/imu_link'

        mag_msg.magnetic_field.x = x
        mag_msg.magnetic_field.y = y
        mag_msg.magnetic_field.z = z

        self._mag_pub.publish(mag_msg)

    def _send_wheel_joints(self, now, angle, distance):
        self._total_distance = self._total_distance + distance
        rotation = self._total_distance / self._radius
        joint_state = JointState()
        joint_state.header.stamp = now
        joint_state.name = ['left_front_wheel_joint', 'left_rear_wheel_joint',
                            'right_rear_wheel_joint', 'right_front_wheel_joint',
                            'left_steering_hinge_joint', 'right_steering_hinge_joint']
        lsinphy = 2*self._l*math.sin(angle)
        lcosphy = 2*self._l*math.cos(angle)
        wsinphy = self._w*math.sin(angle)
        steering_angle_left = math.atan(lsinphy / (lcosphy - wsinphy))
        steering_angle_right = math.atan(lsinphy / (lcosphy + wsinphy))
        joint_state.position = [rotation, rotation, rotation, rotation, steering_angle_left, steering_angle_right]
        self._joints_pub.publish(joint_state)

def main(args=None):
    rclpy.init(args=args)
    arduinoSensors = ArduinoSensors()
    rclpy.spin(arduinoSensors)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
