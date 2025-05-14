#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
import tf_transformations # Для преобразования углов Эйлера в кватернион
import numpy as np

class GroundTruthOdometryStubPublisher(Node):
    def __init__(self):
        super().__init__('ground_truth_odometry_stub_publisher')

        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('publish_rate', 10.0) # Гц
        # Параметры для имитации движения (если хотите)
        self.declare_parameter('fake_linear_x_speed', 0.0)  # м/с
        self.declare_parameter('fake_angular_z_speed', 0.0) # рад/с

        self.odom_frame = self.get_parameter('odom_frame_id').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame_id').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.fake_vx = self.get_parameter('fake_linear_x_speed').get_parameter_value().double_value
        self.fake_wz = self.get_parameter('fake_angular_z_speed').get_parameter_value().double_value

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        self.get_logger().info(f"Узел-заглушка одометрии запущен. Публикация на '{self.odom_pub.topic_name}' с частотой {self.publish_rate} Hz.")
        if self.fake_vx != 0.0 or self.fake_wz != 0.0:
            self.get_logger().info(f"Имитация движения: Vx={self.fake_vx} m/s, Wz={self.fake_wz} rad/s")
        else:
            self.get_logger().info("Имитация статической одометрии (робот на месте).")


    def timer_callback(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        # Обновляем позу на основе фейковых скоростей
        delta_x = (self.fake_vx * np.cos(self.theta)) * dt
        delta_y = (self.fake_vx * np.sin(self.theta)) * dt
        delta_theta = self.fake_wz * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        
        # Нормализуем угол тета в диапазон [-pi, pi]
        while self.theta > np.pi: self.theta -= 2.0 * np.pi
        while self.theta < -np.pi: self.theta += 2.0 * np.pi

        # Создаем кватернион из угла рыскания (theta)
        q = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        ros_quaternion = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        # Создаем сообщение Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0 # Для 2D SLAM
        odom_msg.pose.pose.orientation = ros_quaternion
        
        # Ковариации можно оставить очень маленькими для "идеальной" фейковой одометрии
        odom_msg.pose.covariance[0] = 1e-3   # x
        odom_msg.pose.covariance[7] = 1e-3   # y
        odom_msg.pose.covariance[14] = 1e-6  # z - не используется в 2D
        odom_msg.pose.covariance[21] = 1e-6  # rotation x - не используется в 2D
        odom_msg.pose.covariance[28] = 1e-6  # rotation y - не используется в 2D
        odom_msg.pose.covariance[35] = 1e-3  # rotation z (yaw)

        odom_msg.twist.twist.linear.x = self.fake_vx
        odom_msg.twist.twist.linear.y = 0.0 # В 2D обычно нет бокового движения
        odom_msg.twist.twist.angular.z = self.fake_wz
        
        # Ковариации скоростей
        odom_msg.twist.covariance[0] = 1e-3   # vx
        odom_msg.twist.covariance[7] = 1e-3   # vy
        odom_msg.twist.covariance[35] = 1e-3  # wz

        self.odom_pub.publish(odom_msg)

        # Публикуем TF odom -> base_link
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0 # Для 2D
        t.transform.rotation = ros_quaternion
        self.tf_broadcaster.sendTransform(t)

        self.last_time = current_time
        self.get_logger().info(f"Опубликована фейковая одометрия: x={self.x:.2f}, y={self.y:.2f}, th={self.theta:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthOdometryStubPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()