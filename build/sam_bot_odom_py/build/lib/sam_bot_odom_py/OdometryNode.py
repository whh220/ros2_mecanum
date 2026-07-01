#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster
import math

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')
        
        # 声明参数
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_tf', False)

        # 读取参数
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value
        # 发布者
        self.odom_pub = self.create_publisher(Odometry, '/wheel_odom', 10)

        # TF 广播器（仅 publish_tf 时创建，避免 rqt 上显示多余的 /tf 连接）
        self.tf_broadcaster = None
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
        
        # 订阅速度反馈（你的通信节点发布的 /get_vel）
        self.vel_sub = self.create_subscription(
            Twist, '/get_vel', self.vel_callback, 10)
        
        # 位姿初始化
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()
        
        self.get_logger().info('里程计节点已启动')

    def vel_callback(self, msg):
        """接收速度反馈，进行位姿积分"""
        # 获取当前速度
        vx = msg.linear.x    # X方向线速度 (m/s)
        vy = msg.linear.y    # Y方向线速度 (m/s)
        vth = msg.angular.z  # 角速度 (rad/s)
        
        # 计算时间差
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0.0 or dt > 0.1:  # 防止异常时间差
            self.last_time = current_time
            return
        
        # 麦克纳姆轮位姿积分（全向移动模型）
        # 注意：th是绕Z轴的旋转角度，影响坐标变换
        delta_x = (vx * math.cos(self.th) - vy * math.sin(self.th)) * dt
        delta_y = (vx * math.sin(self.th) + vy * math.cos(self.th)) * dt
        delta_th = vth * dt
        
        self.x += delta_x
        self.y += delta_y
        self.th += delta_th
        
        # 归一化角度到 [-pi, pi]
        self.th = math.atan2(math.sin(self.th), math.cos(self.th))
        
        # 构建 Odometry 消息
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        
        # 位置
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        
        # 姿态（四元数）
        odom_msg.pose.pose.orientation.z = math.sin(self.th / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.th / 2.0)
        
        # 速度（在机器人坐标系下）
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.angular.z = vth
        
        # 协方差矩阵（可根据实际调整）
        odom_msg.pose.covariance[0] = 0.01    # x方向位置方差
        odom_msg.pose.covariance[7] = 0.01    # y方向位置方差
        odom_msg.pose.covariance[35] = 0.01   # 角度方差
        
        self.odom_pub.publish(odom_msg)
        
        # 2. 发布 TF 变换（odom -> base_footprint），仅在纯轮式里程计模式下发布
        if self.publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = current_time.to_msg()
            tf_msg.header.frame_id = self.odom_frame
            tf_msg.child_frame_id = self.base_frame

            # 使用相同的位姿数据
            tf_msg.transform.translation.x = self.x
            tf_msg.transform.translation.y = self.y
            tf_msg.transform.translation.z = 0.0

            tf_msg.transform.rotation.x = 0.0
            tf_msg.transform.rotation.y = 0.0
            tf_msg.transform.rotation.z = math.sin(self.th / 2.0)
            tf_msg.transform.rotation.w = math.cos(self.th / 2.0)

            # 广播 TF
            self.tf_broadcaster.sendTransform(tf_msg)
        
        self.last_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()