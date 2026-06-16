#!/usr/bin/env python3
"""
3iRobotics Delta-2A LiDAR ROS2 driver
Based on official protocol and actual data capture
"""

import math
import subprocess
import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

HEADER_1 = 0xAA
HEADER_2 = 0x00
CT_SCAN = 0x61

# 根据实际数据，参数长度是0x0077 = 119
# 包总大小 = 帧头(2) + 固定(1) + 版本(1) + CT(1) + 命令(1) + 参数长度(2) + 参数(119)
# = 2+1+1+1+1+2+119 = 127 字节 ✓
PACKET_SIZE = 127

BAUD_RATE = 230400
SERIAL_PORT = '/dev/ttyUSB0'

RANGE_MIN = 0.15
RANGE_MAX = 8.0


class Delta2ANode(Node):
    def __init__(self):
        super().__init__('delta2a_lidar')

        self.declare_parameter('port', SERIAL_PORT)
        self.declare_parameter('baud', BAUD_RATE)
        self.declare_parameter('frame_id', 'lidar_link')
        self.declare_parameter('topic', 'scan')

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        self._frame_id = self.get_parameter('frame_id').value
        topic = self.get_parameter('topic').value

        self._pub = self.create_publisher(LaserScan, topic, 10)

        self._scan: dict[float, float] = {}
        self._prev_start_angle: float = -1.0
        self._frame_count = 0

        # 配置串口
        try:
            subprocess.run(
                ['stty', '-F', port, str(baud), 'raw', 'cs8', '-cstopb', '-parenb'],
                check=True,
                stderr=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL
            )
            self.get_logger().info(f'stty configured {port}')
        except Exception as e:
            self.get_logger().warn(f'stty failed: {e}')

        try:
            self._ser = serial.Serial(
                port=port,
                baudrate=baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1.0,
            )
            self.get_logger().info(f'Opened {port} @ {baud} baud')
        except serial.SerialException as e:
            self.get_logger().fatal(f'Cannot open port: {e}')
            raise SystemExit(1)

        self._buf = bytearray()
        self.create_timer(0.01, self._read_loop)

    def _read_loop(self):
        try:
            if not self._ser.is_open:
                return
            waiting = self._ser.in_waiting
            if waiting > 0:
                data = self._ser.read(min(waiting, 4096))
                if data:
                    self._buf.extend(data)
                    if len(self._buf) > PACKET_SIZE * 10:
                        self._buf = self._buf[-PACKET_SIZE*2:]
                    self._parse_buffer()
        except Exception as e:
            self.get_logger().error(f'Read error: {e}')

    def _parse_buffer(self):
        buf = self._buf
        buf_len = len(buf)
        
        i = 0
        while i < buf_len - 1:
            # 查找 AA 00 帧头
            if buf[i] != HEADER_1 or (i+1 < buf_len and buf[i+1] != HEADER_2):
                i += 1
                continue

            if i + PACKET_SIZE > buf_len:
                break

            # 验证CT
            if i+4 < buf_len and buf[i+4] == CT_SCAN:
                packet = buf[i:i+PACKET_SIZE]
                self._process_packet(packet)
                i += PACKET_SIZE
                self._frame_count += 1
            else:
                i += 1

        # 清理缓冲区
        if i > 0:
            self._buf = buf[i:]
        elif buf_len > PACKET_SIZE * 2:
            self._buf = bytearray()

    def _process_packet(self, packet: bytearray):
        """
        根据官方协议解析数据包
        
        包结构（127字节）：
        字节0-1:  AA 00 (帧头)
        字节2:    7F
        字节3:    01 (版本)
        字节4:    61 (CT)
        字节5:    AD (命令)
        字节6-7:  00 77 (参数长度=119)
        字节8-?   参数数据（119字节）
        
        参数结构（根据官方协议）：
        字节0:     转速值
        字节1-2:   零点偏移量
        字节3-4:   起始角度值
        字节5:     点1信号值
        字节6-7:   点1距离值
        字节8:     点2信号值
        字节9-10:  点2距离值
        ...
        """
        if len(packet) < 8:
            return
        
        # 提取参数（从字节8开始，长度119）
        params = packet[8:]
        
        if len(params) < 5:
            return
        
        # 获取起始角度（字节3-4）
        start_angle_raw = (params[3] << 8) | params[4]
        start_angle_deg = start_angle_raw / 100.0
        
        # 计算点数：参数长度119 - 5(头部) = 114，114/3=38点
        num_points = ((packet[6]<<8) | (packet[7]) - 5) // 3
        
        if num_points <= 0:
            return
        
        # 每帧22.5度
        angle_step = 22.5 / num_points
        
        # 第一包：记录初始角度作为参考
        if not hasattr(self, '_reference_angle'):
            self._reference_angle = start_angle_deg
            # self.get_logger().info(f'Reference angle set: {self._reference_angle:.1f}°')
        
        # 检测是否再次检测到参考角度（转完一圈）
        if self._prev_start_angle >= 0.0 and start_angle_deg == self._reference_angle:
            self._publish_scan()
            self._scan.clear()
            # self.get_logger().info(f'Full rotation: {len(self._scan)} points')
        # ====================================================
            
        self._prev_start_angle = start_angle_deg
        
        # 解析距离数据
        points_added = 0
        for i in range(num_points):
            base = 5 + i * 3   #参数长度
            if base + 2 >= len(params):
                break
            
            # signal = params[base]  # 信号值，不用
            dist_raw = (params[base+1] << 8) | params[base+2]
            
            # 距离分辨率 0.25mm
            dist_m = dist_raw * 0.00025
            
            # 角度 = 起始角度 + 22.5° * i / num_points
            angle_deg = start_angle_deg + 22.5 * (i-1) / num_points
            
            if RANGE_MIN <= dist_m <= RANGE_MAX:
                self._scan[angle_deg] = dist_m
                points_added += 1
            else:
                self._scan[angle_deg] = float("inf")
        
        if self._frame_count % 100 == 0:
            self.get_logger().debug(
                f'Frame {self._frame_count}: angle={start_angle_deg:.1f}°, '
                f'points={num_points}, total={len(self._scan)}'
            )

    def _publish_scan(self):
        if len(self._scan) < 10:
            self.get_logger().warn(f'Not enough points: {len(self._scan)}')
            return

        angles_deg = sorted(self._scan.keys())
        n = len(angles_deg)

        angle_min_rad = math.radians(angles_deg[0])
        angle_max_rad = math.radians(angles_deg[-1])
        angle_inc = (angle_max_rad - angle_min_rad) / max(n - 1, 1)

        ranges = [self._scan[a] for a in angles_deg]

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.angle_min = angle_min_rad
        msg.angle_max = angle_max_rad
        msg.angle_increment = angle_inc
        msg.time_increment = 0.0
        msg.scan_time = 1.0 / 8.0
        msg.range_min = RANGE_MIN
        msg.range_max = RANGE_MAX
        msg.ranges = ranges

        self._pub.publish(msg)
        # self.get_logger().info(f'Published scan: {n} points')

    def destroy_node(self):
        if hasattr(self, '_ser') and self._ser.is_open:
            self._ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Delta2ANode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()