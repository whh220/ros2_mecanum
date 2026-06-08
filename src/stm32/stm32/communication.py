import rclpy
from rclpy.node import Node
import serial
from geometry_msgs.msg import Twist
import struct
import threading
import time

class CommunicationStm32(Node):

    def __init__(self):
        super().__init__('stm32_node')

         # 机器人参数 (单位：米)
        self.wheel_radius = 0.06     # 轮子半径
        self.wheel_base = 0.185         # 前后轮距!!!!!!!!!!!!!!!!!!!!!!!!
        self.track_width = 0.2       # 左右轮距

        self.max_linear_speed = 0.32    # 最大线速度 m/s!!!!!!!!!!!!!!!!!
        self.max_angular_speed = 3.0   # 最大角速度 rad/s

        self.rx_buffer = bytearray()

        self.connect_ser()

        self.vel_sub=self.create_subscription(Twist,'/cmd_vel',self.send_data_callback,10)

         # 发布 /get_vel
        self.pub = self.create_publisher(Twist, "/get_vel", 10)

        #线程
        threading.Thread(target=self.receive_data, daemon=True).start()
        

    def receive_data(self):
        rate=1.0/100.0  #10ms
        

        while rclpy.ok():
            #获得缓冲区大小
            n = self.ser.inWaiting()
            if n>0:
                chunk = self.ser.read(n)
                self.process_received_data(chunk)
            time.sleep(rate)

    def process_received_data(self,chunk):
        self.rx_buffer.extend(chunk)
        while len(self.rx_buffer) >= 10:
            if self.rx_buffer[0] == 0xA8 and self.rx_buffer[1] == 0x01:
                if len(self.rx_buffer) >= 10:
                    frame = self.rx_buffer[:10]
                    self.parse_wheel_data(frame)
                    self.rx_buffer = self.rx_buffer[10:]
                else:
                    break 
            else:
                self.rx_buffer.pop(0)

    def parse_wheel_data(self,frame):
        
        lf_bytes = bytes(frame[2:4])
        rh_bytes = bytes(frame[4:6])
        rf_bytes = bytes(frame[6:8])
        lh_bytes = bytes(frame[8:10])

        lf_value = struct.unpack('>h', lf_bytes)[0]
        rh_value = struct.unpack('>h', rh_bytes)[0]
        rf_value = struct.unpack('>h', rf_bytes)[0]
        lh_value = struct.unpack('>h', lh_bytes)[0]

        vx = (lf_value - rf_value + lh_value -rh_value) /1000/ 4.0
        vy = (lf_value + rf_value + lh_value + rh_value) /1000/ 4.0
        wz = (lf_value + rf_value + lh_value + rh_value)  /1000/ (4.0 * (self.wheel_base+ self.track_width))

        # 发布速度消息
        if rclpy.ok():
            twist = Twist()
            twist.linear.x = vx
            twist.linear.y = vy
            twist.angular.z = wz
            self.pub.publish(twist)
        self.last_speed_update = time.time() 

    def send_data_callback(self,msg_data):
        # 提取速度指令
        vx= msg_data.linear.x
        vy= msg_data.linear.y
        wz = msg_data.angular.z

         # 速度限制
        vx = max(min(vx, self.max_linear_speed), -self.max_linear_speed)
        vy = max(min(vy, self.max_linear_speed), -self.max_linear_speed)
        wz = max(min(wz, self.max_angular_speed), -self.max_angular_speed)

        # 配置B的运动学逆解公式
        # 辊子方向: LF(-45°), RF(45°), LB(45°), RB(-45°)
        l = self.wheel_base / 2.0
        w = self.track_width / 2.0
        
        # 计算各轮速度 (单位: m/s)!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        w1 = ( vx - vy - (l + w) * wz)  # 左前轮 (LF)
        w2 = -( vx - vy + (l + w) * wz)  # 右后轮 (Rb)
        w3 = -( vx + vy + (l + w) * wz)  # 右前轮 (rh)
        w4 = ( vx + vy - (l + w) * wz)  # 左后轮 (lB)

        v1=bytearray(struct.pack('>h',int(w1*1000)))
        v2=bytearray(struct.pack('>h',int(w2*1000)))
        v3=bytearray(struct.pack('>h',int(w3*1000)))
        v4=bytearray(struct.pack('>h',int(w4*1000)))

        cmd=[0xb8,0x02]

        cmd.append(v1[0]) #高位
        cmd.append(v1[1])

        cmd.append(v2[0]) #高位
        cmd.append(v2[1])

        cmd.append(v3[0]) #高位
        cmd.append(v3[1])

        cmd.append(v4[0]) #高位
        cmd.append(v4[1])

        self.ser.write(cmd)


    def connect_ser(self):
        try:
            self.ser=serial.Serial(port='/dev/ttyUSB0',baudrate=115200)
            print('serial open'+str(self.ser.isOpen()))
        except Exception as e:
            print(e)

    def destroy_node(self):
        if self.ser is None:return
        self.ser.cancel_read()
        self.ser.close()


def main():
    try:
        rclpy.init()
        node = CommunicationStm32()
        rclpy.spin(node)
        rclpy.shutdown()
    except:
        node.destroy_node()




if __name__ == '__main__':
    main()