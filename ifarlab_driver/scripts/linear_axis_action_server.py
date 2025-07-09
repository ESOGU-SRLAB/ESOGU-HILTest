#!/usr/bin/env python3
#Şu anda bu kod m hesabına göre çalışıyor.
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from festo_edcon_ros2.action import MoveAxis  # type: ignore # Action tanımı
from rclpy.action import ActionServer
from edcon.edrive.motion_handler import MotionHandler
from std_msgs.msg import String  # Mesaj türü
from edcon.edrive.com_modbus import ComModbus
from sensor_msgs.msg import JointState

class LinearAxisActionServer(Node):
    def __init__(self):
        super().__init__('linear_axis_action_server')
        self.get_logger().info("Linear Axis Action Server başlatildi.")
        qos_profile = QoSProfile(depth=10)
        # Publisher (konum, hız, durum bilgisi için)
        self.publisher = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.timer = self.create_timer(0.1, self.publish_status)

        self.com = ComModbus(ip_address="192.168.4.1", timeout_ms=1000)             # ComModbus nesnesini oluştur, IP adresini ve timeout'u ayarla
        print("ComModbus nesnesi oluşturuldu.")
        self.position_scaling = 1 / 10 ** self.com.read_pnu(11724, 0)               # Scaling faktörünü ayarla, read_pnu=-6
        self.velocity_scaling = 1 / 10 ** self.com.read_pnu(11725, 0)               # Scaling faktörünü ayarla, read_pnu=-3
                                                                                    # Position_scaling=1,000,000
                                                                                    # Velocity_scaling=1,000
        self.motion_handler = MotionHandler(self.com, config_mode="write")          # MotionHandler (sürücü kontrolü için)
        print("MotionHandler nesnesi oluşturuldu.")
        self.motion_handler.base_velocity=self.com.read_pnu(12345, 0)
        self.motion_handler.acknowledge_faults()                                    # Sürücünün aktifleştirilmesi için
        self.motion_handler.enable_powerstage()
        
        # Action server
        self._action_server = ActionServer(
            self,
            MoveAxis,
            'move_axis',
            self.execute_callback
        )


    def execute_callback(self, goal_handle):                                        # Sürücüyü hareket ettirmek için çağrılır
        self.get_logger().info(f"Yeni hedef alindi: {goal_handle.request.target_position}")
        try:
            self.velocity=(1*self.velocity_scaling)/10
            goal_handle.request.target_position = (goal_handle.request.target_position * self.position_scaling)
            print("Hedef konum: ", goal_handle.request.target_position)
            print("Hedef hiz: ", self.velocity)
            self.motion_handler.position_task(
                int(goal_handle.request.target_position),int(self.velocity), absolute=True, nonblocking=False)
            goal_handle.succeed()
            result = MoveAxis.Result()
            result.success = True
            return result
        except Exception as e:
            self.get_logger().error(f"Hareket sirasinda hata: {e}")
            goal_handle.abort()
            result = MoveAxis.Result()
            result.success = False
            return result

    def publish_status(self):                                                       # ROS2 tarafına hız, konum ve durum bilgisi yayını 
        try:
            position = self.motion_handler.current_position()
            position = position * self.position_scaling
            position = position / 10**10
            velocity = str(self.motion_handler.current_velocity())
            # print("Position: ", position)
            # print("Velocity: ", velocity)
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            joint_state_msg.name = ["linear_axis_moving_joint"]
            joint_state_msg.position = [position/100]
            joint_state_msg.velocity = [float(velocity)]

            self.publisher.publish(joint_state_msg)
        except Exception as e:
            self.get_logger().error(f"Durum yayimlanirken hata: {e}")

    def cleanup(self):
            self.get_logger().info("Temizlik yapiliyor: Powerstage kapatiliyor...")
            try:
                self.motion_handler.disable_powerstage()
            except Exception as e:
                self.get_logger().error(f"Powerstage kapatma hatasi: {e}")

def main(args=None):
    print("Ana fonksiyon basladi")
    rclpy.init(args=args)
    node = LinearAxisActionServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Ctrl+C ile durduruldu")
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()
main()
