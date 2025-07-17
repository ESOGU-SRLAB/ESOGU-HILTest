#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float64  # JointPositionController'dan gelen komutlar için
from sensor_msgs.msg import JointState
import threading
import time

# Festo EdCon kütüphanesi importları (sizin reposunuzdan)
from edcon.edrive.motion_handler import MotionHandler
from edcon.edrive.com_modbus import ComModbus

class LinearAxisControllerAdapter(Node):
    def __init__(self):
        super().__init__('linear_axis_controller_adapter')
        self.get_logger().info("Linear Axis Controller Adapter başlatıldı.")

        # Parametreleri tanımla (launch dosyasından override edilebilir)
        self.declare_parameter('axis_id', 1)
        self.declare_parameter('ip_address', '192.168.4.1') # Festo EdCon IP adresi
        self.declare_parameter('port', 502) # Festo EdCon Modbus TCP portu
        self.declare_parameter('joint_name', 'linear_axis_moving_joint')
        self.declare_parameter('update_frequency_hz', 125.0) # Festo'dan okuma ve ROS'a yayınlama frekansı
        self.declare_parameter('command_send_frequency_hz', 20.0) # Festo'ya komut gönderme frekansı

        self.axis_id = self.get_parameter('axis_id').get_parameter_value().integer_value
        self.ip_address = self.get_parameter('ip_address').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.joint_name = self.get_parameter('joint_name').get_parameter_value().string_value
        self.update_frequency_hz = self.get_parameter('update_frequency_hz').get_parameter_value().double_value
        self.command_send_frequency_hz = self.get_parameter('command_send_frequency_hz').get_parameter_value().double_value

        self.current_position_meters = 0.0  # Lineer eksenin mevcut pozisyonu (metre)
        self.target_position_meters = 0.0   # Lineer eksenin hedef pozisyonu (metre)
        self.velocity_festo_units_per_s = 0 # Festo'ya gönderilecek hız (dahili birim)

        self.com = None
        self.motion_handler = None
        self.position_scaling = 1.0 # Festo dahili birimleri / CM
        self.velocity_scaling = 1.0 # Festo dahili birimleri / (CM/s)
        self.connected_to_festo = False


        # Festo EdCon entegrasyonu
        try:
            self.com = ComModbus(ip_address=self.ip_address, timeout_ms=1000)
            self.get_logger().info(f"ComModbus nesnesi oluşturuldu. IP: {self.ip_address}")

            # Scaling faktörlerini oku (PNU değerleri direkt üs olarak kabul edildi)
            pnu_pos_scaling_exp = self.com.read_pnu(11724, 0)
            pnu_vel_scaling_exp = self.com.read_pnu(11725, 0)
            
            # Örneğin, PNU -6 döndürdüyse, scaling faktörü 10^6 olmalı
            self.position_scaling = 10 ** (-pnu_pos_scaling_exp) 
            self.velocity_scaling = 10 ** (-pnu_vel_scaling_exp)
            
            self.get_logger().info(f"PNU 11724 (pos_scaling_exp): {pnu_pos_scaling_exp}, hesaplanan position_scaling (Festo_units/CM): {self.position_scaling}")
            self.get_logger().info(f"PNU 11725 (vel_scaling_exp): {pnu_vel_scaling_exp}, hesaplanan velocity_scaling (Festo_units/(CM/s)): {self.velocity_scaling}")

            self.motion_handler = MotionHandler(self.com, config_mode="write")
            self.get_logger().info("MotionHandler nesnesi oluşturuldu.")

            # Hız parametresini oku (varsayılan hız için)
            base_velocity_pnu = self.com.read_pnu(12345, 0)
            self.motion_handler.base_velocity = base_velocity_pnu
            self.velocity_festo_units_per_s = base_velocity_pnu # Varsayılan hızı Festo birimlerinde al
            self.get_logger().info(f"Festo base velocity (PNU 12345): {base_velocity_pnu}")


            self.motion_handler.acknowledge_faults()
            self.motion_handler.enable_powerstage()
            self.get_logger().info("Festo EdCon powerstage etkinleştirildi.")

            # İlk pozisyonu oku ve hedefi mevcut pozisyona ayarla
            initial_position_festo_units = self.motion_handler.current_position()
            self.current_position_meters = (initial_position_festo_units / self.position_scaling) # Festo -> CM -> Metre
            self.get_logger().info(f"Başlangıç lineer eksen pozisyonu: {self.current_position_meters:.4f} metre")
            self.connected_to_festo = True
            self.target_position_meters=self.current_position_meters

        except Exception as e:
            self.get_logger().error(f"Festo EdCon bağlantısı veya başlatma hatası: {e}")
            self.com = None # Bağlantı kurulamadıysa nesneleri None yap
            self.motion_handler = None
            self.connected_to_festo = False

        # ROS2 Subscriptions ve Publishers
        # JointPositionController'dan gelen hedef pozisyon komutlarını dinle
        self.subscription = self.create_subscription(
            JointState,
            f'/linear_axis_joint_commands', # JointPositionController'ın varsayılan komut topiki
            self.command_callback,
            10
        )
        self.subscription  # unused variable uyarısını engelle

        # JointState mesajlarını yayınla (robot_state_publisher ve MoveIt2 için)
        qos_profile = QoSProfile(depth=10)
        self.joint_state_publisher = self.create_publisher(JointState, '/linear_axis_joint_states', qos_profile)
        self.status_timer = self.create_timer(1.0 / self.update_frequency_hz, self.publish_status)

        # Festo'ya hareket komutlarını göndermek için timer
        self.command_timer = self.create_timer(1.0 / self.command_send_frequency_hz, self.send_motion_command)

        self.get_logger().info(f'Linear axis adapter node başlatıldı. Joint: {self.joint_name}')

    def command_callback(self, msg: JointState):
        """
        TopicBasedSystem'dan gelen JointState mesajını alır.
        Sadece kendi ekleminin (linear_axis_moving_joint) pozisyon komutunu ayrıştırır.
        """
        try:
            # Kendi ekleminin indeksini bul
            joint_index = msg.name.index(self.joint_name)
            if joint_index < len(msg.position):
                self.target_position_meters = msg.position[joint_index]
                self.get_logger().info(f'Alınan hedef pozisyon (metre) from JointState: {self.target_position_meters:.4f}')
            else:
                self.get_logger().warn(f"Pozisyon verisi '{self.joint_name}' eklemi için bulunamadı.")
        except ValueError:
            self.get_logger().warn(f"'{self.joint_name}' eklemi JointState mesajında bulunamadı.")
        except Exception as e:
            self.get_logger().error(f"JointState mesajını işlerken hata: {e}")

    def send_motion_command(self):
        """
        Hedef pozisyona hareket komutunu Festo EdCon'a gönderir.
        """
        # if not self.connected_to_festo or not self.motion_handler:
        #     self.get_logger().warn("Festo EdCon MotionHandler başlatılmadı veya bağlı değil. Komut gönderilemiyor.")
        #     return

        # Mevcut pozisyon ile hedef pozisyon arasında küçük bir fark varsa komut gönder
        # Örneğin, 0.5 mm'den büyük bir fark varsa hareket ettir
        if abs(self.target_position_meters - self.current_position_meters) > 0.0005: 
            try:
                # Hedef pozisyonu metre -> CM -> Festo dahili birimlere çevir
                target_position_cm = self.target_position_meters  # Metreyi CM'ye çevir
                target_position_festo_units = target_position_cm * self.position_scaling
                self.velocity_festo_units_per_s = self.velocity_scaling/10
                print(f"target_position_festo_units: {int(target_position_festo_units)}, velocity_festo_units_per_s: {int(self.velocity_festo_units_per_s)}")

                # nonblocking=True ile çağırıyoruz ki ROS2 döngüsü bloke olmasın
                # Hız olarak pnu'dan okunan varsayılan hızı kullanıyorum.
                self.motion_handler.position_task(
                    int(target_position_festo_units),
                    int(self.velocity_festo_units_per_s), 
                    absolute=True,
                    nonblocking=True 
                )
                self.get_logger().debug(f"Festo'ya gönderilen komut: Hedef={self.target_position_meters:.4f}m ({target_position_festo_units} Festo units), Hız={self.velocity_festo_units_per_s}")

            except Exception as e:
                self.get_logger().error(f"Festo EdCon'a hareket komutu gönderirken hata: {e}")
        # else:
        #     self.get_logger().debug("Hedefe yakın, komut gönderilmiyor.")


    def publish_status(self):
        """
        Lineer eksenin mevcut durumunu okur ve joint_states topikine yayınlar.
        """
        if not self.connected_to_festo or not self.motion_handler:
            return

        try:
            # Festo EdCon'dan güncel pozisyonu oku (Festo dahili birimlerde)
            current_position_festo_units = self.motion_handler.current_position()
            current_velocity_festo_units_per_s = self.motion_handler.current_velocity()

            # Festo dahili birimleri -> CM -> Metreye çevir
            self.current_position_meters = (current_position_festo_units / self.position_scaling)
            current_velocity_meters_per_s = (current_velocity_festo_units_per_s / self.velocity_scaling) / 100.0 # Hızı da dönüştür

            # JointState mesajı oluştur ve yayınla
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            joint_state_msg.name = [self.joint_name]
            joint_state_msg.position = [self.current_position_meters]
            joint_state_msg.velocity = [current_velocity_meters_per_s] 

            self.joint_state_publisher.publish(joint_state_msg)
            # self.get_logger().debug(f"Yayınlanan durum: Konum={self.current_position_meters:.4f}m, Hız={current_velocity_meters_per_s:.4f}m/s")

        except Exception as e:
            self.get_logger().error(f"Durum okunurken veya yayınlanırken hata: {e}")

    def cleanup(self):
        """
        Düğüm kapatılırken temizlik yapar.
        """
        self.get_logger().info("Temizlik yapılıyor: Powerstage kapatılıyor...")
        try:
            self.motion_handler.disable_powerstage()
            self.get_logger().info("Festo EdCon powerstage kapatıldı.")
        except Exception as e:
            self.get_logger().error(f"Powerstage kapatma hatası: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = LinearAxisControllerAdapter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl+C ile durduruldu.")
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()