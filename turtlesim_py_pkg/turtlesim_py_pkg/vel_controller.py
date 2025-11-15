#!/usr/bin/env python3
# Yukarıdaki satır Python yorumlayıcısının yolunu belirtir (Unix sistemlerinde gereklidir)

import sys                           # Terminalden argüman almak için sys modülü
import rclpy                         # ROS2 Python istemcisi
from rclpy.node import Node          # ROS2 node oluşturmak için temel sınıf
from geometry_msgs.msg import Twist  # TurtleSim'in hareket mesajı


class VelControllerNode(Node):
    # Bu sınıf ROS2 Node'u temsil eder

    def __init__(self):
        # Node adı "vel_controller_node" olarak ayarlanır
        super().__init__("vel_controller_node")

        # Publisher oluşturulur: Twist mesajını /turtle1/cmd_vel konusuna yayınlar
        self.publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

        # Her 1 saniyede bir publisher_vel fonksiyonu çalıştırılır
        self.timer = self.create_timer(1.0, self.publisher_vel)

        # Node başlatıldığında terminale bilgi yazdırılır
        self.get_logger().info("Velocity Controller Node has been started!")

    def publisher_vel(self):
        # Timer tarafından düzenli olarak çağrılan fonksiyon
        
        msg = Twist()  # Gönderilecek hız mesajı oluşturulur

        # Terminal argümanlarının sayısını kontrol et
        # Beklenen kullanım:
        # ros2 run turtlesim_py_pkg vel_controller <linear_x> <radius>
        if len(sys.argv) < 3:
            self.get_logger().error("Eksik argüman! Kullanım: ros2 run turtlesim_py_pkg vel_controller <linear_x> <radius>")
            return
        
        # Komut satırından alınan hız ve yarıçap değerleri okunur
        linear_x = float(sys.argv[1])  # Lineer hız (m/s)
        radius = float(sys.argv[2])    # Dönüş yarıçapı (m)

        # Lineer hız verilir
        msg.linear.x = linear_x
        msg.linear.y = 0.0             # Turtlesim için her zaman 0
        msg.linear.z = 0.0

        # Açısal hız hesaplanır: w = v / r
        msg.angular.z = linear_x / radius

        # Mesaj yayınlanır
        self.publisher_.publish(msg)

        # Yayınlanan değerler terminale yazdırılır
        self.get_logger().info(
            f"Publishing -> linear_x: {linear_x:.2f}, radius: {radius:.2f}, angular_z: {msg.angular.z:.2f}"
        )


def main(args=None):
    # ROS2'yi başlatır
    rclpy.init(args=args)

    # Node oluşturulur
    node = VelControllerNode()

    # Node çalışmaya devam eder (callback'leri dinler)
    rclpy.spin(node)

    # Node kapatılır
    rclpy.shutdown()


# Bu dosya direkt çalıştırıldığında main() fonksiyonu çağrılır
if __name__ == "__main__":
    main()
