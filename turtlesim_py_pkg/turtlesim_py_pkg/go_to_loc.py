#!/usr/bin/env python3
# Bu satır Python yorumlayıcısının yolunu belirtir (Unix sistemleri için)

import sys               # Terminalden parametre almak için
import math              # Mesafe ve açı hesaplamaları için

import rclpy             # ROS2 Python istemcisi
from rclpy.node import Node
from turtlesim.msg import Pose     # TurtleSim konum mesajı
from geometry_msgs.msg import Twist  # Hareket komutu

class GoToLocationNode(Node):
    def __init__(self):
        super().__init__('go_to_location_node')

        # Hedef konuma yaklaşma toleransı
        # Pozisyon ve açı için ayrı eşikler tanımlanabilir
        self.pose_threshold = 0.3
        self.pose_threshold_angular = 0.03

        # Varsayılan hedef konum (terminalden de alınabilir)
        self.target_x = 2.0      # Default hedef x
        self.target_y = 11.0     # Default hedef y
        # INFO: Eğer terminalden iki değer gelirse onları alacağız

        # Publisher → hareket komutlarını göndermek için
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Subscriber → turtle1/pose dinleyip güncel konumu almak için
        self.subscriber = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.callback_turtle_pose,
            10
        )

        # 0.1 saniyede bir turtle_controller fonksiyonu çağrılır
        self.timer = self.create_timer(0.1, self.turtle_controller)

        self.pose = Pose()  # Güncel pozisyon bilgisini tutar

        self.get_logger().info("Go To Location Node has been started!")

        # Terminalden hedef pozisyon alınmış mı kontrol et
        if len(sys.argv) >= 3:
            self.target_x = float(sys.argv[1])
            self.target_y = float(sys.argv[2])
            self.get_logger().info(f"New target set from terminal: x={self.target_x}, y={self.target_y}")
        else:
            self.get_logger().info(f"Using default target: x={self.target_x}, y={self.target_y}")

    # TurtleSim'den gelen pose mesajını burada güncelliyoruz
    def callback_turtle_pose(self, msg):
        self.pose = msg

    # TurtleSim'i hedef konuma götüren kontrol fonksiyonu
    def turtle_controller(self):
        msg = Twist()

        # Hedef ile mevcut pozisyon arasındaki fark
        dist_x = self.target_x - self.pose.x
        dist_y = self.target_y - self.pose.y

        # Gerçek mesafe
        distance = math.sqrt(dist_x**2 + dist_y**2)

        # Hedef konuma dönmek için gerekli açı
        target_theta = math.atan2(dist_y, dist_x)

        # Açı farkı
        angle_error = target_theta - self.pose.theta

        # 1️⃣ Önce dönme kontrolü (Heading Control)
        if abs(angle_error) > self.pose_threshold_angular:
            # Robot önce hedefe doğru dönemiyorsa sadece açısal hız verilir
            msg.angular.z = angle_error
            msg.linear.x = 0.0

        else:
            # 2️⃣ Robot doğru yöne baktığında ileri hareket
            if distance >= self.pose_threshold:
                msg.linear.x = distance
                msg.angular.z = 0.0
            else:
                # Hedefe ulaşıldı!
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.get_logger().info("SUCCESS! Target Reached!")

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GoToLocationNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
