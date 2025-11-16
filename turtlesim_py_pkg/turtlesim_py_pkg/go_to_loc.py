#!/usr/bin/env python3
# Bu satır Python yorumlayıcısının yolunu belirtir (Unix sistemleri için)

import sys               # Terminalden parametre almak için
import math              # Mesafe ve açı hesaplamaları için

import rclpy             # ROS2 Python istemcisi
from rclpy.node import Node
from turtlesim.msg import Pose     # TurtleSim konum mesajı
from geometry_msgs.msg import Twist  # Hareket komutu




class GoToLocationNode(Node):



    # ----------------------------
    # 1.FUNCTION: spawn_turtle: Node’u başlatır,varsayılan konun ve  zamanlayıcıyı kurar ve gerekli değişkenleri ayarlar.
    # ----------------------------
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
        self.subscriber = self.create_subscription(Pose,'/turtle1/pose',self.callback_turtle_pose,10)

        # 0.1 saniyede bir turtle_controller fonksiyonu çağrılır
        self.timer = self.create_timer(0.1, self.turtle_controller)

        self.pose = Pose()  # Güncel pozisyon bilgisini tutar

        self.get_logger().info("Go To Location Node has been started!")

        # Terminalden hedef pozisyon alınmış mı kontrol et. 1. parametre node adı(otomatik istiyor pyton) 2. x 3. y.
        if len(sys.argv) >= 3:
            self.target_x = float(sys.argv[1])
            self.target_y = float(sys.argv[2])
            self.get_logger().info(f"New target set from terminal: x={self.target_x}, y={self.target_y}")
        else:
            self.get_logger().info(f"Using default target: x={self.target_x}, y={self.target_y}")


    # ----------------------------
    # 2.FUNCTION: callback_turtle_pose: TurtleSim'den gelen pose mesajını burada güncelliyoruz. Call back  fonksiyonu yazmamız gerekiyor çünkü subscriber oluşturduk ve onu dinlemek için sürekli çağırılıyor
    # ----------------------------
    def callback_turtle_pose(self, msg):
        self.pose = msg




    # ----------------------------
    # 3.FUNCTION: turtle_controller: TurtleSim'i hedef konuma götüren kontrol fonksiyonu
    # ----------------------------
    def turtle_controller(self):
        msg = Twist()   # hız mesajı oluştur ancak hız yerine konum bilgisi gönderilir yani bu similasyon olduğu için konuma göre hız veriyoruz.
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

        # turtle_controller fonksiyonu her 0.1 saniyede bir çalışarak kaplumbağayı hedefe doğru yönlendiren
        # kontrol mekanizmasını uygular: İlk olarak Twist tipinde bir msg oluşturulur ve mevcut konum (self.pose)
        # ile hedef konum (self.target_x, self.target_y) arasındaki x ve y farkları hesaplanır; bu farklardan
        # Öklid mesafesi (distance) bulunur ve hedefe doğru bakılması gereken yön açısı atan2 ile target_theta
        # olarak elde edilir. Daha sonra robotun şu an baktığı açı (self.pose.theta) ile hedef açısı arasındaki
        # fark angle_error olarak hesaplanır. Eğer bu açı farkı belirlenen eşikten (pose_threshold_angular) büyükse,
        # robot önce hedefe doğru dönsün diye sadece açısal hız (msg.angular.z = angle_error) verilir ve ileri hız
        # sıfırlanır (msg.linear.x = 0.0). Açı hatası yeterince küçükse, bu sefer hedefe olan mesafeye bakılır; mesafe
        # eşikten (pose_threshold) büyükse robot hedefe doğru yaklaşsın diye ileri hız distance değeriyle ayarlanır
        # (msg.linear.x = distance) ve dönme durdurulur (msg.angular.z = 0.0). Hem açı hatası küçük hem de mesafe
        # eşik değerin altına düşmüşse robotun hedefe ulaştığı kabul edilir, hem lineer hem açısal hızlar 0 yapılır
        # ve "SUCCESS! Target Reached!" şeklinde log basılır. Son olarak hesaplanan bu Twist mesajı publisher
        # aracılığıyla /turtle1/cmd_vel topiğine gönderilerek kaplumbağanın hareket etmesi sağlanır.



def main(args=None):
    rclpy.init(args=args)
    node = GoToLocationNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()




# ----------------------------
# 1.FUNCTION: spawn_turtle: Node’u başlatır,varsayılan konun ve  zamanlayıcıyı kurar ve gerekli değişkenleri ayarlar.
# ----------------------------


# ----------------------------
# 2.FUNCTION: callback_turtle_pose: TurtleSim'den gelen pose mesajını burada güncelliyoruz. Call back  fonksiyonu yazmamız gerekiyor çünkü subscriber oluşturduk ve onu dinlemek için sürekli çağırılıyor
# ----------------------------


# ----------------------------
# 3.FUNCTION: turtle_controller: TurtleSim'i hedef konuma götüren kontrol fonksiyonu
# ----------------------------


# Bu kod, TurtleSim içindeki kaplumbağayı verilen hedef konuma otomatik olarak götüren bir
# "go-to-goal" kontrol düğümüdür. Node çalıştığında /turtle1/pose topiğini dinleyerek kaplumbağanın
# güncel konum ve yön bilgisini alır ve her 0.1 saniyede bir turtle_controller() fonksiyonu ile hedefe
# olan mesafeyi ve gerekli dönüş açısını hesaplar. Kaplumbağa önce hedefe doğru bakmıyorsa yalnızca
# açısal hız verilir (angular.z), doğru yöne dönünce ileri hareket başlar (linear.x). Kaplumbağa hedefe
# yeterince yaklaştığında hızlar sıfırlanır ve "SUCCESS! Target Reached!" mesajı basılır. Hedef koordinatlar
# terminalden alınabilir veya varsayılan (2.0, 11.0) değerleri kullanılır. Böylece TurtleSim içinde basit bir
# otonom hedefe gitme davranışı gerçekleştirilmiş olur.




#sys.argv[0] → "go_to_loc"         (program adı)
#sys.argv[1] → "5"                 (1. parametre)
#sys.argv[2] → "8"                 (2. parametre)