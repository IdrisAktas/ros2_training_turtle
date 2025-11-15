#!/usr/bin/env python3   # Python çalışma yolu

import math              # Açı için (theta)
import random            # Rastgele x, y, theta üretmek için
from functools import partial   # Callback'e ekstra parametre geçirmek için

import rclpy             # ROS2 Python client
from rclpy.node import Node
from turtlesim.srv import Spawn   # TurtleSim spawn servisi


class SpawnTurtleNode(Node):

    # ----------------------------
    # FUNCTION: __init__: Node’u başlatır, zamanlayıcıyı kurar ve gerekli değişkenleri ayarlar.
    # ----------------------------
    def __init__(self):
        super().__init__("spawn_turtle_node")                           # Node adı
        self.name_ = "turtle"                                           # Oluşturulacak turtle ismi prefix
        self.counter_ = 1                                               # Kaçıncı turtle olduğunu sayar
        self.timer_ = self.create_timer(0.1, self.spawn_turtle)         # Her 1 saniyede bir spawn_turtle() fonksiyonu çağrılır
   

    # ----------------------------
    # FUNCTION: spawn_turtle: Spawn x,y ve theta tanımlamaları ve robot adı verileri tanımlanır.
    # ----------------------------
    def spawn_turtle(self):
        self.counter_ += 1                              # İsim sırasını artır
        turtle_name = self.name_ + str(self.counter_)   # Örn: turtle2, turtle3
        x = random.uniform(0.0, 8.0)                    # Rastgele X pozisyonu
        y = random.uniform(0.0, 8.0)                    # Rastgele Y pozisyonu
        
        theta =random.uniform(0.0, 2 * math.pi)         # Rastgele açı (radyan)

        # Spawn servisini çağır
        self.call_spawn_turtle_server(x, y, theta, turtle_name)
        self.get_logger().info("Kordinatlar: " + str(x)+ " --- "+ str(y))




    # ----------------------------
    # FUNCTION: call_spawn_turtle_server: Spawn servisinin hazır olmasını bekler ve verilen pozisyonla yeni turtle oluşturmak için asenkron servis isteği gönderir.
    # ----------------------------
    def call_spawn_turtle_server(self, x, y, theta, turtle_name):

        client = self.create_client(Spawn, "/spawn")   # Spawn servisine client oluştur

        while not client.wait_for_service(1.0):        #  Eğer servis hazır değilse 1 saniye boyunca bekler ve sonra tekrar dener.
            self.get_logger().warn("Waiting for Server - [Spawn Turtles]")

        request = Spawn.Request()       # İstek paketini oluştur
        request.x = x                   # Turtle x pozisyonu
        request.y = y                   # Turtle y pozisyonu
        request.theta = theta           # Turtle açısı
        request.name = turtle_name      # Turtle adı

        #Future:servis cevabıdır
        future = client.call_async(request)    # Asenkron servis isteği gönder. Servise isteği gönderir, ama Cevap gelene kadar beklemez.İsteği gönderdim, cevap gelince bu future’nin içine koyacağım

        # Servis cevabı geldiğinde callback çalıştır.
        # Çünkü ROS2 callback’leri sadece future parametresi gönderir; bizim ise ekstra parametrelere (x, y, theta, turtle_name) ihtiyacımız var.
        # partial, callback fonksiyonuna bu ek parametreleri önceden sabitleyerek göndermemizi sağlar.
        # eğer diğer değişkenler olmnasa future.add_done_callback(self.callback_call_spawn_turtle) olarak yazmamız yeterli
        future.add_done_callback(partial(self.callback_call_spawn_turtle,x=x, y=y, theta=theta, turtle_name=turtle_name  )) # Ek parametreler
        


    # ----------------------------
    # FUNCTION: callback_call_spawn_turtle: Spawn servisi yanıt verdiğinde çalışır ve yeni oluşturulan turtle’ın başarıyla yaratıldığını terminale yazar. x,y ve theta parametreleri kullanılmadı.
    # ----------------------------
    def callback_call_spawn_turtle(self, future, x, y, theta, turtle_name):
        try:
            response = future.result()          # Servis cevabını al
            if response.name != "":             # Başarılıysa turtle adı gelir
                self.get_logger().info("Turtle: " + response.name + " is created!")
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))
            # Servis çağrısı başarısız olursa hata bastırılır


# ----------------------------
# FUNCTION: main: ROS2'yi başlatır, node’u çalıştırır ve programı döngüye alır.
# ----------------------------
def main(args=None):
    rclpy.init(args=args)
    node = SpawnTurtleNode()
    rclpy.spin(node)            # Node'u çalıştır
    rclpy.shutdown()            # Kapat


if __name__ == "__main__":
    main()


    
    # ----------------------------
    # 1.FUNCTION: __init__: Node’u başlatır, zamanlayıcıyı kurar ve gerekli değişkenleri ayarlar.
    # ----------------------------


    # ----------------------------
    # 2.FUNCTION: spawn_turtle: Spawn x,y ve theta tanımlamaları ve robot adı verileri tanımlanır.
    # ----------------------------

    # ----------------------------
    # 3.FUNCTION: call_spawn_turtle_server: Spawn servisinin hazır olmasını bekler ve verilen pozisyonla yeni turtle oluşturmak için asenkron servis isteği gönderir.
    # ----------------------------

    # NOT: 3. ve 4. fonksiyonun ayrı olması lazım yani ikisi aynı fonksiyona birleştirilemez çünkü Aynı fonksiyonda iki işi birden yapamazsın, çünkü cevap anında gelmez
    # ✔ İki fonksiyon olması gerekli, çünkü biri servisi çağırır, diğeri ise cevap geldiğinde çalışır
    # ✔ Tek fonksiyonda yapmak mümkün değildir çünkü call_async() sonuçları anında döndürmez.

    
    # ----------------------------
    # 4.FUNCTION: callback_call_spawn_turtle: Spawn servisi yanıt verdiğinde çalışır ve yeni oluşturulan turtle’ın başarıyla yaratıldığını terminale yazar. x,y ve theta parametreleri kullanılmadı.
    # ----------------------------


    # ----------------------------
    # 5.FUNCTION: main: ROS2'yi başlatır, node’u çalıştırır ve programı döngüye alır.
    # ----------------------------