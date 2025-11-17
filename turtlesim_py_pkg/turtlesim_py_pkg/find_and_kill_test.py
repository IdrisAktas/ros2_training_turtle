#!/usr/bin/env python3

import math
from functools import partial

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist          # turtle1'e hız komutu göndermek için
from turtlesim.msg import Pose               # turtle1'in anlık pozisyon bilgisi
from turtlesim.srv import Kill               # Herhangi bir turtle'ı öldürmek için servis

from turtlesim_interfaces.msg import Turtle       # Tek bir turtle bilgisini tutan custom msg
from turtlesim_interfaces.msg import TurtleArray  # Birden fazla turtle bilgisini tutan custom msg


class FindAndKillNode(Node):
    """
    Bu node, turtle1'i bir 'avcı' gibi kullanarak,
    SpawnTurtleNode tarafından spawn edilen turtle'ları tek tek hedefleyip
    onlara doğru gidiyor ve yeterince yaklaştığında Kill servisini çağırarak yok ediyor.
    """

    # ----------------------------
    # 0. __init__: Node’un başlatılması, yapılandırılması
    # ----------------------------


    # ---------------------------------------------------------
    # FUNCTION: __init__
    # Node’u başlatır, tüm ROS bileşenlerini (publisher, subscriber, service client, timer)
    # kurar ve avcı–hedef takip/yok etme sistemi için gerekli verileri hazırlar.
    # Node adı "find_and_kill_node" olarak ayarlanır. turtle1’in anlık pozisyonunu tutmak
    # için hunter_pose isimli bir Pose nesnesi oluşturulur. Spawn edilen diğer kaplumbağaların
    # tutulduğu turtles_dict isimli sözlük (anahtar: turtle adı, değer: Turtle mesajı) ve
    # öldürülmüş kaplumbağaların isimlerini tutan killed_turtles isimli set tanımlanır.
    # current_target değişkeni, o anda takip edilen hedef Turtle nesnesini gösterecek şekilde
    # başta None olarak ayarlanır. turtle1’e hız komutu göndermek için /turtle1/cmd_vel
    # topiğine bir Twist publisher oluşturulur; turtle1’in anlık pozisyon bilgisini almak
    # için /turtle1/pose topiğine Pose subscriber tanımlanır. SpawnTurtleNode’dan gelen
    # tüm turtle listesini dinlemek üzere /new_turtles topiğine TurtleArray subscriber
    # kurulur. Herhangi bir turtle’ı yok etmek için turtlesim’in /kill servisine erişen
    # Kill service client oluşturulur. Son olarak her 0.1 saniyede bir çağrılacak olan
    # control_loop fonksiyonunu tetikleyen bir timer oluşturulur ve böylece node, yeni
    # turtle’ları takip etmeye, turtle1’in pozisyonunu güncellemeye ve hedefleri otonom
    # olarak avlamaya hazır hâle gelir.
    # ---------------------------------------------------------



    def __init__(self):
        # Node adını "find_and_kill_node" olarak ayarla
        super().__init__('find_and_kill_node')

        # Hedefe yaklaşma toleransları
        self.pose_threshold = 0.4          # Hedefe olan mesafe 0.4'ten küçükse "yeterince yaklaştı" kabul edilir
        self.pose_threshold_angular = 0.05 # Açı hatası (radyan) 0.05'ten küçükse "yeterince doğru yöne bakıyor" kabul edilir

        
        self.hunter_pose = Pose()# Ana Turtle (Avcı) : turtle1'in anlık pozisyonu burada tutulur

        # Takip edilecek turtle'ların tutulduğu yapı
        # turtles_dict: {"turtle2": Turtle(msg), "turtle3": Turtle(msg), ...}
        self.turtles_dict = {}   # Anahtar: turtle adı, Değer: Turtle mesajı (x,y,theta, name). dict obje gibidir isme göre filtreleme yapılabilir.
        self.killed_turtles = set()  # Öldürülmüş turtle isimlerini set içinde saklıyoruz (hızlı lookup için). set tekrarlı veriyi engeller ve sadece ismi tutar
        self.current_target: Turtle | None = None  # Şu anda peşinde olunan hedef (Turtle msg), yoksa None

        # turtle1'e hız komutu göndermek için publisher oluştur
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # turtle1'in pozisyonunu dinleyen subscriber
        # /turtle1/pose topiğine yeni Pose mesajı geldikçe callback_hunter_pose çağrılır
        self.hunter_pose_sub = self.create_subscription(Pose,'/turtle1/pose',self.callback_hunter_pose,10)

        # SpawnTurtleNode tarafından yayınlanan tüm turtle listesini dinleyen subscriber
        # /new_turtles topiğinden TurtleArray gelir, callback_new_turtles ile alınır
        self.new_turtles_sub = self.create_subscription(TurtleArray,'/new_turtles',self.callback_new_turtles,10)
        

        # Kill servisi için client oluştur (turtlesim'in kendi Kill servisi)
        self.kill_client = self.create_client(Kill, '/kill')

        # Ana kontrol döngüsü: her 0.1 saniyede bir control_loop fonksiyonu çağrılır
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Find & Kill Node has been started!')

    # ----------------------------
    # 1. CALLBACK: Hunter (turtle1) pose callback
    # ----------------------------



    # ---------------------------------------------------------
    # FUNCTION: callback_hunter_pose
    # /turtle1/pose topiğine yeni bir Pose mesajı geldiğinde otomatik olarak çağrılan
    # callback fonksiyonudur. Gelen msg, turtle1’in o andaki konumunu (x, y) ve yönünü
    # (theta) içerir. Bu fonksiyon, msg’nin içeriğini doğrudan self.hunter_pose içine
    # kopyalayarak avcı turtle1’in en güncel pozisyon bilgisini saklar. Böylece ana kontrol
    # döngüsü olan control_loop fonksiyonu her çağrıldığında turtle1’in konumu ve yönü
    # açısından en son değerler kullanılır; bu da mesafe ve açı hesabının doğru yapılmasını
    # sağlar. Fonksiyonun görevi sadece pozisyonu güncel tutmak olup, herhangi bir hareket
    # ya da karar alma işlemi yapmaz.
    # ---------------------------------------------------------




    def callback_hunter_pose(self, msg: Pose):
        """
        /turtle1/pose topiğinden gelen her yeni Pose mesajında çağrılır.
        turtle1'in en güncel x, y, theta bilgilerini self.hunter_pose içine kaydeder.
        Böylece control_loop fonksiyonu her zaman en güncel konumu kullanır.
        """
        self.hunter_pose = msg

    # ----------------------------
    # 2. CALLBACK: Yeni turtle listesi callback
    # ----------------------------


    # ---------------------------------------------------------
    # FUNCTION: callback_new_turtles
    # SpawnTurtleNode her yeni turtle oluşturduğunda, mevcut tüm turtle’ları içeren bir
    # TurtleArray mesajını /new_turtles topiği üzerinden yayınlar. Bu callback fonksiyonu,
    # gelen TurtleArray mesajındaki turtles listesini dolaşır. Liste içindeki her Turtle
    # nesnesi için önce isminin "turtle1" olup olmadığını kontrol eder; turtle1, avcı
    # turtle olduğu için asla hedef listesine eklenmez. Daha sonra turtle’ın adı
    # self.turtles_dict sözlüğünde yoksa ve self.killed_turtles setinde (yani daha önce
    # öldürülmüşler listesinde) bulunmuyorsa, bu turtle yeni bir hedef adayı olarak
    # self.turtles_dict içine eklenir. Böylece node, sahnede bulunan ve henüz
    # öldürülmemiş tüm kaplumbağaların güncel bir listesini elinde tutar. Bu liste,
    # ilerleyen aşamada hedef seçimi ve kovalamaca için kullanılır.
    # ---------------------------------------------------------


    def callback_new_turtles(self, msg: TurtleArray):
        """
        SpawnTurtleNode her yeni turtle oluşturduğunda, tüm turtle listesini TurtleArray
        olarak tekrar yayınlar. Burada gelen listedeki turtle'ları dolaşıp:
          - Eğer turtle1 ise (avcı) onu listeye almayız.
          - Daha önce hiç görmediğimiz ve öldürmediğimiz turtlesa sözlüğe ekleriz.
        Böylece self.turtles_dict içinde hedeflenebilecek tüm canlı turtle bilgilerini tutmuş oluruz.
        """
        for turtle in msg.turtles:
            # turtle1'i asla hedef yapma (avcı)
            if turtle.name == 'turtle1':
                continue

            # Eğer isim daha önce kaydedilmemişse ve öldürülmüş listesinde yoksa:
            if turtle.name not in self.turtles_dict and turtle.name not in self.killed_turtles:
                self.turtles_dict[turtle.name] = turtle
                self.get_logger().info(f'Yeni hedef kaydedildi: {turtle.name} (x={turtle.x:.2f}, y={turtle.y:.2f})')





    # ----------------------------
    # 3.Yardımcı fonksiyon: Açıyı [-pi, pi] aralığına normalize et
    # ----------------------------



    # ---------------------------------------------------------
    # FUNCTION: normalize_angle
    # Bu yardımcı fonksiyon, verilen bir açı değerini -pi ile +pi aralığına normalize
    # etmek için kullanılır. Robotikte açı farklılıklarının kontrol edilmesinde,
    # açının bu aralıkta tutulması, "en kısa dönüş yönünün" seçilebilmesi için önemlidir.
    # Eğer angle değeri pi’den büyükse, fonksiyon angle’dan 2*pi çıkararak bu değeri
    # aralığa çeker; eğer açı -pi’den küçükse, 2*pi ekleyerek aralığa geri getirir.
    # Sonuç olarak, fonksiyon her koşulda -pi ile +pi arasında bir açı değeri döndürür.
    # Bu sayede turtle1’in hedefe dönmesi için gereken açıyı hesaplarken gereksiz
    # 360°’lik tam tur dönüşlerden kaçınılır ve kontrol daha kararlı çalışır.
    # ---------------------------------------------------------


    
    def normalize_angle(self, angle):
        """
        Açı değerini -pi ile +pi arasına sıkıştırmak için kullanılır.
        Örneğin 4 rad -> -2.283... gibi dönüştürülür.
        Bu sayede robot gereksiz yere büyük açılar dönmek yerine,
        en kısa yönü tercih eder.
        """
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    # ----------------------------
    # 4. Yardımcı fonksiyon: Sıradaki hedefi seç
    # ----------------------------


    # ---------------------------------------------------------
    # FUNCTION: select_next_target
    # Bu fonksiyon, o anda herhangi bir hedef (current_target) yoksa yeni bir hedef
    # seçmek için çağrılır. self.turtles_dict sözlüğü boşsa, sahnede hiç hedef
    # kalmadığı anlamına gelir ve fonksiyon herhangi bir işlem yapmadan döner. Eğer
    # sözlük boş değilse, içindeki turtle’lardan birinin adı (örneğin ilk eklenen)
    # alınır ve buna karşılık gelen Turtle nesnesi self.current_target değişkenine
    # atanır. Böylece avcı turtle1’in peşine düşeceği yeni bir hedef belirlenmiş olur.
    # Bu yöntem şu anda en basit haliyle "ilk eklenen turtle’ı hedefle" mantığını
    # kullanır; istenirse burası "en yakın turtle’ı seç", "en eski turtle’ı seç" gibi
    # daha gelişmiş stratejilerle değiştirilebilir. Fonksiyon hedef seçildiğinde bunu
    # log’a yazarak hangi turtle’ın takip edilmeye başlandığını bildirir.
    # ---------------------------------------------------------


    def select_next_target(self):
        """
        Eğer şu anda bir hedef yoksa (current_target None ise),
        turtles_dict içindeki turtle'lardan birini hedef olarak seçer.
        Burada en basit haliyle sözlükteki ilk turtle seçiliyor.
        (İstersen buraya "en yakın turtle'ı seç" gibi akıllı bir strateji ekleyebilirsin.)
        """
        if not self.turtles_dict:
            # Hiç turtle yoksa hedef seçemeyiz
            return

        # Sözlükteki ilk turtle'ı seç (order Python 3.7+ ile insert sırasını korur)
        next_name = list(self.turtles_dict.keys())[0]
        self.current_target = self.turtles_dict[next_name]
        self.get_logger().info(f'Yeni hedef seçildi: {self.current_target.name} 'f'(x={self.current_target.x:.2f}, y={self.current_target.y:.2f})')

    # ----------------------------
    # 5. Kill servisini çağıran fonksiyon
    # ----------------------------


    # ---------------------------------------------------------
    # FUNCTION: call_kill_service
    # Bu fonksiyon, verilen turtle_name’e sahip turtle’ı öldürmek (silmeyi) denemek için
    # turtlesim’in /kill servisine asenkron bir istek gönderir. İlk olarak kill_client
    # üzerinden servis hazır mı diye kontrol edilir; hazır değilse 1 saniye aralıklarla
    # tekrar kontrol edilerek servis hazır olana dek beklenir. Servis hazır olduğunda
    # bir Kill.Request nesnesi oluşturulur ve içine hedef turtle’ın adı yazılır.
    # Daha sonra call_async ile bu istek servise gönderilir ve dönen future nesnesine,
    # servis cevabı tamamlandığında çalışacak olan callback_kill_response fonksiyonu
    # add_done_callback ile bağlanır. partial kullanılarak turtle_name parametresi
    # callback fonksiyonuna sabitlenir, böylece cevap geldiğinde hangi turtle için
    # kill işlemi yapıldığı bilinebilir. Fonksiyonun kendisi, kill işleminin sonucunu
    # beklemez, sadece isteği başlatır.
    # ---------------------------------------------------------
    def call_kill_service(self, turtle_name):
        """
        turtlesim'in /kill servisini asenkron olarak çağırır.
        Verilen turtle_name'e sahip turtle'ı yok etmeye çalışır.
        Servis hazır değilse hazır olana kadar bekler.
        Cevap geldiğinde callback_kill_response çalışır.
        """
        # Servis hazır değilse 1 saniyede bir kontrol ederek bekle
        while not self.kill_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for Server - [Kill Turtle]')

        # Kill servis isteği oluştur
        request = Kill.Request()
        request.name = turtle_name

        # Asenkron servis çağrısı yap
        future = self.kill_client.call_async(request)

        # Cevap geldiğinde callback_kill_response fonksiyonunu çağır,
        # turtle_name bilgisini de partial ile sabitliyoruz.
        future.add_done_callback(
            partial(self.callback_kill_response, turtle_name=turtle_name)
        )


    # ----------------------------
    # 6. CALLBACK: Kill servisi
    # ----------------------------



    # ---------------------------------------------------------
    # FUNCTION: callback_kill_response
    # Bu fonksiyon, /kill servisine yapılan asenkron çağrının sonucunu işleyen
    # callback fonksiyonudur. Servis cevabı future.result() ile alınır; Kill servisi
    # herhangi bir veri gövdesi döndürmediği için sonuç içerik olarak kullanılmasa da,
    # işlemin başarılı olup olmadığını anlayabilmek için çağrılır. Eğer future.result()
    # çağrısı bir hata üretmezse, kill isteğinin başarıyla tamamlandığı log’a yazılır.
    # Eğer servis çağrısı sırasında bir istisna (exception) oluşursa, bu exception
    # yakalanır ve hata mesajı log’a basılır. Bu fonksiyon yalnızca kill servisinin
    # sonucunu raporlamakla görevli olup, hedef seçimi veya hareket mantığını
    # değiştirmez.
    # ---------------------------------------------------------

    def callback_kill_response(self, future, turtle_name):
        """
        Kill servisine yapılan isteğin sonucunu işleyen callback.
        Servis sonucu başarıyla gelirse log yazar, hata olursa exception yakalar.
        """
        try:
            _ = future.result()  # Kill servisinin cevabını alıyoruz (cevap gövdesi boş)
            self.get_logger().info(f'Kill servisi çağrısı tamamlandı: {turtle_name}')
        except Exception as e:
            self.get_logger().error(f'Kill servisi hata verdi: {e!r}')

    # ----------------------------
    # 6. ANA KONTROL DÖNGÜSÜ
    # ----------------------------

    # ---------------------------------------------------------
    # FUNCTION: control_loop
    # Bu fonksiyon, node içinde tanımlı timer tarafından her 0.1 saniyede bir çağrılan
    # ana kontrol döngüsüdür ve tüm "hedef bul, hedefe git, hedefi öldür" mantığını
    # yönetir. İlk adımda, eğer current_target None ise yani şu anda aktif bir hedef
    # yoksa, select_next_target fonksiyonu çağrılarak self.turtles_dict içinden yeni
    # bir hedef seçilir. Eğer hâlâ hedef bulunamazsa (turtles_dict boşsa), turtle1’in
    # hız komutları sıfırlanarak turtle1 durdurulur ve fonksiyon sona erer. Hedef
    # belirlendiyse, öncelikle hedefin (x, y) konumu ile avcı turtle1’in mevcut
    # konumu (hunter_pose.x, hunter_pose.y) arasındaki x ve y farkı hesaplanır ve bu
    # farklardan Öklid mesafesi (distance) elde edilir. Daha sonra hedefe doğru
    # bakılması gereken açı atan2(dist_y, dist_x) ile target_theta olarak bulunur;
    # turtle1’in mevcut yönü (hunter_pose.theta) ile target_theta arasındaki fark
    # normalize_angle fonksiyonu ile -pi ile +pi arasına çekilerek angle_error
    # olarak hesaplanır. Eğer açı hatası pose_threshold_angular değerinden büyükse,
    # turtle1’in önce hedefe dönmesi gerektiği kabul edilir ve Twist mesajının
    # angular.z bileşeni hata ile orantılı bir değer alırken (örneğin 2.0 * angle_error),
    # linear.x sıfırlanarak sadece dönme hareketi yapılır. Açı hatası küçük fakat
    # hedefe olan mesafe pose_threshold değerinden büyükse, artık turtle1’in hedefe
    # doğru ileri hareket etmesi istenir; bu durumda linear.x mesafe ile orantılı bir
    # ileri hız alır (örneğin 2.0 * distance) ve angular.z sıfırlanarak düz ileri
    # hareket sağlanır. Hem açı hatası küçük hem de mesafe pose_threshold altına
    # düştüğünde turtle1’in hedefe yeterince yaklaştığı kabul edilir, hızlar sıfırlanır,
    # hedefin adı alınarak call_kill_service fonksiyonu çağrılır ve ilgili turtle’ın
    # yok edilmesi için /kill servisine istek gönderilir. Aynı zamanda bu hedefin
    # ismi killed_turtles setine eklenir, turtles_dict sözlüğünden çıkarılır ve
    # current_target None yapılarak bir sonraki hedef için sistem hazır hâle getirilir.
    # Fonksiyonun sonunda oluşturulan Twist mesajı /turtle1/cmd_vel topiğine publish
    # edilerek turtle1’in konumuna uygun hareket etmesi sağlanır.
    # ---------------------------------------------------------
    def control_loop(self):
        """
        Bu fonksiyon her 0.1 saniyede bir çağrılır ve ana go-to-goal + öldürme
        mantığını içerir:
          1) Eğer şu an hedef yoksa yeni hedef seç.
          2) Açı hatası büyükse önce hedefe doğru dön.
          3) Açı uygun, mesafe uzaksa hedefe doğru ilerle.
          4) Hedefe yeterince yaklaşınca Kill servisini çağır ve bir sonraki hedefe geç.
        """
        msg = Twist()  # turtle1 için hız komutu oluşturulacak mesaj

        # Henüz bir hedef seçilmemişse (None ise), hedef seçmeye çalış
        if self.current_target is None:
            self.select_next_target()

            # Hâlâ hedef yoksa (turtles_dict tamamen boş olabilir)
            if self.current_target is None:
                # Bu durumda hiçbir turtle yok, turtle1'i durdur
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.cmd_vel_pub.publish(msg)
                return  # Fonksiyonu bitir

        # Buraya geldiysek elimizde bir hedef var demektir
        # Hedefin konumunu al
        target_x = self.current_target.x
        target_y = self.current_target.y

        # Avcı turtle1'in anlık konumunu al
        current_x = self.hunter_pose.x
        current_y = self.hunter_pose.y
        current_theta = self.hunter_pose.theta

        # Hedef ile turtle1 arasındaki x ve y farkı
        dist_x = target_x - current_x
        dist_y = target_y - current_y

        # İki nokta arasındaki Öklid mesafesi
        distance = math.sqrt(dist_x**2 + dist_y**2)

        # Hedefe doğru bakılması gereken açı (dünya koordinat sistemine göre)
        target_theta = math.atan2(dist_y, dist_x)

        # Hedef açısı ile turtle1'in mevcut açısı arasındaki fark
        angle_error = self.normalize_angle(target_theta - current_theta)

        # -----------------------------
        # 1) ÖNCE AÇIYI DÜZELT (HEADING CONTROL)
        # -----------------------------
        if abs(angle_error) > self.pose_threshold_angular:
            # Eğer açı hatası toleranstan büyükse, turtle1 önce hedefe dönmeli
            msg.angular.z = 2.0 * angle_error   # Açı hatasıyla orantılı bir dönüş hızı
            msg.linear.x = 0.0                  # Dönme sırasında ileri hareket yok

        # -----------------------------
        # 2) AÇI YETERİNCE İYİYSE, HEDEFE DOĞRU İLERLE
        # -----------------------------
        elif distance > self.pose_threshold:
            # Açı hatası küçükse (yani doğru yöne bakıyorsak), artık ileri gidebiliriz
            msg.linear.x = 2.0 * distance       # Mesafeye orantılı bir ileri hız
            msg.angular.z = 0.0                 # Düz ilerleme, ekstra dönme yok

        # -----------------------------
        # 3) HEM AÇI HATASI KÜÇÜK HEM MESAFE KÜÇÜKSE → HEDEFİ YAKALADIK
        # -----------------------------
        else:
            # Hedefe yeterince yaklaştık, turtle1'i durdur
            msg.linear.x = 0.0
            msg.angular.z = 0.0

            name = self.current_target.name
            self.get_logger().info(
                f'HEDEF YAKALANDI! {name} koordinat: ({target_x:.2f}, {target_y:.2f})'
            )

            # Kill servisini çağırarak belirtilen turtle'ı yok et
            self.call_kill_service(name)

            # Bu turtle'ı öldürülmüş listesine ekle
            self.killed_turtles.add(name)

            # Artık bu turtle hedef listesinden çıkarılmalı
            if name in self.turtles_dict:
                del self.turtles_dict[name]

            # Şu anki hedefi sıfırla, bir sonraki hedef için hazırlık
            self.current_target = None

        # Hesaplanan hız komutunu /turtle1/cmd_vel topiğine yayınla
        self.cmd_vel_pub.publish(msg)


def main(args=None):
    """
    ROS2'yi başlatır, FindAndKillNode node'unu oluşturur
    ve rclpy.spin ile callback'lerin çalışmasını sağlar.
    """
    rclpy.init(args=args)
    node = FindAndKillNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()







# """
# Bu kod, TurtleSim ortamında turtle1’i “avcı” yapan bir otonom takip ve yok etme (find & kill) düğümü kuruyor. 
# SpawnTurtleNode tarafından rastgele konumlarda spawn edilen tüm kaplumbağaların bilgileri, TurtleArray mesajı ile /new_turtles topiğinden bu node’a geliyor; 
# node, callback’te bu listeyi okuyup turtles_dict sözlüğüne yeni gördüğü (ve daha önce öldürmediği) turtle’ları isimleri ile kaydediyor. 
# Aynı anda /turtle1/pose topiğini dinleyerek avcı turtle1’in güncel x, y, theta değerleri self.hunter_pose içinde sürekli güncelleniyor. 
# Her 0.1 saniyede bir çalışan control_loop fonksiyonu ana beyin: 
# Eğer şu an takip edilen bir hedef yoksa (current_target None ise), sözlükten bir turtle seçip yeni hedef yapıyor; sonra hedefin konumu ile turtle1’in 
# konumu arasında mesafe (distance) ve hedefe doğru bakılması gereken açı (target_theta) hesaplanıyor, turtle1’in mevcut yönü (theta) ile hedef yönü arasındaki fark angle_error olarak normalize ediliyor. 
# Eğer açı hatası belirlenen eşikten büyükse (yani turtle1 hedefe tam bakmıyorsa) node sadece angular.z bileşenine açı hatasıyla orantılı bir dönme hızı verip turtle1’i yerinde döndürüyor; 
# açı hata toleransın altındaysa bu sefer mesafe kontrolü devreye giriyor, hedefe olan mesafe pose_threshold’dan büyükse linear.x mesafeyle orantılı bir ileri hız alıyor ve turtle1 
# düz bir şekilde hedefe doğru ilerliyor. Hem açı hatası küçük hem de mesafe eşik değerin altına düştüğünde node hedefin “yakalandığına” karar verip turtle1’i durduruyor, h
# edef turtle’ın adını alarak /kill servisine asenkron bir istek gönderiyor ve Kill servisi başarıyla dönerse bu turtle’ı killed_turtles setine ekleyip turtles_dict’ten 
# siliyor ve current_target’ı None yaparak bir sonraki hedef için hazır hale geliyor. Özetle bu kod, gelen tüm turtle listesini dinleyip yeni hedefleri sırayla seçen, 
# turtle1’i önce doğru yöne çevirip sonra hedefe koşturan ve yeterince yaklaştığında turtlesim’in Kill servisiyle o turtle’ı sahneden kaldıran tam bir “hedef bul, 
# kovala, yok et” kontrol algoritması gerçekleştiriyor.
# """