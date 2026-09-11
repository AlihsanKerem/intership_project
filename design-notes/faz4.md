# FAZ-4: Çoklu Robot İletişimi, TF Ağacı ve Konum Belirleme

## 1. Problem
Birden fazla robotun (örn. `robot_1`, `robot_2`) aynı Gazebo simülasyon ortamında bağımsız olarak çalışabilmesi; her robotun kendi sensör ve TF verilerini (`odom` -> `base_link` -> `laser_frame`) karışıklık olmadan yayınlaması, robotların hem kendi konumlarını hem de algıladıkları engellerin küresel koordinatlardaki yerini doğru hesaplayıp merkezi dünya modeline (ana beyine) iletebilmesi gereklidir.

## 2. Gereksinimler
- Birden fazla robot Gazebo simülasyonunda çakışmadan, farklı koordinatlarda spawn edilebilmeli.
- Her robot için topic ve frame ayrımı ROS 2 Namespace (`/robot_1`, `/robot_2`) ve TF prefix mekanizması ile sağlanmalı.
- Her robot kendi klavye kontrolcüsü (`teleop_wasd`) veya bağımsız komutlarla ayrı ayrı hareket ettirilebilmeli.
- Her robot kendi `laser_detection` (obstacle extractor) düğümünü çalıştırarak tespit ettiği engelleri kendi namespace'i altındaki `/robot_X/detected_obstacles` topiğinden yayınlamalı.
- Engellerin koordinatları robotun yerel sensör ekseninden ortak küresel referans çerçevesine (`map` veya ortak `odom` frame'ine) TF dönüşümüyle aktarılmalı.

## 3. Alternatifler
- **Çoklu Robot Mimarisi:**
  - ROS 2 Namespaces (`/robot_1/scan`, `/robot_2/scan`) ✅
  - Düğüm Adı Ön Ekleri (Node Prefixing)
- **Koordinat Çerçevesi (Global vs Local TF):**
  - Ortak `map` frame'i altında birleştirilmiş TF ağacı ✅
  - Bağımsız `odom` frame'leri + Başlangıç konumu ofset (initial pose offset) dönüşümü
- **İletişim Yapısı:**
  - Merkezi Node İletişimi (Robotlar -> Shared World Model Node) ✅
  - Dağıtık (Peer-to-Peer / Decentralized) İletişim

## 4. Başarı Kriteri
- Gazebo ortamında en az 2 robot başarıyla spawn olur ve birbirine çarpmadan ayrı ayrı hareket ettirilebilir.
- `/robot_1/scan` ve `/robot_2/scan` verileri birbirine karışmadan bağımsız okunur.
- Her iki robot da kendi çevresindeki engelleri kümeleyip `TrackedObjectArray` formatında yayınlar.
- RViz üzerinde her iki robotun konumu, TF ağacı ve algıladığı engel marker'ları ortak eksende hatasız görselleştirilir.

## 5. Sonuç
- ✅ **Başarıyla Tamamlandı.**
  - URDF/Xacro modellerine parametrik `prefix` desteği eklendi.
  - `spawn_multi_robots.launch.py` ile `robot_1` ve `robot_2` farklı koordinatlarda ve bağımsız namespace'ler altında Gazebo'ya spawn edildi.
  - Ortak `world` ekseni üzerinden Static TF bağlantıları kuruldu.
  - `teleop_wasd` düğümüne `robot_name` parametresi eklenerek bağımsız hareket sağlandı.
  - `laser_detection` paketi `TrackedObjectArray` çıktısı üretecek şekilde güncellendi ve `detect_obstacles.launch.py` ile iki robot için ayrı ayrı çalıştırıldı.
  - RViz2 üzerinde iki robotun modelleri, Lidar taramaları ve engel marker'ları ortak `world` ekseninde doğrulandı.

---

## 6. Adım Adım Yapılacaklar

### 🔹 1. Adım: URDF / Xacro Parametrik TF Prefix Desteği
1. Robot URDF/Xacro dosyasına `robot_name` veya `prefix` parametresi ekle.
2. Link ve joint isimlerini `$(arg prefix)_base_link`, `$(arg prefix)_laser_frame` şeklinde dinamik hale getir.
3. Gazebo plugins (diferansiyel sürüş ve lidar eklentileri) için topic ve frame adlarını bu parametreye bağla.

### 🔹 2. Adım: Çoklu Robot Spawn Launch Dosyası (`spawn_robots.launch.py`)
1. Ana launch dosyasında her robot için bir `GroupAction` ve `PushRosNamespace` bloğu oluştur.
2. Her robotu Gazebo'da farklı başlangıç koordinatlarına ($x_1, y_1$, $x_2, y_2$) yerleştir (`spawn_entity.py` ile `-x`, `-y`, `-entity` argümanları).
3. Her robot için ayrı bir `robot_state_publisher` düğümünü ilgili namespace altında başlat.

### 🔹 3. Adım: Bağımsız Kontrol ve Teleop Ayrımı
1. Her robotun hız topiğini (`/robot_1/cmd_vel`, `/robot_2/cmd_vel`) ayrı yapılandır.
2. İstenen robotu kontrol edebilmek için `teleop_wasd` düğümüne hedef namespace parametresi (örn. `robot_name:=robot_1`) desteği kazandır.

### 🔹 4. Adım: Algılama (Obstacle Extractor) Düğümlerinin Çoklu Robota Entegrasyonu
1. `obstacle_extractor_node` düğümünü her robotun namespace'i altında çalışacak şekilde launch dosyasına dahil et.
2. Düğümün giriş topiğini `/robot_X/scan`, çıkış topiğini `/robot_X/detected_obstacles` olarak remap et.
3. TF dinleyicisinin (`tf_listener`) ilgili robotun TF zincirini (`robot_X_base_link` -> `robot_X_laser`) doğru çözmesini sağla.

### 🔹 5. Adım: RViz2 Çoklu Robot Konfigürasyonu
1. RViz üzerinde her iki robotun `RobotModel`, `LaserScan`, `TF` ve `MarkerArray` görüntüleyicilerini ekle.
2. Sabit çerçeveyi (Fixed Frame) ortak `odom` veya `map` olarak ayarla.

---

## 7. Araştırma ve Öğrenme Kaynakları

Kodu yazarken ve çoklu robot mimarisini kurarken araştırabileceğin başlıklar ve terimler:

1. **ROS 2 Multi-Robot ve Namespacing:**
   - Arama terimi: `ROS 2 multi robot gazebo spawn namespace launch example`
   - Doküman: [ROS 2 Launch Multi-Node and Namespaces](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html)

2. **URDF / Xacro Parametreleri ve TF Prefix:**
   - Arama terimi: `xacro arguments robot name prefix gazebo ros2`
   - Doküman: [ROS 2 URDF Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html)

3. **Gazebo Diff Drive & Lidar Plugin Remapping:**
   - Arama terimi: `gazebo_ros_diff_drive namespace remapping ros2`
   - Arama terimi: `gazebo_ros_ray_sensor ros2 humble namespace`

4. **Multi-Robot TF Ağacı Yönetimi:**
   - Arama terimi: `ROS 2 tf2 multi robot frame_id mapping`
   - Doküman: [tf2 Tutorials - Adding a Frame](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Adding-A-Frame-Cpp.html)
