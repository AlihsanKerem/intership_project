# Projenin Amacı
Bu proje birden fazla otonom aracın ortak bir haritaya sahip olması için çevrelerinde gördükleri engelleri algılayıp birbirleri ile iletişim halinde kendi ortak dünya haritalarını oluşturmasasını amaçlıyor.

# Sistem Yapısı
<pre>
internship_project
├── src
│   ├── <a href="src/interfaces/README.md">interfaces</a>              # Özel mesaj tipleri
│   ├── <a href="src/robots/README.md">robots</a>                  # Simülasyon ortamı, robot modelleri ve nodeları
│   ├── <a href="src/laser_detection/README.md">laser_detection</a>         # Lidar verilerini işleme ve engel bulma
│   └── <a href="src/shared_world_model/README.md">shared_world_model</a>      # Farklı araçlardan gelen verileri birleştirip ortak dünya haritası oluşturma
</pre>

# Kodları Çalıştırmak
* **`ros2 launch robots spawn_robots.launch.py`**: Simülasyon dünyasını ve robot modellerini başlatır.
* **`ros2 run laser_detection obstacle_extractor_node`**: Laser scan verilerini işler ve engelleri algılar.
* **`ros2 run shared_world_model shared_world_model_node`**: Farklı araçlardan gelen verileri birleştirir.
* **`./start.sh`**: Yukarıdaki 3 komutu tek seferde çalıştırır.

# Docker Başlatma
* **`docker-compose up -d`**: Docker'ı başlatmak için.
* **`docker exec -it internship_project_container bash`**: Çalışan Docker'ın içine girmek için.
* **`docker ps`**: Çalışan Docker'ları listelemek için.
* **`docker compose down`**: Açık olan Docker'ları kapatmak için.

# Yapılacaklar
* [X] Robotların temel bir modeli
* [X] Gazebo ortamını oluşturmak
* [X] Robotların hareketini basitçe yapabilmek
* [X] Algoritmaları düzgünce yazmak (denedim)
* [X] Araçların şeklini değiştirmek gerekiyor çünkü ileri geri yapınca düşüyorlar
* [] Birden fazla araç olmalı ve bu araçlar birbirleri ile ayırt edilebilmeli
* [X] Visualization marker eklenebilir
* [] Projedeji ilerlemeye göre algoritmalarada yapılan değişikliklere göre dökümantasyonu güncellemem gerekiyor. Şuanda ilk yaptığım çözüme göre hazırladım ama daha sonrasında SLAM veya 3D Lidar a geçersem ona göre güncellemem gerekiyor.
### Düşünülen Algoritma Geliştirmeleri
* [ ] SLAM Entegrasyonu (Statik Harita vs Dinamik Engeller): Ortamın statik bir haritasını çıkararak eğer ortamda obje varsa bunu static haritadaki duvarlarla kıyaslayarak aha bu objedir demesini sağlamak.
* [ ] 3D Lidar (Point Cloud) ile Sensör Yükseltmesi: 2D lidarın duvar ve obje algılamadaki problemini 3. boyutu kullanarak çözmeye çalışmak. Bunun için laserscan verisinden pointcloud verisine geçicez.
* [ ] Çoklu Robot (Multi-Agent) ile Ortak Dünya Haritası (Sensor Fusion): Birden fazla robotun tespit ettiği objeleri tek bir Haritada birleştirmek.
