# Projenin Amacı
Birden fazla robotun çevresinde gördükleri nesneleri algılayarak hepsinin sahip olduğu ortak bir dünya haritası oluşturmak. 
# Klasörü Yapısı

# Docker Başlatma
Docker'ı başlatmak
* docker-compose up -d

Çalışan Docker'ın içine girmek için
* docker exec -it internship_project_container bash

Çalışan Docker'ları listelemek
* docker ps

Açık olan Docker'ları kapatmak için
* docker compose down

# Yapılacaklar

* [X] Robotların temel bir modeli
* [X] Gazebo ortamını oluşturmak
* [X] Robotların hareketini basitçe yapabilmek
* [X] Algoritmaları düzgünce yazmak (denedim)
* [X] Araçların şeklini değiştirmek gerekiyor çünkü ileri geri yapınca düşüyorlar
* [] Birden fazla araç olmalı ve bu araçlar birbirleri ile ayırt edilebilmeli
* [X] Visualization marker eklenebilir
### Algoritma Geliştirmeleri
* [ ] SLAM Entegrasyonu (Statik Harita vs Dinamik Engeller): Ortamın statik bir haritasını çıkararak eğer ortamda obje varsa bunu static haritadaki duvarlarla kıyaslayarak aha bu objedir demesini sağlamak.
* [ ] 3D Lidar (Point Cloud) ile Sensör Yükseltmesi: 2D lidarın duvar ve obje algılamadaki problemini 3. boyutu kullanarak çözmeye çalışmak. Bunun için laserscan verisinden pointcloud verisine geçicez.
* [ ] Çoklu Robot (Multi-Agent) ile Ortak Dünya Haritası (Sensor Fusion): Birden fazla robotun tespit ettiği objeleri tek bir Haritada birleştirmek.
