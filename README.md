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
### Algoritma testleri
* [] Duvarların konumlarına göre duvarları engel olarak algılamamasını sağlamak
* [] SLAM ile harita oluşturup o harita üzerinde duvarlarla kıyaslama yapıp engelleri öyle anlamak
* [] 3D Lidar kullanarak duvarlarla engelleri birbirinden ayırmak
* [] Birden fazla araç algoritmayı kullanıp ortak bir dünya haritası oluşturmalı yani bu algoritmayı birden fazla araçla denemeliyiz

