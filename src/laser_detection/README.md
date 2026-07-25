# Paketin Amacı
Bu paket, robotun Lidar verilerini işleyerek çevresindeki nesneleri algılamasını sağlar. Gelen 2 boyutlu sensör verisini (mesafe ve açı), Kartezyen (x, y) koordinat sistemine dönüştürür ve noktaları birbirine olan uzaklıklarına göre kümeleyerek nesneleri tespit eder. Son olarak, tespit edilen her bir nesnenin merkezini ve konumundaki kovaryans değerlerini hesaplayarak diğer paketlerin kullanması için yayınlar.

# Subscribed Topics
* **`/scan`** (`sensor_msgs/msg/LaserScan`): Lidar sensöründen gelen ham mesafe verileri. Node bu veriyi okur ve işler.

# Published Topics
* **`/detected_obstacles`** (`interfaces/msg/TrackedObjectArray`): Lidar taraması sonucunda başarıyla tespit edilen nesnelerin listesi. Her bir nesne kendi id numarası, (x, y) konumu ve kovaryans matrisi ile birlikte yayınlanır.

# Parametreler
* **Distance Threshold: `0.5 metre`**
  Lidar'dan gelen iki nokta arasındaki mesafe 50 cm'den kısaysa, bu iki nokta aynı nesneye (kümeye) ait kabul edilir.
* **Min Cluster Size: `5 nokta`**
  Bir kümenin içerisinde 5'ten az Lidar noktası varsa, bu gürültü (noise) olarak kabul edilir ve nesne listesine eklenmez.
* **Max Cluster Size: `40 nokta`**
  Bir kümenin içerisinde 40'tan fazla nokta varsa, bu büyük ihtimalle bir engel değil, ortamdaki devasa bir duvar veya objedir. Bu yüzden duvarları haritadan filtrelemek için bu nesneler yok sayılır.
* **Algılama Sınırı:** Robotun `odom` çerçevesine göre X veya Y ekseninde `4.5` metreden daha uzakta olan noktalar işleme dahil edilmez (sınır dışı sayılır).
