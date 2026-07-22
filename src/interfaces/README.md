# Paketin Amacı
Bu paket, ROS 2 uygulamaları için gerekli olan özel mesaj tiplerinin (.msg) bulunduğu pakettir. Projedeki algılama (laser_detection) ve haritalama (shared_world_model) paketleri, birbirleriyle haberleşirken burada tanımlanan mesaj formatlarını kullanır.

# Özel Mesaj Tipleri
## 1. TrackedObject.msg
Algılanan tek bir nesnenin id numarasını, konumunu ve belirsizlik durumunu (kovaryans) ifade eder.
* `int32 id`: Algılanan nesneye atanan benzersiz id numarası.
* `float64[2] state`: Nesnenin haritadaki [x, y] koordinatları.
* `float64[4] covariance`: Sensör ölçümündeki belirsizliği temsil eden 2x2'lik kovaryans matrisi. (Sırasıyla: [var_x, cov_xy, cov_yx, var_y])
## 2. TrackedObjectArray.msg
Birden fazla `TrackedObject` mesajını bir array halinde gruplayarak tek seferde yayınlanmasını sağlar. Lidar'ın bir taramasında tespit edilen tüm objeler bu mesajın içinde gönderilir.
* `std_msgs/Header header`: Mesajın yayınlandığı zaman damgasını (timestamp) ve referans çerçevesini (frame_id) tutar.
* `TrackedObject[] objects`: Algılanan nesnelerin listesi.