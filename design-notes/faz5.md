# FAZ-5: Ortak Dünya Haritası ve Sensör Füzyonu

## 1. Problem
Farklı robotlardan (veya aynı robotun ardışık taramalarından) gelen engel verilerinin (`TrackedObjectArray`) tek bir küresel ortak dünya modelinde (Shared World Model) birleştirilmesi gerekir. Aynı fiziksel engele ait ölçümlerin tespit edilmesi (Data Association), ölçüm belirsizliklerinin (kovaryans) kaynaştırılması (Sensor Fusion) ve yeni engellerin haritaya eklenerek dinamik ve tutarlı bir harita oluşturulması hedeflenir.

## 2. Gereksinimler
- Robotlardan gelen `/robot_X/detected_obstacles` mesajlarını dinleyen merkezi bir `shared_world_model_node` düğümü olmalı.
- **Veri İlişkilendirme (Data Association):** Gelen yeni ölçümler ile mevcut küresel haritadaki engeller arasındaki mesafe ve belirsizlik Mahalanobis Uzaklığı ile hesaplanmalı.
- **Optimum Eşleştirme:** Maliyet matrisi Hungarian (Munkres) Algoritması ile çözülerek 1'e 1 en uygun eşleşmeler bulunmalı.
- **Kovaryans Kesişimi (Covariance Intersection - CI):** Eşleşen engellerin eski harita konumu ve kovaryansı ile yeni gelen ölçümün konum ve kovaryansı kaynaştırılarak belirsizlik azaltılmalı.
- **Harita Yönetimi:** Eşleşmeyen yeni ölçümler yeni ID atanarak küresel haritaya eklenmeli.
- **Görselleştirme:** Ortak dünya haritasındaki tüm engeller `visualization_msgs/msg/MarkerArray` formatında `/world_map_markers` topiği üzerinden RViz'de 3D olarak yayınlanmalı.

## 3. Alternatifler
- **Veri İlişkilendirme (Data Association):**
  - Öklid Mesafesi (Nearest Neighbor - Sadece mesafe odaklı)
  - Mahalanobis Uzaklığı + Hungarian Algoritması (Belirsizlik ve varyans odaklı) ✅
  - Joint Compatibility Branch and Bound (JCBB)
- **Sensör Füzyonu (Sensor Fusion):**
  - Basit Ağırlıklı Ortalama (Ağırlıklı konum hesabı)
  - Genişletilmiş Kalman Filtresi (EKF)
  - Covariance Intersection (CI - Bilinmeyen korelasyonlarda tutarlı füzyon) ✅
- **Görselleştirme Tipleri:**
  - `Marker::CUBE` / `Marker::CYLINDER` (Temel 3D Şekil) ✅
  - Kovaryans Elipsleri / Güven Aralıkları Çizimi
  - `Marker::TEXT_VIEW_FACING` (Küresel ID ve Güven Skoru) ✅

## 4. Başarı Kriteri
- İki farklı robot aynı engeli algıladığında haritada iki mükerrer engel yerine tek bir ortak engel kaydı oluşur.
- Eşleşen engellerin kovaryansı küçülür, konumu zamanla daha kararlı ve hassas hale gelir.
- Robotlar yeni bir alana girdiğinde yeni gördükleri engeller haritaya anında "Yeni Obje" olarak eklenir.
- RViz üzerinde `/world_map_markers` topiği üzerinden tek ve tutarlı bir küresel dünya haritası izlenebilir.

## 5. Sonuç
- *Geliştirme / Entegrasyon aşamasında* (Data Association, Mahalanobis, Hungarian ve Covariance Intersection algoritmalarının C++ uygulaması).

---

## 6. Adım Adım Yapılacaklar

### 🔹 1. Adım: Merkezi Düğüm Yapısı ve Subscriber Kurulumu
1. `shared_world_model` paketinde `shared_world_model_node` oluştur.
2. Tüm robotların tespit topiklerine (`/robot_1/detected_obstacles`, `/robot_2/detected_obstacles` veya birleştirilmiş `/detected_obstacles`) abone ol.
3. Küresel harita durumunu tutmak için `std::vector<TrackedObject>` tabanlı bellek içi harita yapısını oluştur.

### 🔹 2. Adım: Mahalanobis Uzaklığı ve Maliyet Matrisi
1. Haritadaki her obje $i$ ile yeni gelen her ölçüm $j$ arasındaki fark vektörünü hesapla: $d = z_j - x_i$.
2. Birleşik kovaryans matrisini oluştur: $S = P_i + R_j$.
3. Mahalanobis mesafesini hesapla:
   $$D_M = \sqrt{d^T S^{-1} d}$$
4. Belirlenen eşik değerinden (Gate Threshold, örn. 5.0m) büyük olan eşleşmeleri engellemek için maliyet matrisine sonsuz/büyük değer ata.

### 🔹 3. Adım: Hungarian Algoritması ile Eşleştirme
1. Oluşturulan $N \times M$ boyutundaki maliyet matrisini Hungarian (Munkres) algoritmasına ver.
2. Minimum toplam maliyetli 1'e 1 eşleşen çiftleri $(i, j)$ elde et.
3. Eşik değerini aşan sahte eşleşmeleri filtrele.

### 🔹 4. Adım: Covariance Intersection (CI) ile Füzyon
1. Eşleşen her çift için ağırlık parametresini ($\omega \in [0, 1]$, örn. 0.5) belirle.
2. Yeni birleşik kovaryansı hesapla:
   $$P_{new}^{-1} = \omega P_i^{-1} + (1 - \omega) R_j^{-1}$$
3. Yeni birleşik konum vektörünü hesapla:
   $$x_{new} = P_{new} \left( \omega P_i^{-1} x_i + (1 - \omega) R_j^{-1} z_j \right)$$
4. Haritadaki $i$ numaralı objenin konum ve kovaryansını bu yeni değerlerle güncelle.

### 🔹 5. Adım: Yeni Obje Ekleme ve Harita Yönetimi
1. Gelen ölçümlerden hiçbir harita objesiyle eşleşmeyenleri tespit et.
2. Bu ölçümlere benzersiz yeni bir küresel `id` ata ve global harita listesine ekle.
3. *(İsteğe bağlı)* Uzun süre hiçbir robot tarafından güncellenmeyen geçici/hayalet engelleri haritadan temizlemek için bir sayaç/yaşlandırma mekanizması ekle.

### 🔹 6. Adım: RViz3D Marker Görselleştirmesi
1. `visualization_msgs::msg::MarkerArray` oluştur.
2. Haritadaki her obje için `Marker::CUBE` veya `Marker::SPHERE` ile pozisyonu çizdir.
3. Her objenin üzerine `Marker::TEXT_VIEW_FACING` ile "Global ID: X" bilgisini yaz.
4. RViz'de anlık izlemek için `/world_map_markers` topiğinde yayınla.

---

## 7. Araştırma ve Öğrenme Kaynakları

Kodu yazarken takıldığın yerlerde araştırabileceğin başlıklar ve terimler:

1. **Mahalanobis Uzaklığı (Mahalanobis Distance):**
   - Arama terimi: `Mahalanobis distance 2D point covariance C++`
   - Arama terimi: `Data association in multi target tracking Mahalanobis gating`

2. **Hungarian (Munkres) Algoritması:**
   - Arama terimi: `Hungarian algorithm C++ implementation bipartite matching`
   - Doküman: [Kuhn-Munkres Algorithm Explained](https://en.wikipedia.org/wiki/Hungarian_algorithm)

3. **Covariance Intersection (CI) Algoritması:**
   - Arama terimi: `Covariance Intersection algorithm sensor fusion C++`
   - Doküman: Julier, S. J., & Uhlmann, J. K. *"A Non-divergent Estimation Algorithm in the Presence of Unknown Correlations"*

4. **ROS 2 Ortak Haritalama ve Sensor Fusion:**
   - Arama terimi: `ROS 2 multi robot map merging sensor fusion example`
   - Doküman: [ROS 2 Robot Localization & Fusion](https://docs.ros.org/en/humble/index.html)

5. **RViz MarkerArray ile Harita Görselleştirme:**
   - Arama terimi: `visualization_msgs MarkerArray C++ ROS 2 example`
   - Doküman: [RViz Display Types - Markers](https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/Using-Markers/Using-Markers.html)
