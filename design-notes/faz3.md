# FAZ-3: Robot çevresinde kaç engel olduğunu nasıl engeller olduğunu anlamalı

## 1. Problem
2D lidar verisinden gelen nokta bulutlarından clustering yaparak robot kaç tane engel görüyor ve bu engellerin şekilleri nasıl bunu anlamalıyız

## 2. Gereksinimler
- Rviz ekranında nokta bulutu verileri gelip clustering ile tanınan engellerin etrafında farklı renkte oluşmalılar veya bir şekilde ayırt edilebilmeliler 
- Engellerin merkezlerini clustering ile bulmalıyız
- Kaç tane engel olduğunu bulmalıyız

## 3. Alternatifler
- **Kümeleme (Clustering):**
  - Euclidean Distance (Jump Distance) ✅
  - Adaptive Breakpoint Detector (Dinamik eşikli)
  - DBSCAN (Yoğunluk tabanlı)
- **Şekil Çıkarma (Bounding Box):**
  - AABB (Axis-Aligned Bounding Box - Eksenlere paralel)
  - OBB (Oriented Bounding Box - 2D PCA ile dönük kutu)

## 4. Başarı Kriteri
- RViz'de noktalar kümelere göre farklı renklerde görünür.
- Engellerin etrafında doğru boyutta kutular (AABB/OBB) ve merkez noktaları çizilir.
- Robot hareket etse de engellerin dünya (`odom`) konumları doğru kalır.

## 5. Sonuç
✅ **Faz-3 Başarıyla Tamamlandı!**
- Lidar verileri Polar -> Kartezyen dönüşümü ve TF senkronizasyonu ile `odom` frame'ine aktarıldı.
- Euclidean Jump Distance algoritması ile noktalar kümelendi, min/max nokta ve uzunluk filtreleri ile duvar/gürültü ayrımı yapıldı.
- Kümeler renkli `sensor_msgs/msg/PointCloud2` formatında `/clustered_points` topiğinde yayınlandı.
- Her engelin ağırlık merkezi, AABB boyutları ve ID etiketleri `visualization_msgs/msg/MarkerArray` formatında `/obstacle_markers` topiğinde RViz üzerinde 3 boyutlu olarak başarıyla görselleştirildi.

---

## 6. Adım Adım Yapılacaklar

### 🔹 1. Adım: Polar -> Kartezyen ve TF Dönüşümü
1. Lidar `ranges[i]` mesafesini al.
2. Açıyı $angle = \text{angle\_min} + i \times \text{angle\_increment}$ olarak hesapla.
3. Lazer koordinatını bul: $x = r \times \cos(angle)$, $y = r \times \sin(angle)$.
4. Bu noktayı `tf2::doTransform` ile `odom` frame'ine taşı.

### 🔹 2. Adım: Kümeleme (Clustering)
1. Geçerli noktaları sırayla gez.
2. İki nokta arasındaki mesafe belirlediğin eşikten (örn. 0.3m - 0.5m) küçükse aynı kümeye ekle.
3. Mesafe büyükse eski kümeyi kaydet, yeni bir küme başlat.
4. **Filtreler:**
   - Çok az noktalı kümeleri (örn. < 4 nokta) sensör gürültüsü sayıp at.
   - Çok fazla noktalı veya çok geniş kümeleri (duvarlar) filtrele.

### 🔹 3. Adım: Renkli PointCloud2 Oluşturma (RViz'de Görmek İçin)
1. `sensor_msgs::msg::PointCloud2` mesajı oluştur.
2. `sensor_msgs::PointCloud2Modifier` ile mesaja `x, y, z, rgb` alanları ekle.
3. Her bir kümeye ayrı bir renk (Kırmızı, Yeşil, Mavi, Sarı vb.) vererek noktaları içine yaz.
4. `/clustered_points` topiğinde yayınla.

### 🔹 4. Adım: Merkez ve Bounding Box (AABB / OBB) Hesabı
1. **Merkez:** Kümedeki tüm $X$ ve $Y$'lerin ortalamasını al.
2. **AABB (Düz Kutu):** Kümedeki $x_{min}, x_{max}, y_{min}, y_{max}$ değerlerini bul. Genişlik = $x_{max} - x_{min}$, Yükseklik = $y_{max} - y_{min}$.
3. **OBB (Dönük Kutu - İsteğe Bağlı):** Kovaryans hesabı ile yön açısını ($\theta$) bulup engeli saran en küçük kutuyu oluştur.

### 🔹 5. Adım: RViz Marker'ları ile Kutuları Çizme
1. `visualization_msgs::msg::MarkerArray` oluştur.
2. Her küme için `Marker::CUBE` (veya `LINE_STRIP`) tipiyle kutu çizdir.
3. `Marker::SPHERE` ile merkez noktasını göster.
4. `Marker::TEXT_VIEW_FACING` ile engelin üzerine "ID: 1" gibi yazı koy.
5. `/obstacle_markers` topiğinde yayınla.

---

## 7. Araştırma ve Öğrenme Kaynakları

Kodu yazarken takıldığın yerlerde araştırabileceğin başlıklar ve terimler:

1. **TF Dönüşümü ve Lidar:**
   - Arama terimi: `ROS 2 tf2 transform PointStamped C++`
   - Doküman: [ROS 2 tf2 Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html)

2. **Lidar Kümeleme (Clustering):**
   - Arama terimi: `LaserScan point clustering algorithm C++`
   - Arama terimi: `Euclidean distance clustering 2D LiDAR`

3. **Renkli PointCloud2 Mesajı Doldurma:**
   - Arama terimi: `sensor_msgs PointCloud2Modifier PointCloud2Iterator example C++`
   - Doküman: ROS 2 `sensor_msgs` PointCloud2 API

4. **AABB ve OBB Bounding Box:**
   - Arama terimi: `Axis-Aligned Bounding Box 2D formula`
   - Arama terimi: `2D Oriented Bounding Box PCA covariance C++`

5. **RViz Marker Görselleştirme:**
   - Arama terimi: `visualization_msgs MarkerArray C++ ROS 2 example`
   - Doküman: [RViz Display Types - Markers](https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/Using-Markers/Using-Markers.html)
