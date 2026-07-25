# Paketin Amacı
Bu paket projenin ana amacını yerine getirmektir. Amacı, robotlardan gelen farklı nesne algılamalarını tek bir **Küresel Ortak Dünya Haritasında (Shared World Map)** birleştirmektir. Bu node sayesinde, sistem ortamdaki yeni engelleri haritaya eklerken, önceden bilinen engellerin konumunu ve kovaryans değerlerini gelen yeni ölçümlerle birleştirerek daha hassas hale getirir.

# Subscribed Topics
* **`/detected_obstacles`** (`interfaces/msg/TrackedObjectArray`): `laser_detection` paketinden gelen ve algılanan nesneleri içeren veri.

# Published Topics
* **`/world_map_markers`** (`visualization_msgs/msg/MarkerArray`): Oluşturulan ortak dünya haritasındaki nesnelerin RViz ortamında görselleştirilebilmesi için yayınlanan 3 boyutlu marker verileri.

# Kullanılan Algoritmalar
Gelen yeni sensör ölçümleri, global haritaya entegre edilirken 3 temel adımdan geçer:

1. **Veri İlişkilendirme (Data Association) & Hungarian Algoritması:**
   Sistemde halihazırda kayıtlı olan harita nesneleri ile sensörden yeni gelen ölçümler birbirleriyle eşleştirilir. Bu eşleşme probleminde maliyet matrisi oluşturulurken **Mahalanobis Uzaklığı (Mahalanobis Distance)** kullanılır. Sadece fiziksel mesafeyi Öklid değil, sensör ölçümündeki belirsizliği (kovaryans) de hesaba katan Mahalanobis uzaklığı sayesinde çok daha güvenilir bir eşleştirme yapılır. Oluşturulan maliyet matrisi, en uygun eşleşmeyi (minimum maliyeti) bulmak için **Hungarian Algoritmasına** verilir.
   * *Parametre:* Eşleşme için maksimum mesafe eşiği `5.0 metre` olarak belirlenmiştir.
2. **Kovaryans Kesişimi (Covariance Intersection):**
   Eşleşen (aynı olduğu anlaşılan) nesnelerin eski konum ve kovaryans değerleri ile yeni sensör ölçümündeki konum ve kovaryans değerleri kaynaştırılır. Bu sayede belirsizlik azaltılır ve objenin haritadaki konumu daha stabil hale gelir.
   * *Parametre:* CI (Covariance Intersection) algoritmik ağırlık parametresi (omega) `0.5` olarak ayarlanmıştır.
3. **Harita Güncellemesi (Map Update):**
   Herhangi bir harita nesnesiyle eşleşmeyen (yani yeni görülen) ölçümler, global haritaya "Yeni Obje" olarak eklenir ve takip edilmeye başlanır.

