# Paketin Amacı
Bu paket, projenin simülasyon ortamını ve robot kontrollerini barındırır. Robotların fiziksel tasarımlarını (URDF/Xacro), test edildikleri Gazebo dünyasını (`test_obstacles.world`) ve robotu hareket ettirmek için kullanılan manuel klavye kontrolcüsünü `Teleop` içerir.

# Published Topics
* **`/cmd_vel`** (`geometry_msgs/msg/Twist`): Robotun hız ve yön komutlarını (ileri/geri, sağa/sola dönüş) yayınlar. Bu topic `teleop_wasd` düğümü (node) tarafından klavye tuşlarına (W,A,S,D) basıldığında tetiklenir.
* **`/robot_description`** (`std_msgs/msg/String`): Robotun xacro (URDF) dosyasından okunan fiziksel ve görsel özelliklerini ağ üzerinde yayınlar. `robot_state_publisher` tarafından kullanılır.

# Önemli Dosyalar
* **`robot_designs/`**: Robotun tekerlek, şasi ve sensör gibi fiziksel özelliklerinin tanımlandığı URDF (Xacro) dosyalarını barındırır.
* **`robots/teleop_wasd.py`**: Kullanıcının terminal üzerinden W, A, S, D tuşlarıyla robotu manuel olarak kontrol etmesini sağlayan nodedur.
* **`worlds/`**: Gazebo'da robotun etrafını algılaması için test amacıyla oluşturulmuş engelleri (kutu, silindir vb.) barındıran dünya dosyalarını içerir.

