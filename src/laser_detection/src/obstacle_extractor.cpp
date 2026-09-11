#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <vector>
#include <cmath>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <interfaces/msg/tracked_object_array.hpp>
#include <interfaces/msg/tracked_object.hpp>

class LidarClusterNode : public rclcpp::Node
{
public:
    LidarClusterNode() : Node("lidar_cluster_node")
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // target_frame parametresi (varsayılan: "world")
        this->declare_parameter<std::string>("target_frame", "world");
        target_frame_ = this->get_parameter("target_frame").as_string();

        // Başına slash KOYMADAN göreceli topic'ler:
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&LidarClusterNode::scan_callback, this, std::placeholders::_1));

        cluster_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("clustered_points", 10);

        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("obstacle_markers", 10);

        // Ortak Beyin için TrackedObjectArray publisher'ı
        tracked_obj_pub_ = this->create_publisher<interfaces::msg::TrackedObjectArray>("detected_obstacles", 10);
        RCLCPP_INFO(this->get_logger(), "Obstacle Extractor Baslatildi! Hedef Frame: %s", target_frame_.c_str());
    }

private:
     struct Point {
        float x;
        float y;
    };
        struct Obstacle {
        Point center;      // Ağırlık merkezi (noktaların ortalaması)
        Point box_center;  // Kutunun geometrik merkezi
        double size_x;     // Kutunun X genişliği
        double size_y;     // Kutunun Y derinliği
    };

    // Kümeleme Parametreleri
    const double cluster_tolerance_ = 0.40; // 40 cm: İki nokta arasındaki mesafe bundan büyükse yeni engel kabul edilir.
    const size_t min_cluster_size_ = 5;    // En az 4 nokta: 1-2 noktalık lazer gürültülerini (noise) elemek için.
    const size_t max_cluster_size_ = 200;    // Duvar filtresi 1: Çok fazla noktalı büyük yapıları at
    const double max_cluster_length_ = 2.0; // Duvar filtresi 2: Boyu 2.0 metreden uzun olanları at


    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        // Zaman senkronizasyonu: TF ağacında odom -> laser_frame dönüşümü mevcut mu?
        if (!tf_buffer_->canTransform(target_frame_, scan_msg->header.frame_id, 
                                      scan_msg->header.stamp, rclcpp::Duration::from_seconds(0.1))) {
            RCLCPP_WARN(this->get_logger(), "TF dönüşümü henüz hazır değil.");
            return;
        }

        // Polar veriyi Cartesian PointCloud2'ye çevirme
        sensor_msgs::msg::PointCloud2 cloud_msg;
        projector_.projectLaser(*scan_msg, cloud_msg);

        // PointCloud2'yi tf2::doTransform ile odom frame'ine taşıma
        sensor_msgs::msg::PointCloud2 cloud_transformed;
        try {
            auto transform_stamped = tf_buffer_->lookupTransform(
                target_frame_, scan_msg->header.frame_id, scan_msg->header.stamp);
            
            tf2::doTransform(cloud_msg, cloud_transformed, transform_stamped);
        } 
        catch (const tf2::TransformException & ex) {
            RCLCPP_ERROR(this->get_logger(), "TF Hatası: %s", ex.what());
            return;
        }

        // Kümeleme
        auto clusters = cluster_points(cloud_transformed);
        RCLCPP_INFO(this->get_logger(), "Robot: %s , Tespit edilen engel (küme) sayısı: %zu", this->get_namespace(), clusters.size());

        // Header'ın frame_id'sini target_frame_ ("world") yapalım
        std_msgs::msg::Header target_header = scan_msg->header;
        target_header.frame_id = target_frame_;

        // Renkli olarak yayınlama
        publish_clusters(clusters, target_header);

        // Marker olarak yayınlama
        auto obstacles = extract_obstacles(clusters);
        publish_markers(obstacles, target_header);

        // Ortak Beyin için TrackedObjectArray yayınlama
        publish_tracked_objects(obstacles, target_header);



    }

    std::vector<std::vector<Point>> cluster_points(const sensor_msgs::msg::PointCloud2 & cloud_msg)
    {
        std::vector<std::vector<Point>> clusters;
        std::vector<Point> current_cluster;
        // PointCloud2 içerisindeki "x" ve "y" alanlarını okuyacak iterator'lar
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud_msg, "y");
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
            // NaN veya geçersiz değerleri kontrol et
            if (std::isnan(*iter_x) || std::isnan(*iter_y)) {
                continue;
            }
            Point current_point{*iter_x, *iter_y};
            if (current_cluster.empty()) {
                current_cluster.push_back(current_point);
            } else {
                Point last_point = current_cluster.back();
                // İki ardışık nokta arasındaki Öklid mesafesi: sqrt((x2-x1)^2 + (y2-y1)^2)
                double distance = std::hypot(current_point.x - last_point.x, current_point.y - last_point.y);
                if (distance <= cluster_tolerance_) {
                    // Mesafe küçük: Aynı engelin devamı, mevcut kümeye ekle
                    current_cluster.push_back(current_point);
                } else {
                    // Mesafe büyük: Engel bitti! Eski kümeyi kaydet ve yenisini başlat
                    if (is_valid_cluster(current_cluster)) {
                        clusters.push_back(current_cluster);
                    }
                    current_cluster.clear();
                    current_cluster.push_back(current_point);
                }
            }
        }
        // Döngü bittiğinde elde kalan son kümeyi de kontrol edip ekle
        if (is_valid_cluster(current_cluster)) {
            clusters.push_back(current_cluster);
        }
        return clusters;
    }

    bool is_valid_cluster(const std::vector<Point> & cluster)
    {
        // 1. Min / Max nokta sayısı kontrolü
        if (cluster.size() < min_cluster_size_ || cluster.size() > max_cluster_size_) {
            return false;
        }
        // 2. Kümenin boyutu / genişliği kontrolü (İlk nokta ile son nokta arasındaki mesafe)
        double length = std::hypot(cluster.back().x - cluster.front().x, 
                                   cluster.back().y - cluster.front().y);
        if (length > max_cluster_length_) {
            return false; // Çok uzun bir nesne (büyük ihtimalle duvar)
        }
        return true;
    }

    std::vector<Obstacle> extract_obstacles(const std::vector<std::vector<Point>> & clusters)
    {
        std::vector<Obstacle> obstacles;
        for (const auto & cluster : clusters) {
            if (cluster.empty()) continue;
            float sum_x = 0.0f, sum_y = 0.0f;
            float min_x = cluster[0].x, max_x = cluster[0].x;
            float min_y = cluster[0].y, max_y = cluster[0].y;
            for (const auto & pt : cluster) {
                sum_x += pt.x;
                sum_y += pt.y;
                min_x = std::min(min_x, pt.x);
                max_x = std::max(max_x, pt.x);
                min_y = std::min(min_y, pt.y);
                max_y = std::max(max_y, pt.y);
            }
            Obstacle obs;
            // 1. Ortalama Merkez
            obs.center.x = sum_x / cluster.size();
            obs.center.y = sum_y / cluster.size();
            // 2. Kutu Boyutları (Çok ince çizgi olmaması için min 0.1m pay veriyoruz)
            obs.size_x = std::max(0.1f, max_x - min_x);
            obs.size_y = std::max(0.1f, max_y - min_y);
            // 3. Kutu Merkezi
            obs.box_center.x = (min_x + max_x) / 2.0f;
            obs.box_center.y = (min_y + max_y) / 2.0f;
            obstacles.push_back(obs);
        }
        return obstacles;
    }

    void publish_clusters(const std::vector<std::vector<Point>> & clusters, const std_msgs::msg::Header & header)
    {
        if (clusters.empty()) {
            return;
        }
        // 1. Toplam nokta sayısını hesapla
        size_t total_points = 0;
        for (const auto & cluster : clusters) {
            total_points += cluster.size();
        }
        // 2. PointCloud2 mesajını hazırla
        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header = header;

        // 3. Modifier ile x, y, z ve rgb alanlarını ekle
        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
        modifier.resize(total_points);
        // 4. Iterator'ları tanımla
        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud_msg, "r");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud_msg, "g");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud_msg, "b");
        // Her küme için farklı bir renk paleti (RGB)
        const std::vector<std::tuple<uint8_t, uint8_t, uint8_t>> colors = {
            {255, 0, 0},     // Kırmızı
            {0, 255, 0},     // Yeşil
            {0, 0, 255},     // Mavi
            {255, 255, 0},   // Sarı
            {255, 0, 255},   // Magenta (Pembe)
            {0, 255, 255},   // Cyan (Camgöbeği)
            {255, 128, 0},   // Turuncu
            {128, 0, 255}    // Mor
        };
        // 5. Her kümeyi ayrı renkle mesajın içine yaz
        for (size_t cluster_idx = 0; cluster_idx < clusters.size(); ++cluster_idx) {
            auto [r, g, b] = colors[cluster_idx % colors.size()];
            for (const auto & pt : clusters[cluster_idx]) {
                *iter_x = pt.x;
                *iter_y = pt.y;
                *iter_z = 0.0f; // 2D lidar olduğu için Z = 0
                *iter_r = r;
                *iter_g = g;
                *iter_b = b;
                ++iter_x; ++iter_y; ++iter_z;
                ++iter_r; ++iter_g; ++iter_b;
            }
        }
        // 6. RViz için yayınla
        cluster_pub_->publish(cloud_msg);
    }

    void publish_markers(const std::vector<Obstacle> & obstacles, const std_msgs::msg::Header & header)
    {
        visualization_msgs::msg::MarkerArray marker_array;
        int id = 0;
        for (size_t i = 0; i < obstacles.size(); ++i) {
            const auto & obs = obstacles[i];
            // 1. Sınırlayıcı Kutu (CUBE)
            visualization_msgs::msg::Marker box_marker;
            box_marker.header = header;

            box_marker.ns = "obstacle_boxes";
            box_marker.id = id++;
            box_marker.type = visualization_msgs::msg::Marker::CUBE;
            box_marker.action = visualization_msgs::msg::Marker::ADD;
            box_marker.pose.position.x = obs.box_center.x;
            box_marker.pose.position.y = obs.box_center.y;
            box_marker.pose.position.z = 0.25; // Yerden biraz yükseltiyoruz
            box_marker.scale.x = obs.size_x;
            box_marker.scale.y = obs.size_y;
            box_marker.scale.z = 0.5; // Kutu yüksekliği
            box_marker.color.r = 0.0f;
            box_marker.color.g = 1.0f;
            box_marker.color.b = 0.0f;
            box_marker.color.a = 0.35f; // Yarı saydam yeşil
            box_marker.lifetime = rclcpp::Duration::from_seconds(0.15);
            marker_array.markers.push_back(box_marker);
            // 2. Merkez Noktası (SPHERE)
            visualization_msgs::msg::Marker sphere_marker;
            sphere_marker.header = header;

            sphere_marker.ns = "obstacle_centers";
            sphere_marker.id = id++;
            sphere_marker.type = visualization_msgs::msg::Marker::SPHERE;
            sphere_marker.action = visualization_msgs::msg::Marker::ADD;
            sphere_marker.pose.position.x = obs.center.x;
            sphere_marker.pose.position.y = obs.center.y;
            sphere_marker.pose.position.z = 0.5;
            sphere_marker.scale.x = 0.15;
            sphere_marker.scale.y = 0.15;
            sphere_marker.scale.z = 0.15;
            sphere_marker.color.r = 1.0f;
            sphere_marker.color.g = 0.0f;
            sphere_marker.color.b = 0.0f;
            sphere_marker.color.a = 1.0f; // Parlak kırmızı
            sphere_marker.lifetime = rclcpp::Duration::from_seconds(0.15);
            marker_array.markers.push_back(sphere_marker);
            // 3. Bilgi Yazısı (TEXT)
            visualization_msgs::msg::Marker text_marker;
            text_marker.header = header;

            text_marker.ns = "obstacle_labels";
            text_marker.id = id++;
            text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::msg::Marker::ADD;
            text_marker.pose.position.x = obs.center.x;
            text_marker.pose.position.y = obs.center.y;
            text_marker.pose.position.z = 0.8;
            text_marker.scale.z = 0.25; // Yazı boyutu
            text_marker.text = "ID: " + std::to_string(i + 1);
            text_marker.color.r = 1.0f;
            text_marker.color.g = 1.0f;
            text_marker.color.b = 1.0f;
            text_marker.color.a = 1.0f; // Beyaz
            text_marker.lifetime = rclcpp::Duration::from_seconds(0.15);
            marker_array.markers.push_back(text_marker);
        }
        marker_pub_->publish(marker_array);
    }

    void publish_tracked_objects(const std::vector<Obstacle> & obstacles, const std_msgs::msg::Header & header)
    {
        interfaces::msg::TrackedObjectArray array_msg;
        array_msg.header = header;
        for (size_t i = 0; i < obstacles.size(); ++i) {
            interfaces::msg::TrackedObject obj;
            obj.id = static_cast<int32_t>(i + 1);
            obj.state[0] = obstacles[i].center.x; // X koordinatı (world)
            obj.state[1] = obstacles[i].center.y; // Y koordinatı (world)
            
            // Başlangıç Kovaryansı (0.05 belirsizlik)
            obj.covariance[0] = 0.05; obj.covariance[1] = 0.0;
            obj.covariance[2] = 0.0;  obj.covariance[3] = 0.05;
            array_msg.objects.push_back(obj);
        }
        tracked_obj_pub_->publish(array_msg);
    }

    std::string target_frame_;
    rclcpp::Publisher<interfaces::msg::TrackedObjectArray>::SharedPtr tracked_obj_pub_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    laser_geometry::LaserProjection projector_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_pub_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarClusterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}