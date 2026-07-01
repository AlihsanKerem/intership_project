#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <vector>
#include <cmath>

struct Point2D {
    double x;
    double y;
};

class ObstacleExtractor : public rclcpp::Node
{
public:

    ObstacleExtractor() : Node("obstacle_extractor_node")
    {
        // 1. Subscriber'ı oluşturuyoruz.
        // /scan topic'ini dinleyecek, tipi LaserScan olacak, kuyruk boyutu 10.
        // Veri geldiğinde scan_callback fonksiyonunu çalıştıracak.
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>
        (
            "/scan", 
            10,
            std::bind(&ObstacleExtractor::scan_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), 
        "Obstacle Extractor Node baslatildi. ");

    }

private:
        // 2. Callback (Geri Çağırma) Fonksiyonu
        // Lidar'dan her veri geldiğinde ROS2 otomatik olarak bu fonksiyonu çalıştırır.

        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
        {
            // Polar veriyi (r, teta) Kartezyene (x, y) çevirmek
            std::vector<Point2D> valid_points;
            double angle = msg->angle_min;

            for (size_t i = 0; i < msg->ranges.size(); i++) {
                double r = msg->ranges[i];

                // Geçersiz verileri engelliyoruz
                if (std::isinf(r) || std::isnan(r) ||
                    r < msg->range_min || r > msg->range_max) 
                    {
                        angle += msg->angle_increment;
                        continue;
                    }

                    // Formül: x = r * cos(aci), y = r * sin(aci)
                    Point2D pt;
                    pt.x = r * std::cos(angle);
                    pt.y = r * std::sin(angle);
                    valid_points.push_back(pt);

                    // Açıyı bir sonraki nokta için artır
                    angle += msg->angle_increment;
            }

            // Eger hic gecerli nokta yoksa fonksiyonu sonlandır
            if (valid_points.empty()) return;

            std::vector<std::vector<Point2D>> clusters;
            std::vector<Point2D> current_cluster;

            // İlk noktayı geçiyi kumeye koyuyoruz
            current_cluster.push_back(valid_points[0]); 
            // 30 cm. Aralarında 30 cm'den az olan noktalar aynı objedir!
            double threshold = 0.3;

            for (size_t i = 1; i < valid_points.size(); i++ ) {
                // Şimdiki nokta ile bir önceki nokta arasındaki uzaklığı (hipotenüs) hesaplıyoruz
                double dx = valid_points[i].x - valid_points[i-1].x;
                double dy = valid_points[i].y - valid_points[i-1].y;
                double distance = std::sqrt(dx*dx + dy*dy);

                if (distance < threshold) {
                    current_cluster.push_back(valid_points[i]);
                }
                else {
                    // Eski objeyi listeye kaydediyor
                    clusters.push_back(current_cluster);
                    // Geçici kümeyi temizliyor
                    current_cluster.clear();
                    // Yeni objenin ilk noktasını ekliyoruz
                    current_cluster.push_back(valid_points[i]);
                }
            }

            // Döngü bittikten sonra elimizde kalan son objeyi de ekliyoruz
            clusters.push_back(current_cluster);

            int object_id = 1;
            for (const auto& cluster : clusters) {
                // Çok küçük kümeleri yok sayıyoruz
                if (cluster.size() < 5) continue;

                // Merkez hesaplama
                double sum_x = 0, sum_y = 0;
                for (const auto& pt : cluster) {
                    sum_x += pt.x;
                    sum_y += pt.y;
                }
                double mean_x = sum_x / cluster.size();
                double mean_y = sum_y / cluster.size();

                // Kovaryans hesaplama
                double var_x = 0.0, var_y = 0.0, cov_xy = 0.0;
                for(const auto& pt : cluster) {
                    var_x += (pt.x - mean_x) * (pt.x - mean_x);
                    var_y += (pt.y - mean_y) * (pt.y - mean_y);
                    cov_xy += (pt.x - mean_x) * (pt.y - mean_y);
                }

                double N = cluster.size();
                var_x /= (N - 1);
                var_y /= (N - 1);
                cov_xy /= (N - 1);

                RCLCPP_INFO(this->get_logger(),
                    "Obje %d -> Merkez(X: %.2f, Y: %.2f) | Nokta Sayisi: %zu | VarX: %.4f, VarY: %.4f", 
                    object_id, mean_x, mean_y, cluster.size(), var_x, var_y);

                    object_id++;
            }
            



            RCLCPP_INFO(this->get_logger(),
            "Lidardan veri geldi. Toplam nokta sayisi: %zu", msg->ranges.size());

            RCLCPP_INFO(this->get_logger(),
            "Lidar taranmasi bitti. %zu adet obje bulundu!", clusters.size());
        }
        
        //Subscribe olunan topic burada tanımlanıyor
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
};

// Main
int main(int argc, char * argv[])
{
    // ROS2'yi başlatır
    rclcpp::init(argc, argv);

    // Bizim oluşturduğumuz Node'u çalıştırır ve sürekli dinlemede kalmasını sağlar (spin)
    rclcpp::spin(std::make_shared<ObstacleExtractor>());

    // ROS2'yi güvenli şekilde kapatır
    rclcpp::shutdown();
    return 0;
}
