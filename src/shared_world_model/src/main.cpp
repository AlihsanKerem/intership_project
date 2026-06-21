#include <rclcpp/rclcpp.hpp>
#include "shared_world_model/world_model_node.hpp" // Yazdığımız Node sınıfı

int main(int argc, char * argv[])
{
    // ROS 2 iletişim sistemini başlat
    rclcpp::init(argc, argv);

    // WorldModelNode sınıfından bir nesne (object) oluştur
    // std::make_shared, C++'ta güvenli bir şekilde hafıza(pointer) ayırmamızı sağlar
    auto node = std::make_shared<shared_world_model::WorldModelNode>();

    // rclcpp::spin, düğümü "sonsuz bir döngüye" sokar. 
    rclcpp::spin(node);

    // Ctrl+C ile program kapatılırsa ROS 2'yi güvenli şekilde kapat
    rclcpp::shutdown();
    
    return 0;
}
