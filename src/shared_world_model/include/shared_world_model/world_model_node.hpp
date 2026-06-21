#ifndef SHARED_WORLD_MODEL__WORLD_MODEL_NODE_HPP_
#define SHARED_WORLD_MODEL__WORLD_MODEL_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <vector>

// Senin yazdığın matematiksel algoritmalar
#include "shared_world_model/types.hpp"
#include "shared_world_model/data_association.hpp"
#include "shared_world_model/covariance_intersection.hpp"

namespace shared_world_model
{

class WorldModelNode : public rclcpp::Node
{
public:
    WorldModelNode() : Node("shared_world_model_node")
    {
        // 1. ROS 2 Timer, Subscriber ve Publisher tanımlamaları buraya gelir.
        RCLCPP_INFO(this->get_logger(), "Shared World Model Node Baslatildi!");
    }

    // Sensörden her yeni veri geldiğinde bu fonksiyon tetiklenecek
    void processMeasurements(const std::vector<TrackedObject>& incoming_measurements)
    {
        // 1. VERİ İLİŞKİLENDİRME (Data Association)
        // Senin yazdığın associate fonksiyonunu hayal et (şu an pseudo-kod mantığında)
        // matched_pairs -> Bize hangi harita objesinin hangi sensör objesiyle eşleştiğini verir.
        std::vector<std::pair<int, int>> matched_pairs; 
        std::vector<int> new_measurement_indices; 
        
        // DataAssociation::associate(map_objects_, incoming_measurements, 5.0, matched_pairs, new_measurement_indices);

        // 2. KOVARYANS KESİŞİMİ (Covariance Intersection) - Eşleşenler için
        for (auto& pair : matched_pairs) {
            int map_idx = pair.first;
            int meas_idx = pair.second;

            
            TrackedObject fused_obj = CovarianceIntersection::fuseCI(
                map_objects_[map_idx], 
                incoming_measurements[meas_idx], 
                0.5 // omega
            );
            
            map_objects_[map_idx] = fused_obj; // Haritadaki objeyi güncelle (Harita Güncellemesi)
        }

        // 3. YENİ OBJELERİ HARİTAYA EKLE (Map Update) - Eşleşmeyen ölçümler için
        for (int idx : new_measurement_indices) {
            map_objects_.push_back(incoming_measurements[idx]);
        }

        RCLCPP_INFO(this->get_logger(), "Harita Guncellendi. Toplam Obje Sayisi: %zu", map_objects_.size());
    }

private:
    // Düğümün (Node) kendi içinde tuttuğu HAFIZASI: Küresel Haritamız
    std::vector<TrackedObject> map_objects_;
};

} // namespace shared_world_model

#endif // SHARED_WORLD_MODEL__WORLD_MODEL_NODE_HPP_
