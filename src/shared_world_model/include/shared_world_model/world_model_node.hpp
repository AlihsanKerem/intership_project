#ifndef SHARED_WORLD_MODEL__WORLD_MODEL_NODE_HPP_
#define SHARED_WORLD_MODEL__WORLD_MODEL_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include "visualization_msgs/msg/marker_array.hpp"

// Senin yazdığın matematiksel algoritmalar
#include "shared_world_model/types.hpp"
#include "shared_world_model/data_association.hpp"
#include "shared_world_model/covariance_intersection.hpp"
#include "interfaces/msg/tracked_object_array.hpp"


namespace shared_world_model
{

class WorldModelNode : public rclcpp::Node
{
public:
    WorldModelNode() : Node("shared_world_model_node")
    {
        obj_sub_ = this->create_subscription<interfaces::msg::TrackedObjectArray>(
            "/detected_obstacles",
            10,
            std::bind(&WorldModelNode::obstacle_callback, this, std::placeholders::_1)
        );
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/world_map_markers", 10);

        RCLCPP_INFO(this->get_logger(), "Shared World Model Node Baslatildi!");
    }

    // Sensörden her yeni veri geldiğinde bu fonksiyon tetiklenecek
    void processMeasurements(const std::vector<TrackedObject>& incoming_measurements)
    {
        // 1. VERİ İLİŞKİLENDİRME (Data Association)
        // matched_pairs -> Bize hangi harita objesinin hangi sensör objesiyle eşleştiğini verir.
        std::vector<std::pair<int, int>> matched_pairs; 
        std::vector<int> new_measurement_indices; 
        
        DataAssociation::associate(map_objects_, incoming_measurements, 5.0, matched_pairs, new_measurement_indices);

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
        publishMarkers();
    }

    void publishMarkers()
    {
        visualization_msgs::msg::MarkerArray marker_array;

        for (size_t i = 0; i < map_objects_.size(); i++) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "odom";
            marker.header.stamp = this->now();
            marker.ns = "tracked_objects";
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.action = visualization_msgs::msg::Marker::ADD;

            // Objenin konumu
            marker.pose.position.x = map_objects_[i].state[0];
            marker.pose.position.y = map_objects_[i].state[1];
            marker.pose.position.z = 0.5;

            // Objenin boyutu
            marker.scale.x = 0.5;
            marker.scale.y = 0.5;
            marker.scale.z = 1.0,

            // Objenin rengi
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;

            marker_array.markers.push_back(marker);
        }
        marker_pub_->publish(marker_array);
    }

    void obstacle_callback(const
    interfaces::msg::TrackedObjectArray::SharedPtr msg)
    {
        std::vector<TrackedObject> incoming_measurements;

        for(const auto& obj_msg : msg->objects)
        {
            Eigen::Vector2d state(obj_msg.state[0], obj_msg.state[1]);

            Eigen::Matrix2d covariance;
            covariance(0, 0) = obj_msg.covariance[0];
            covariance(0, 1) = obj_msg.covariance[1];
            covariance(1, 0) = obj_msg.covariance[2];
            covariance(1, 1) = obj_msg.covariance[3];

            TrackedObject obj(obj_msg.id, state, covariance);
            incoming_measurements.push_back(obj);


        }
        processMeasurements(incoming_measurements);
    }

private:
    // Düğümün (Node) kendi içinde tuttuğu HAFIZASI: Küresel Haritamız
    std::vector<TrackedObject> map_objects_;
    rclcpp::Subscription<interfaces::msg::TrackedObjectArray>::SharedPtr obj_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

} // namespace shared_world_model

#endif // SHARED_WORLD_MODEL__WORLD_MODEL_NODE_HPP_
