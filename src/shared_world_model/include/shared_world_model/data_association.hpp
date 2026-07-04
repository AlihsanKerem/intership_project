#ifndef SHARED_WORLD_MODEL__DATA_ASSOCIATION_HPP_
#define SHARED_WORLD_MODEL__DATA_ASSOCIATION_HPP_

#include "shared_world_model/types.hpp"
#include "shared_world_model/hungarian.hpp"
#include <cmath> // sqrt işlemi için

namespace shared_world_model
{

class DataAssociation
{
public:
    // İki obje arasındaki Mahalanobis mesafesini hesaplar
    static double computeMahalanobisDistance(
        const TrackedObject& map_obj, 
        const TrackedObject& measurement)
    {
        // Pozisyon farkı vektörü (z - x)
        Vector2d diff = measurement.state - map_obj.state;

        // İnovasyon Kovaryansı: S = P + R
        // map_obj.covariance: Haritadaki objenin belirsizliği (P)
        // measurement.covariance: Sensör ölçümünün belirsizliği (R)
        Matrix2d S = map_obj.covariance + measurement.covariance;

        S(0, 0) += 1e-4;
        S(1, 1) += 1e-4;
        
        // Hesaplama
        double mahalanobis_distance = std::sqrt( (diff.transpose() * S.inverse() * diff).value() ); 

        return mahalanobis_distance;
    }

    // Elimizdeki tüm ölçümler ile haritadaki tüm objeler arasında eşleştirme yapar
    static void associate(
        const std::vector<TrackedObject>& map_objects,
        const std::vector<TrackedObject>& measurements,
        double max_allowed_distance, // Eşik değeri: Bu mesafeden uzaksa kesinlikle aynı obje değildir.
        std::vector<std::pair<int, int>>& matched_indices,
        std::vector<int>& new_measurement_indices)  
    {
        int num_map_objs = map_objects.size();
        int num_measurements = measurements.size();
        
        if(num_map_objs == 0 || num_measurements == 0) {
             // Harita boşsa gelen tüm veriler doğrudan yeni objedir eşleştirme yapmaya gerek yok
             for (int j = 0; j < num_measurements; j++) {
                new_measurement_indices.push_back(j);
             }
             return;
        } 
        
        Eigen::MatrixXd cost_matrix = Eigen::MatrixXd::Constant(num_map_objs, num_measurements, 1e9); 
        
        for (int i = 0; i < num_map_objs; i++) {
            for (int j = 0; j < num_measurements; j++) {
                double dist = computeMahalanobisDistance(map_objects[i], measurements[j]);
                // NaN değilse ve eşik değerinden küçükse maliyete yaz
                if (!std::isnan(dist) && dist < max_allowed_distance) {
                    cost_matrix(i, j) = dist;
                }
            }
        }
        std::vector<int> unmatched_map_objs;
        std::vector<int> assignment;
        HungarianAlgorithm hungarian;
        
        std::vector<std::vector<double>> std_cost_matrix(num_map_objs, std::vector<double>(num_measurements, 0.0));
        for (int i = 0; i < num_map_objs; i++) {
            for (int j = 0; j < num_measurements; j++) {
                std_cost_matrix[i][j] = cost_matrix(i, j);
            }
        }
        
        // Artık çevirdiğimiz yeni matrisi (std_cost_matrix) gönderiyoruz!
        hungarian.Solve(std_cost_matrix, assignment); 

        // Hangi ölçümlerin eşleştiğini takip edelim (Yeni objeleri bulabilmek için)
        std::vector<bool> measurement_assigned(num_measurements, false);


        for (int i = 0; i < num_map_objs; i++) {
            int j = assignment[i];
            if (j >= 0 && cost_matrix(i, j) < 1e9) {
                matched_indices.push_back(std::make_pair(i, j));
                measurement_assigned[j] = true;
            }
            else {
                unmatched_map_objs.push_back(i);
            }
        }

        // Hiçbir harita objesiyle eşleşmeyen ölçümleri Yeni Objeler sepetine atıyoruz
        for (int j = 0; j < num_measurements; j++) {
            if (!measurement_assigned[j]) {
                new_measurement_indices.push_back(j);
            }
        }



    }
};

} // namespace shared_world_model

#endif // SHARED_WORLD_MODEL__DATA_ASSOCIATION_HPP_
