#ifndef SHARED_WORLD_MODEL__DATA_ASSOCIATION_HPP_
#define SHARED_WORLD_MODEL__DATA_ASSOCIATION_HPP_

#include "shared_world_model/types.hpp"
#include "shared_world_model/hungarian.hpp"
#include <cmath> // sqrt işlemi için

namespace shared_world_model
{

/**
     * @class DataAssociation
     * @brief Haritadaki mevcut nesneler ile Lidar'dan yeni gelen nesne ölçümlerini 
     * birbirleriyle eşleştiren class.
     * 
     * Eşleştirme maliyeti olarak Mahalanobis Uzaklığı kullanılır. En uygun 
     * eşleşmeyi bulmak için ise Hungarian Algoritması çalıştırılır.
     */
class DataAssociation
{
public:
    /**
     * @brief İki nesne arasındaki Mahalanobis uzaklığını hesaplar.
     * 
     * Klasik Öklid uzaklığından farklı olarak bu metot, sensörlerin ve haritanın 
     * kovaryans durumunu da hesaba katar. Belirsizliğin yüksek olduğu 
     * durumlarda nesnelerin eşleşme toleransı artar.
     * 
     * Formül: D = sqrt( (z - x)^T * S^-1 * (z - x) )
     * (S = İnovasyon Kovaryansı = P + R)
     * 
     * @param map_obj Haritada var olan nesne
     * @param measurement Lidar'dan yeni gelen ölçüm
     * @return double Hesaplanmış uzaklık değeri (Maliyet)
     */
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

        // Sayısal stabilite için kovaryans matrisinin köşegenlerine küçük bir pay ekliyoruz
        // Bu işlem, belirsizliğin sıfır olup matrisin tersinin alınamaması (Singular Matrix) hatasını önlüyor
        S(0, 0) += 1.0;
        S(1, 1) += 1.0;
        
        // Hesaplama
        double mahalanobis_distance = std::sqrt( (diff.transpose() * S.inverse() * diff).value() ); 

        return mahalanobis_distance;
    }

    /**
     * @brief Tüm ölçümler ile haritadaki tüm objeler arasında optimum eşleştirmeyi yapar.
     * 
     * Adımlar:
     * 1. N x M boyutunda bir Cost Matrix oluşturulur.
     * 2. Matrisin her bir hücresine o iki nesne arasındaki Mahalanobis uzaklığı yazılır.
     *    Uzaklık eşik değerinden (max_allowed_distance) büyükse eşleşme imkansız sayılır.
     * 3. Hungarian algoritması çalıştırılarak toplam maliyeti en aza indirecek eşleşme bulunur.
     * 4. Eşleşmeyen (yeni) ölçümler tespit edilir.
     * 
     * @param map_objects Global haritadaki mevcut nesneler
     * @param measurements Sensörden (Lidar) gelen yeni ölçümler
     * @param max_allowed_distance Eşleşme için izin verilen maksimum Mahalanobis uzaklığı
     * @param matched_indices Eşleşen nesnelerin indeks çiftleri (Çıktı)
     * @param new_measurement_indices Haritadaki hiçbir nesneyle eşleşmeyen YENİ nesnelerin indeksleri (Çıktı)
     */
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
