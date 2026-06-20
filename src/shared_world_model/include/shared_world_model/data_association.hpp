#ifndef SHARED_WORLD_MODEL__DATA_ASSOCIATION_HPP_
#define SHARED_WORLD_MODEL__DATA_ASSOCIATION_HPP_

#include "shared_world_model/types.hpp"
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
        // 1. Pozisyon farkı vektörü (z - x)
        Vector2d diff = measurement.state - map_obj.state;

        // 2. İnovasyon Kovaryansı: S = P + R
        // map_obj.covariance: Haritadaki objenin belirsizliği (P)
        // measurement.covariance: Sensör ölçümünün belirsizliği (R)
        Matrix2d S = map_obj.covariance + measurement.covariance;
        
        // Hesaplama
        double mahalanobis_distance = std::sqrt( (diff.transpose() * S.inverse() * diff).value() ); 

        return mahalanobis_distance;
    }

    // Elimizdeki tüm ölçümler ile haritadaki tüm objeler arasında eşleştirme yapar
    static void associate(
        const std::vector<TrackedObject>& map_objects,
        const std::vector<TrackedObject>& measurements,
        double max_allowed_distance) // Eşik değeri: Bu mesafeden uzaksa kesinlikle aynı obje değildir.
    {
        int num_map_objs = map_objects.size();
        int num_measurements = measurements.size();
        // Maliyet matrisini oluşturmak için Eigen'ın dinamik boyutlu matrisini kullanalım
        Eigen::MatrixXd cost_matrix = Eigen::MatrixXd::Constant(num_map_objs, num_measurements, 1e9); 
        // 1e9 (çok büyük bir sayı) ile doldurduk. Eğer uzaklık max_allowed_distance'tan büyükse eşleşmesin diye maliyet devasa kalacak.

        for (int i = 0; i < num_map_objs; i++) {
            for (int j = 0; j < num_measurements; j++) {
                double dist = computeMahalanobisDistance(map_objects[i], measurements[j]);
                if (dist < max_allowed_distance) {
                    cost_matrix(i, j) = dist;
                }
            }
        }

        // Döngü bitti ve cost_matrix hazır.

        // Eşleşmeleri tutacağımız listeler (Geri döndüreceğimiz sonuçlar)
        std::vector<std::pair<int, int>> matched_indices; // (harita_index, olcum_index)
        std::vector<int> unmatched_measurements; // Yeni objeler
        std::vector<int> unmatched_map_objs;     // Görülemeyen/Eski objeler

        // (Varsayımsal bir Macar algoritması çözücüsü)
        // assignment vektörü bize harita objelerinin hangi sensör ölçümleriyle eşleştiğini döner.
        // Örneğin assignment[0] = 2 demek -> 0. harita objesi, 2. ölçüm ile eşleşti demektir.
        // Eğer assignment[0] = -1 ise -> 0. harita objesi hiçbir ölçümle EŞLEŞEMEDİ demektir.
        std::vector<int> assignment;
        // Hungarian.Solve(cost_matrix, assignment); 

        // --- GÖREV BAŞLANGICI ---
        // Senden "matched_indices" ve "unmatched_map_objs" listelerini dolduran bir döngü yazmanı istiyorum.
        // assignment vektörünün boyutu num_map_objs kadardır.
        // i=0'dan başlayıp num_map_objs'a kadar dönen bir for döngüsü yaz.
        // Eğer assignment[i] değeri 0 veya daha büyükse (ve o hücredeki cost 1e9'dan küçükse) -> matched_indices'e ekle
        // Eğer assignment[i] değeri -1 ise -> unmatched_map_objs'e ekle
        // (std::vector'e eleman eklemek için -> liste_adi.push_back(eleman); kullanılır)

        // KODUNU BURAYA YAZ:

        for (int i = 0; i < num_map_objs; i++) {
            int j = assignment[i];
            if (j >= 0 && cost_matrix(i, j) < 1e9) {
                matched_indices.push_back(std::make_pair(i, j));
            }
            else {
                unmatched_map_objs.push_back(i);
            }
        }



        // --- GÖREV BİTİŞİ ---


    }
};

} // namespace shared_world_model

#endif // SHARED_WORLD_MODEL__DATA_ASSOCIATION_HPP_
