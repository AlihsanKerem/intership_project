#ifndef SHARED_WORLD_MODEL__COVARIANCE_INTERSECTION_HPP_
#define SHARED_WORLD_MODEL__COVARIANCE_INTERSECTION_HPP_

#include "shared_world_model/types.hpp"

namespace shared_world_model
{

/**
     * @class CovarianceIntersection
     * @brief İki farklı sensör ölçümünü (veya harita ile sensörü) bilinmeyen
     * korelasyonlar altında güvenli bir şekilde birleştiren (Sensor Fusion) sınıf.
     */
class CovarianceIntersection
{
public:
    /**
     * @brief İki nesnenin konum ve kovaryans matrislerini CI algoritması ile birleştirir.
     * 
     * @param obj_A Birinci nesne (Genellikle global haritadaki mevcut nesne)
     * @param obj_B İkinci nesne (Genellikle sensörden gelen yeni ölçüm)
     * @param omega Ağırlık parametresi (0.0 ile 1.0 arasında). Sabit 0.5 olarak ayarlandı.
     * @return TrackedObject Birleştirilmiş yeni konum ve kovaryansa sahip güncellenmiş nesne
     */
    static TrackedObject fuseCI(
        const TrackedObject& obj_A, 
        const TrackedObject& obj_B, 
        double omega = 0.5)
    {
        // Kovaryans Matrislerinin Tersini Alıyoruz ve bu Information Matrix oluyor
        // Belirsizlik ne kadar küçükse, tersi (bilgi) o kadar büyük olur.
        Matrix2d P_A_inv = obj_A.covariance.inverse();
        Matrix2d P_B_inv = obj_B.covariance.inverse();

        // Yeni Kovaryansın Hesaplanması
        Matrix2d P_new = ( (omega * P_A_inv + (1.0 - omega) * P_B_inv).inverse() );

        // Yeni Konumun Hesaplanması
        Vector2d x_new = P_new * (omega * P_A_inv * obj_A.state + (1.0 - omega) * P_B_inv * obj_B.state);
        

        // Yeni objeyi oluşturup geri döndürüyoruz (ID'sini obj_A'dan koruyoruz)
        return TrackedObject(obj_A.id, x_new, P_new);
    }
};

} // namespace shared_world_model

#endif // SHARED_WORLD_MODEL__COVARIANCE_INTERSECTION_HPP_
