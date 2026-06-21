#ifndef SHARED_WORLD_MODEL__COVARIANCE_INTERSECTION_HPP_
#define SHARED_WORLD_MODEL__COVARIANCE_INTERSECTION_HPP_

#include "shared_world_model/types.hpp"

namespace shared_world_model
{

class CovarianceIntersection
{
public:
    // İki objeyi birleştirip güncellenmiş (yeni) bir obje döndürür
    static TrackedObject fuseCI(
        const TrackedObject& obj_A, 
        const TrackedObject& obj_B, 
        double omega = 0.5) // Omega şimdilik sabit 0.5
    {

        Matrix2d P_A_inv = obj_A.covariance.inverse();
        Matrix2d P_B_inv = obj_B.covariance.inverse();

        Matrix2d P_new = ( (omega * P_A_inv + (1.0 - omega) * P_B_inv).inverse() );


        Vector2d x_new = P_new * (omega * P_A_inv * obj_A.state + (1.0 - omega) * P_B_inv * obj_B.state);
        

        // Yeni objeyi oluşturup geri döndürüyoruz (ID'sini obj_A'dan koruyoruz)
        return TrackedObject(obj_A.id, x_new, P_new);
    }
};

} // namespace shared_world_model

#endif // SHARED_WORLD_MODEL__COVARIANCE_INTERSECTION_HPP_
