#ifndef SHARED_WORLD_MODEL__TYPES_HPP_
#define SHARED_WORLD_MODEL__TYPES_HPP_

#include <Eigen/Dense>

namespace shared_world_model
{

// 2 Boyutlu bir dünyada objeleri takip ettiğimizi varsayalım (X ve Y düzleminde).
// Bu yüzden Durum Vektörümüz (State) 2x1 boyutunda bir matris (vektör) olacak.
// Hata matrisimiz (Covariance) ise X ve Y'nin kendi içindeki ve birbirleriyle olan hatalarını (korelasyon) tuttuğu için 2x2 boyutunda matris olacak.
using Vector2d = Eigen::Vector2d;
using Matrix2d = Eigen::Matrix2d;

// Haritadaki ve sensördeki her bir objeyi temsil eden yapımız
struct TrackedObject
{
    int id;               // Objeyi haritada benzersiz kılan kimlik numarası (Sensörden geliyorsa -1 olabilir)
    Vector2d state;       // Objenin pozisyonu: [X, Y]^T
    Matrix2d covariance;  // Objenin pozisyonundaki belirsizlik (hata) matrisi

    // Yapıcı fonksiyon (Constructor)
    TrackedObject(int obj_id, const Vector2d& obj_state, const Matrix2d& obj_cov)
    : id(obj_id), state(obj_state), covariance(obj_cov)
    {
    }
};

} // namespace shared_world_model

#endif // SHARED_WORLD_MODEL__TYPES_HPP_
