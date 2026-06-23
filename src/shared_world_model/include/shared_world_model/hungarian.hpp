#ifndef SHARED_WORLD_MODEL__HUNGARIAN_HPP_
#define SHARED_WORLD_MODEL__HUNGARIAN_HPP_

#include <vector>
#include <limits>
#include <algorithm>
#include "shared_world_model/types.hpp"

namespace shared_world_model
{

class HungarianAlgorithm
{
    
public:
        // Bu metot, maliyet matrisini alır ve en uygun eşleşmeleri Assignment listesine yazar.
    double Solve(std::vector<std::vector<double>>& DistMatrix, std::vector<int>& Assignment)
    {
        int nRows = DistMatrix.size();
        int nCols = DistMatrix[0].size();
        
        // Eğer satır ve sütun sayıları farklıysa, matrisi kare yapmak için "Dummy" (Sanal) değerler ekler.
        int n = std::max(nRows, nCols);
        std::vector<std::vector<double>> cost(n, std::vector<double>(n, 0.0));
        
        for (int i = 0; i < nRows; i++) {
            for (int j = 0; j < nCols; j++) {
                cost[i][j] = DistMatrix[i][j];
            }
        }

        std::vector<double> u(n, 0.0), v(n, 0.0);
        std::vector<int> p(n, 0), way(n, 0);

        for (int i = 1; i < n; i++) {
            p[0] = i;
            int j0 = 0;
            std::vector<double> minv(n, std::numeric_limits<double>::max());
            std::vector<bool> used(n, false);
            do {
                used[j0] = true;
                int i0 = p[j0], j1 = 0;
                double delta = std::numeric_limits<double>::max();
                for (int j = 1; j < n; j++) {
                    if (!used[j]) {
                        double cur = cost[i0][j] - u[i0] - v[j];
                        if (cur < minv[j]) {
                            minv[j] = cur;
                            way[j] = j0;
                        }
                        if (minv[j] < delta) {
                            delta = minv[j];
                            j1 = j;
                        }
                    }
                }
                for (int j = 0; j < n; j++) {
                    if (used[j]) {
                        u[p[j]] += delta;
                        v[j] -= delta;
                    } else {
                        minv[j] -= delta;
                    }
                }
                j0 = j1;
            } while (p[j0] != 0);
            do {
                int j1 = way[j0];
                p[j0] = p[j1];
                j0 = j1;
            } while (j0 != 0);
        }

        Assignment.assign(nRows, -1);
        double total_cost = 0;
        for (int j = 1; j < n; j++) {
            int i = p[j];
            if (i < nRows && j < nCols) {
                Assignment[i] = j;
                total_cost += cost[i][j];
            }
        }
        return total_cost; // Toplam maliyeti geri döner
    }
};

}

#endif // SHARED_WORLD_MODEL__HUNGARIAN_HPP_