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
    double Solve(const std::vector<std::vector<double>>& DistMatrix, std::vector<int>& Assignment)
    {
        int nRows = DistMatrix.size();
        if (nRows == 0) return 0.0;
        int nCols = DistMatrix[0].size();
        if (nCols == 0) return 0.0;
        
        int n = std::max(nRows, nCols);
        
        // 1-indeksli çalışması için dizileri n+1 boyutunda başlatıyoruz
        std::vector<std::vector<double>> cost(n + 1, std::vector<double>(n + 1, 0.0));
        for (int i = 0; i < nRows; i++) {
            for (int j = 0; j < nCols; j++) {
                cost[i + 1][j + 1] = DistMatrix[i][j];
            }
        }

        std::vector<double> u(n + 1, 0.0), v(n + 1, 0.0);
        std::vector<int> p(n + 1, 0), way(n + 1, 0);

        for (int i = 1; i <= n; i++) {
            p[0] = i;
            int j0 = 0;
            std::vector<double> minv(n + 1, std::numeric_limits<double>::max());
            std::vector<bool> used(n + 1, false);
            
            do {
                used[j0] = true;
                int i0 = p[j0], j1 = 0;
                double delta = std::numeric_limits<double>::max();
                for (int j = 1; j <= n; j++) {
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
                for (int j = 0; j <= n; j++) {
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
        
        for (int j = 1; j <= n; j++) {
            int i = p[j];
            // 1-indeksinden geri 0-indeksine (orijinal haline) dönüştürüyoruz
            if (i >= 1 && i <= nRows && j >= 1 && j <= nCols) {
                Assignment[i - 1] = j - 1;
                total_cost += cost[i][j];
            }
        }
        
        return total_cost;
    }
};

} // namespace shared_world_model

#endif // SHARED_WORLD_MODEL__HUNGARIAN_HPP_
