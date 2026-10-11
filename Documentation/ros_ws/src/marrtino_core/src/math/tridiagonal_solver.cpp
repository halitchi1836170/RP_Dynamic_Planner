#include "marrtino_core/math/tridiagonal_solver.hpp"

#include <cmath>
#include <cstddef>
#include <string>

namespace marrtino::math{

    namespace {
        constexpr double kPivotEpsilon = 1e-12;
    }

    std::vector<double> SolveTridiagonal(const std::vector<double>& a,
                                         const std::vector<double>& b,
                                         const std::vector<double>& c,
                                         const std::vector<double>& d){

        const std::size_t n = b.size();
        if(n == 0){
            throw std::invalid_argument("Sistema vuoto.");
        }
        if(a.size() != n || c.size() != n || d.size() != n){
            throw std::invalid_argument("Tutti i vettori devono avere la stessa lunghezza.");
        }

        std::vector<double> cp(n);
        std::vector<double> dp(n);
        std::vector<double> x(n);

        double pivot = b[0];
        if(std::abs(pivot) < kPivotEpsilon){
            throw SingularSystemError("Pivot nullo alla riga 0: matrice singolare o mal posta.");
        }

        cp[0] = c[0] / pivot;
        dp[0] = d[0] / pivot;

        for(std::size_t i = 1; i < n; ++i){
            pivot = b[i] - a[i] * cp[i - 1];
            if(std::abs(pivot) < kPivotEpsilon){
                throw SingularSystemError("Pivot nullo alla riga " + std::to_string(i) +
                                          ": perdita di dominanza diagonale?");
            }
            cp[i] = c[i] / pivot;
            dp[i] = (d[i] - a[i] * dp[i - 1]) / pivot;
        }

        x[n - 1] = dp[n - 1];
        for(std::size_t i = n - 1; i-- > 0;){
            x[i] = dp[i] - cp[i] * x[i + 1];
        }

        return x;
    }

} // namespace marrtino::math
