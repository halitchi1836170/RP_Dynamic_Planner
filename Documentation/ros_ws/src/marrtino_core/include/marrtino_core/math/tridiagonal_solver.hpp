#pragma once

#include <stdexcept>
#include <vector>

namespace marrtino::math {

    class SingularSystemError : public std::runtime_error{
     public:
        using std::runtime_error::runtime_error;
    };

    std::vector<double> SolveTridiagonal(const std::vector<double>& a,
                                         const std::vector<double>& b,
                                         const std::vector<double>& c,
                                         const std::vector<double>& d);

} // namespace marrtino::math
