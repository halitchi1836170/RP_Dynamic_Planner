#pragma once

#include <array>

namespace marrtino::grid {

    struct Direction{
        int dx;
        int dy;
        float cost;
    };

    inline constexpr float kDiagonalCost = 1.41421356f;

    inline constexpr std::array<Direction, 8> kDirections{{
        {+1,  0, 1.0f},
        {-1,  0, 1.0f},
        { 0, +1, 1.0f},
        { 0, -1, 1.0f},
        {+1, +1, kDiagonalCost},
        {-1, +1, kDiagonalCost},
        {+1, -1, kDiagonalCost},
        {-1, -1, kDiagonalCost},
    }};

} // namespace marrtino::grid
