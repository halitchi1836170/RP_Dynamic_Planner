#pragma once

#include <vector>

#include "marrtino_core/grid/grid_geometry.hpp"

namespace marrtino::trajectory {

    struct Pose2D{
        double x{};
        double y{};
        double theta{};
    };

    struct TrajectorySample{
        double t{};
        double x{};
        double y{};
        double theta{};
        double v{};
        double omega{};
        double curvature{};
    };

    using Path = std::vector<grid::Point2>;
    using Trajectory = std::vector<TrajectorySample>;

    struct Command{
        double v{};
        double omega{};
    };

} // namespace marrtino::trajectory
