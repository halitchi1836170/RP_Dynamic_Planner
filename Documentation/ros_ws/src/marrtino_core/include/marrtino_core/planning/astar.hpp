#pragma once

#include <vector>
#include "marrtino_core/grid/grid.hpp"
#include "marrtino_core/grid/grid_geometry.hpp"
#include <optional>

namespace marrtino::planning {

    struct AStarParams{
        float repulsion_gain = 50.0f;
        float epsilon = 0.01f;
        float start_escape_radius = 0.5f;
        float relax_clearance = 0.2f;
        float relax_radius = 1.2f;
    };

    struct PlanResult{
        std::vector<grid::Cell> path;
        grid::Cell reached_goal;
        bool goal_was_relaxed;
    };

    std::optional<PlanResult> findPath(const grid::Grid<float>& edt,
                                        grid::Cell start, grid::Cell goal,
                                        const AStarParams& params);

} // namespace marrtino::planning
