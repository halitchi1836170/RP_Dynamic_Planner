#pragma once

#include <vector>

#include "marrtino_core/grid/grid.hpp"
#include "marrtino_core/grid/grid_geometry.hpp"
#include "marrtino_core/trajectory/types.hpp"

namespace marrtino::planning {

    struct SmootherParams{
        float tunnel_half_width = 0.25f;        // [m]
        float collinear_tolerance_deg = 10.0f;
        float min_vertex_spacing = 0.12f;       // [m]
    };

    // Entra un percorso in CELLE (uscita di A*), esce una spezzata in coordinate MONDO.
    trajectory::Path smoothPath(const grid::Grid<float>& edt,
                                const std::vector<grid::Cell>& path,
                                const SmootherParams& params);

} // namespace marrtino::planning
