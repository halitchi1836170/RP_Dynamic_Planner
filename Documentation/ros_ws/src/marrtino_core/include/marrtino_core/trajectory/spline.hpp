#pragma once

#include "marrtino_core/trajectory/types.hpp"

namespace marrtino::trajectory {

    struct SplineParams{
        int samples_per_segment = 30;
    };

    // Interpola i vertici con una spline cubica C2 e restituisce un percorso
    // densamente campionato. Meno di 2 vertici -> restituisce l'ingresso invariato.
    Path buildSpline(const Path& vertices, const SplineParams& params);

} // namespace marrtino::trajectory
