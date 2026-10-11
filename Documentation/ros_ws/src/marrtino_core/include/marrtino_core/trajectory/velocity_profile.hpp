#pragma once

#include "marrtino_core/trajectory/types.hpp"

namespace marrtino::trajectory {

    struct ProfileParams{
        double cruise_speed = 0.08;
        double v_max = 0.2;
        double v_min = 0.05;
        double a_max = 0.08;
        double omega_max = 0.8;
        double control_period = 0.05;
        // La spline campiona in modo irregolare, con punti quasi coincidenti: su
        // quelli atan2 e la curvatura sono rumore. Si ricampiona a passo costante
        // PRIMA di calcolare la geometria.
        double geometry_step = 0.03;
    };

    Trajectory buildTrajectory(const Path& path, const ProfileParams& params);

} // namespace marrtino::trajectory
