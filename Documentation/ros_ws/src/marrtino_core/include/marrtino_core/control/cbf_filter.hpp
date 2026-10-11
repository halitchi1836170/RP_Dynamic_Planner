#pragma once

#include "marrtino_core/grid/grid.hpp"
#include "marrtino_core/obstacles/types.hpp"
#include "marrtino_core/trajectory/types.hpp"

namespace marrtino::control {

    struct CBFParams{
        double b_look_ahead = 0.15;
        // La relazione di progetto vorrebbe r_safe_dynamic >= robot_body_radius + b_look_ahead = 0.38: il valore sotto non la rispetta, la riga sul corpo del robot la compensa.
        double r_safe_dynamic = 0.30;
        double r_safe_static = 0.18;
        double alpha_dynamic = 0.8;
        double alpha_static = 0.8;
        double gamma_clf = 0.5;
        double slack_penalty = 100.0;
        double cbf_slack_penalty = 10000.0;
        double command_smoothing_weight = 0.5;
        double v_deviation_weight = 1.0;
        double w_deviation_weight = 0.2;
        double static_activation_distance = 1.0;
        double obstacle_activation_range = 3.0;
        double gradient_step_cells = 1.0;
        double activation_tolerance = 1e-3;
        double w_max_cbf = 0.9;
        double v_max = 0.2;
        double robot_body_radius = 0.23;
    };

    struct CBFDiagnostics{
        bool   active{};
        bool   feasible{true};
        double deviation{};
        double min_barrier{};
        double min_margin{};
        double static_margin{};
        double slack{};
        int    constraints{};
    };

    class CBFFilter{
        public:

        explicit CBFFilter(CBFParams params = {});

        void setObstacles(obstacles::Tracks tracks);
        void setStaticMap(const grid::Grid<float>* edt);

        trajectory::Command filter(const trajectory::Command& u_nom,
                                   const trajectory::Pose2D& pose,
                                   double e1, double e2, double e3, double k2,
                                   double v_des, double omega_des);

        void reset();

        [[nodiscard]] const CBFDiagnostics& diagnostics() const;
        [[nodiscard]] const CBFParams& params() const;

        private:

        CBFParams params_;
        obstacles::Tracks tracks_{};
        const grid::Grid<float>* edt_ = nullptr;
        trajectory::Command last_solution_{};
        CBFDiagnostics diagnostics_{};
    };

} // namespace marrtino::control
