#pragma once

#include <vector>

#include "marrtino_core/obstacles/types.hpp"
#include "marrtino_core/trajectory/types.hpp"

namespace marrtino::obstacles {

    struct TrackerParams{
        double gate = 0.5;                  // associazione detection-track [m]
        double alpha_lowpass = 0.3;
        double v_deadzone = 0.05;
        int    min_hits = 3;
        double forget_time = 1.0;           // [s]
        double blind_zone_radius = 0.7;
        // In zona cieca non vedere NON e' prova di assenza: il track sopravvive
        // molto piu' a lungo. Senza, sparisce proprio mentre il robot gli e' addosso.
        double blind_zone_forget_factor = 6.0;
        double blind_zone_growth_rate = 0.05;
        double max_blind_growth = 0.2;
        double radius_decay_per_update = 0.98;
        int    min_points_for_update = 4;
    };

    class ObstacleTracker{
        public:

        explicit ObstacleTracker(TrackerParams params = {});

        void update(const Detections& detections, double now, const trajectory::Pose2D& robot_pose);

        [[nodiscard]] Tracks confirmed() const;
        [[nodiscard]] const Tracks& all() const;

        void clear();

        [[nodiscard]] const TrackerParams& params() const { return params_; }

        private:
        struct State{
            Track track{};
            double r_max{};
            double first_seen{};
            double last_seen{};
            double last_coast{};
        };

        [[nodiscard]] bool isInBlindZone(const State& state, const trajectory::Pose2D& robot_pose) const;
        void updateState(State& state, const Detection& detection, double now) const;
        void refreshView();

        TrackerParams params_{};
        std::vector<State> states_{};
        Tracks view_{};
        int next_id_{};
    };

} // namespace marrtino::obstacles
