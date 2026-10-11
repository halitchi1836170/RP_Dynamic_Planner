#pragma once

#include <cstddef>
#include <functional>

#include <cstddef>

#include "marrtino_core/trajectory/types.hpp"

namespace marrtino::control {

    struct TrackerParams{
        double b = 4.0;
        double zeta = 0.9;
        double v_max = 0.2;
        double omega_max = 1.2;
        double max_linear_accel = 0.3;
        double max_angular_accel = 1.5;
        double reference_max_lag = 0.20;
        double goal_tolerance = 0.10;
    };

    class TrajectoryTracker{
        // Termini dell'ultimo passo, necessari alla riga CLF del filtro CBF.
        struct TrackingTerms{
            double e1{};
            double e2{};
            double e3{};
            double k2{};
            double v_des{};
            double omega_des{};
        };

        // Il filtro di sicurezza si inserisce fra il controllore nominale e il
        // limitatore di slew, come in ControllerService.getControlInput.
        using SafetyFilter = std::function<trajectory::Command(const trajectory::Command&,
                                                               const trajectory::Pose2D&,
                                                               const TrackingTerms&)>;

        public:

        explicit TrajectoryTracker(TrackerParams params = {});

        void setTrajectory(trajectory::Trajectory trajectory);
        void reset();

        trajectory::Command update(const trajectory::Pose2D& pose, double dt);

        [[nodiscard]] bool isActive() const;
        [[nodiscard]] bool isFinished() const;
        [[nodiscard]] const trajectory::TrajectorySample& reference() const;
        [[nodiscard]] const TrackingTerms& terms() const;
        // Comando della legge di inseguimento PRIMA del filtro di sicurezza:
        // e' il "nominale" che la figura di analisi confronta con il comandato.
        [[nodiscard]] const trajectory::Command& lastNominal() const;
        [[nodiscard]] std::size_t referenceIndex() const;

        void setSafetyFilter(SafetyFilter filter);

        private:

        TrackingTerms computeTerms(const trajectory::Pose2D& pose, const trajectory::TrajectorySample& ref) const;
        trajectory::Command nominalControl(const trajectory::Pose2D& pose, const trajectory::TrajectorySample& ref) const;
        trajectory::Command applyRateLimit(trajectory::Command command, double dt);
        double referenceLag(const trajectory::Pose2D& pose) const;
        void advanceReference(double dt);

        TrackerParams params_;
        trajectory::Trajectory trajectory_;
        std::size_t index_ = 0;
        double reference_time_ = 0.0;
        bool active_ = false;
        bool finished_ = false;
        double last_v_ = 0.0;
        double last_omega_ = 0.0;
        TrackingTerms terms_{};
        trajectory::Command last_nominal_{};
        SafetyFilter filter_;
    };

} // namespace marrtino::control
