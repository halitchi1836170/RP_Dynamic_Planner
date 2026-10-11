#pragma once

#include <cstddef>
#include <unordered_map>

#include "marrtino_core/io/map_loader.hpp"
#include "marrtino_core/obstacles/types.hpp"
#include "marrtino_core/trajectory/types.hpp"

namespace marrtino::replanning {

    enum class State{ Following, Replanning, Blocked };
    enum class Trigger{ None, PersistentObstacle, Stall, Deviation, Infeasible };

    struct ReplannerParams{
        double persistent_obstacle_age = 1.0;
        double look_ahead_distance = 2.0;
        double obstacle_inflation_margin = 0.25;
        double reduced_inflation_factor = 0.5;
        double stall_window = 4.0;
        double stall_min_progress = 0.15;
        double replan_max_deviation = 0.6;
        double deviation_window = 1.5;
        int    max_infeasible_steps = 5;
        double min_cbf_engagement = 1.5;
        double engagement_margin_threshold = 0.4;
        double engagement_deviation_threshold = 0.04;
        double min_replan_interval = 10.0;
        double replan_cooldown = 4.0;
        double replan_retry_period = 5.0;
        int    max_replan_failures = 3;
    };

    class Replanner{
        public:

        explicit Replanner(ReplannerParams params = {});

        // I track vengono copiati; della traiettoria si tiene solo un puntatore: deve restare viva
        // (e non riallocata) fino alla setContext successiva, che e' il contratto del nodo che la possiede.
        void setContext(const obstacles::Tracks& tracks,
                        const trajectory::Trajectory& trajectory,
                        std::size_t reference_index,
                        double now);

        void notifyControlStep(bool cbf_feasible, double cbf_deviation, double dynamic_margin, double now);
        void notifyProgress(const trajectory::Pose2D& pose, double distance_to_plan, double now);
        [[nodiscard]] Trigger evaluate(double now) const;
        void notifyReplanResult(bool success, double now);
        [[nodiscard]] State state() const;
        void reset();

        [[nodiscard]] Trigger lastTrigger() const;
        [[nodiscard]] bool isBlocked() const;
        [[nodiscard]] int failureCount() const;
        [[nodiscard]] double cbfEngagementTime() const;
        [[nodiscard]] double inflationMargin() const;
        [[nodiscard]] const ReplannerParams& params() const;

        static io::OccupancyGrid inflate(const io::OccupancyGrid& pristine,
                                         const obstacles::Tracks& tracks,
                                         double margin);

        private:

        [[nodiscard]] bool isReferenceBlocked(double now) const;
        [[nodiscard]] bool isPersistent(const obstacles::Track& track, double now) const;

        ReplannerParams params_;

        obstacles::Tracks tracks_{};
        const trajectory::Trajectory* trajectory_ = nullptr;
        std::size_t reference_index_ = 0;
        std::unordered_map<int, double> first_seen_{};

        mutable State state_ = State::Following;
        mutable Trigger last_trigger_ = Trigger::None;

        double last_replan_time_ = -999.0;
        int failure_count_ = 0;
        int infeasible_counter_ = 0;
        double cbf_engagement_time_ = 0.0;
        double last_control_step_time_ = -1.0;
        double deviation_since_ = -1.0;
        double stall_window_start_ = -1.0;
        double stall_x_ = 0.0;
        double stall_y_ = 0.0;
    };

} // namespace marrtino::replanning
