#include "marrtino_core/replanning/replanner.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <utility>

namespace marrtino::replanning{

    namespace {

        constexpr std::int8_t kOccupied = 100;

    } // namespace

    Replanner::Replanner(ReplannerParams params) : params_(params){}

    void Replanner::reset(){
        state_ = State::Following;
        last_trigger_ = Trigger::None;
        last_replan_time_ = -999.0;
        failure_count_ = 0;
        infeasible_counter_ = 0;
        cbf_engagement_time_ = 0.0;
        last_control_step_time_ = -1.0;
        deviation_since_ = -1.0;
        stall_window_start_ = -1.0;
        stall_x_ = 0.0;
        stall_y_ = 0.0;
        tracks_.clear();
        trajectory_ = nullptr;
        reference_index_ = 0;
        first_seen_.clear();
    }

    State Replanner::state() const{
        return state_;
    }

    Trigger Replanner::lastTrigger() const{
        return last_trigger_;
    }

    bool Replanner::isBlocked() const{
        return state_ == State::Blocked;
    }

    int Replanner::failureCount() const{
        return failure_count_;
    }

    double Replanner::cbfEngagementTime() const{
        return cbf_engagement_time_;
    }

    double Replanner::inflationMargin() const{
        return params_.obstacle_inflation_margin;
    }

    const ReplannerParams& Replanner::params() const{
        return params_;
    }

    void Replanner::setContext(const obstacles::Tracks& tracks,
                               const trajectory::Trajectory& trajectory,
                               std::size_t reference_index,
                               double now){

        tracks_ = tracks;
        trajectory_ = &trajectory;
        reference_index_ = reference_index;

        if(!std::isfinite(now)){
            return;
        }

        std::unordered_map<int, double> seen;
        seen.reserve(tracks_.size());
        for(const obstacles::Track& track : tracks_){
            auto it = first_seen_.find(track.id);
            seen[track.id] = it != first_seen_.end() ? it->second : now;
        }
        first_seen_ = std::move(seen);
    }

    bool Replanner::isPersistent(const obstacles::Track& track, double now) const{
        auto it = first_seen_.find(track.id);
        if(it == first_seen_.end()){
            return false;
        }
        return now - it->second >= params_.persistent_obstacle_age;
    }

    bool Replanner::isReferenceBlocked(double now) const{
        if(trajectory_ == nullptr || tracks_.empty()){
            return false;
        }

        const trajectory::Trajectory& table = *trajectory_;
        if(table.size() < 2){
            return false;
        }

        double arc = 0.0;
        for(std::size_t i = std::max<std::size_t>(reference_index_, 1); i < table.size(); ++i){
            const double dx = table[i].x - table[i - 1].x;
            const double dy = table[i].y - table[i - 1].y;
            arc += std::sqrt(dx * dx + dy * dy);
            if(arc > params_.look_ahead_distance){
                break;
            }

            for(const obstacles::Track& track : tracks_){
                if(!isPersistent(track, now)){
                    continue;
                }
                const double ex = table[i].x - track.cx;
                const double ey = table[i].y - track.cy;
                const double block_radius = track.r + params_.obstacle_inflation_margin;
                if(ex * ex + ey * ey < block_radius * block_radius){
                    return true;
                }
            }
        }
        return false;
    }

    void Replanner::notifyControlStep(bool cbf_feasible, double cbf_deviation, double dynamic_margin, double now){
        infeasible_counter_ = cbf_feasible ? 0 : infeasible_counter_ + 1;

        const bool deviation_engaged = std::isfinite(cbf_deviation) && cbf_deviation > params_.engagement_deviation_threshold;
        const bool margin_engaged = std::isfinite(dynamic_margin) && dynamic_margin < params_.engagement_margin_threshold;
        const bool engaged_by_obstacle = deviation_engaged && margin_engaged;

        if(!std::isfinite(now)){
            return;
        }

        const double dt = last_control_step_time_ < 0.0 ? 0.0 : std::max(now - last_control_step_time_, 0.0);
        last_control_step_time_ = now;

        const double accumulated = cbf_engagement_time_ + (engaged_by_obstacle ? dt : -dt);
        const double ceiling = 2.0 * params_.min_cbf_engagement + 1.0;
        cbf_engagement_time_ = std::isfinite(accumulated) ? std::clamp(accumulated, 0.0, ceiling) : 0.0;
    }

    void Replanner::notifyProgress(const trajectory::Pose2D& pose, double distance_to_plan, double now){
        if(!std::isfinite(now) || !std::isfinite(pose.x) || !std::isfinite(pose.y)){
            return;
        }

        if(stall_window_start_ < 0.0){
            stall_window_start_ = now;
            stall_x_ = pose.x;
            stall_y_ = pose.y;
        }else{
            const double dx = pose.x - stall_x_;
            const double dy = pose.y - stall_y_;
            if(std::sqrt(dx * dx + dy * dy) > params_.stall_min_progress){
                stall_window_start_ = now;
                stall_x_ = pose.x;
                stall_y_ = pose.y;
            }
        }

        if(!std::isfinite(distance_to_plan) || distance_to_plan <= params_.replan_max_deviation){
            deviation_since_ = -1.0;
        }else if(deviation_since_ < 0.0){
            deviation_since_ = now;
        }
    }

    Trigger Replanner::evaluate(double now) const{
        last_trigger_ = Trigger::None;

        if(!std::isfinite(now)){
            return Trigger::None;
        }

        double wait_time = std::max(params_.replan_cooldown, params_.min_replan_interval);
        if(failure_count_ > 0){
            wait_time = params_.replan_retry_period;
        }
        if(failure_count_ >= params_.max_replan_failures){
            wait_time = params_.replan_retry_period * 3.0;
        }
        if(now - last_replan_time_ < wait_time){
            return Trigger::None;
        }

        if(cbf_engagement_time_ >= params_.min_cbf_engagement && isReferenceBlocked(now)){
            last_trigger_ = Trigger::PersistentObstacle;
        }else if(stall_window_start_ >= 0.0 && now - stall_window_start_ > params_.stall_window){
            last_trigger_ = Trigger::Stall;
        }else if(deviation_since_ >= 0.0 && now - deviation_since_ > params_.deviation_window){
            last_trigger_ = Trigger::Deviation;
        }else if(infeasible_counter_ >= params_.max_infeasible_steps){
            last_trigger_ = Trigger::Infeasible;
        }

        if(last_trigger_ != Trigger::None){
            state_ = State::Replanning;
        }

        return last_trigger_;
    }

    void Replanner::notifyReplanResult(bool success, double now){
        last_replan_time_ = now;
        failure_count_ = success ? 0 : failure_count_ + 1;
        state_ = (!success && failure_count_ >= params_.max_replan_failures) ? State::Blocked : State::Following;
        infeasible_counter_ = 0;
        cbf_engagement_time_ = 0.0;
        deviation_since_ = -1.0;
        stall_window_start_ = -1.0;
    }

    io::OccupancyGrid Replanner::inflate(const io::OccupancyGrid& pristine,
                                         const obstacles::Tracks& tracks,
                                         double margin){

        io::OccupancyGrid inflated = pristine;

        const grid::GridGeometry& geometry = inflated.geometry();
        const double resolution = geometry.resolution();
        if(resolution <= 0.0){
            return inflated;
        }

        for(const obstacles::Track& track : tracks){
            // Statica e senza `now`: la persistenza qui e' quella del tracker (`confirmed`).
            if(!track.confirmed){
                continue;
            }
            if(!std::isfinite(track.cx) || !std::isfinite(track.cy) || !std::isfinite(track.r)){
                continue;
            }

            const double radius = track.r + margin;
            if(!std::isfinite(radius) || radius <= 0.0){
                continue;
            }

            const int cell_radius = static_cast<int>(std::ceil(radius / resolution));
            const grid::Cell center = geometry.worldToCell(grid::Point2{track.cx, track.cy});

            for(int dy = -cell_radius; dy <= cell_radius; ++dy){
                for(int dx = -cell_radius; dx <= cell_radius; ++dx){
                    if(dx * dx + dy * dy > cell_radius * cell_radius){
                        continue;
                    }
                    grid::Cell c{center.x + dx, center.y + dy};
                    if(!geometry.contains(c)){
                        continue;
                    }
                    inflated[geometry.cellToIndex(c)] = kOccupied;
                }
            }
        }

        return inflated;
    }

} // namespace marrtino::replanning
