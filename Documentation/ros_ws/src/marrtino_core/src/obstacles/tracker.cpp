#include "marrtino_core/obstacles/tracker.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace marrtino::obstacles{

    namespace {

        constexpr double kGateSlack = 0.15;
        constexpr double kMinDt = 1e-3;
        constexpr std::size_t kNoIndex = static_cast<std::size_t>(-1);

    } // namespace

    ObstacleTracker::ObstacleTracker(TrackerParams params) : params_(params){}

    bool ObstacleTracker::isInBlindZone(const State& state, const trajectory::Pose2D& robot_pose) const{
        double dx = state.track.cx - robot_pose.x;
        double dy = state.track.cy - robot_pose.y;
        return std::sqrt(dx*dx + dy*dy) < state.track.r + params_.blind_zone_radius;
    }

    void ObstacleTracker::updateState(State& state, const Detection& detection, double now) const{
        Track& t = state.track;
        double dt = now - state.last_seen;
        bool poorObservation = detection.points < params_.min_points_for_update;

        if(dt > kMinDt){
            double vxRaw = (detection.cx - t.cx) / dt;
            double vyRaw = (detection.cy - t.cy) / dt;
            t.vx = (1.0 - params_.alpha_lowpass) * t.vx + params_.alpha_lowpass * vxRaw;
            t.vy = (1.0 - params_.alpha_lowpass) * t.vy + params_.alpha_lowpass * vyRaw;
            if(t.vx*t.vx + t.vy*t.vy < params_.v_deadzone * params_.v_deadzone){
                t.vx = 0.0;
                t.vy = 0.0;
            }
        }

        if(!poorObservation){
            t.cx = detection.cx;
            t.cy = detection.cy;
            state.r_max = std::max(detection.r, state.r_max * params_.radius_decay_per_update);
            t.r = state.r_max;
        }else{
            t.r = std::max(t.r, state.r_max);
        }

        state.last_seen = now;
        state.last_coast = now;
        t.hits += 1;
        t.confirmed = t.hits >= params_.min_hits;
    }

    void ObstacleTracker::refreshView(){
        view_.clear();
        view_.reserve(states_.size());
        for(const State& s : states_){
            view_.push_back(s.track);
        }
    }

    void ObstacleTracker::update(const Detections& detections, double now, const trajectory::Pose2D& robot_pose){

        std::stable_sort(states_.begin(), states_.end(), [](const State& a, const State& b){
            return a.first_seen < b.first_seen;
        });

        std::vector<bool> assignedDetections(detections.size(), false);
        std::vector<bool> assignedTracks(states_.size(), false);

        for(std::size_t ti = 0; ti < states_.size(); ++ti){
            State& state = states_[ti];
            double effectiveGate = std::min(params_.gate, state.track.r + kGateSlack);
            double bestSqDist = effectiveGate * effectiveGate;
            std::size_t best = kNoIndex;

            for(std::size_t i = 0; i < detections.size(); ++i){
                if(assignedDetections[i]){
                    continue;
                }

                double dx = detections[i].cx - state.track.cx;
                double dy = detections[i].cy - state.track.cy;
                double sq = dx*dx + dy*dy;
                if(sq < bestSqDist){
                    bestSqDist = sq;
                    best = i;
                }
            }

            if(best == kNoIndex){
                continue;
            }

            updateState(state, detections[best], now);
            assignedDetections[best] = true;
            assignedTracks[ti] = true;
        }

        for(std::size_t ti = 0; ti < states_.size(); ++ti){
            if(assignedTracks[ti]){
                continue;
            }

            State& state = states_[ti];
            state.track.vx = 0.0;
            state.track.vy = 0.0;

            if(!isInBlindZone(state, robot_pose)){
                continue;
            }

            double growth = params_.blind_zone_growth_rate * std::max(now - state.last_coast, 0.0);
            state.track.r = std::min(state.track.r + growth, state.r_max + params_.max_blind_growth);
            state.last_coast = now;
        }

        for(std::size_t i = 0; i < detections.size(); ++i){
            if(assignedDetections[i]){
                continue;
            }

            State state{};
            state.track.id = next_id_++;
            state.track.cx = detections[i].cx;
            state.track.cy = detections[i].cy;
            state.track.r = detections[i].r;
            state.track.hits = 1;
            state.track.confirmed = 1 >= params_.min_hits;
            state.r_max = detections[i].r;
            state.first_seen = now;
            state.last_seen = now;
            state.last_coast = now;
            states_.push_back(state);
        }

        states_.erase(std::remove_if(states_.begin(), states_.end(), [&](const State& s){
            const double factor = isInBlindZone(s, robot_pose) ? params_.blind_zone_forget_factor : 1.0;
            return now - s.last_seen > params_.forget_time * factor;
        }), states_.end());

        refreshView();
    }

    Tracks ObstacleTracker::confirmed() const{
        Tracks result;
        for(const State& s : states_){
            if(s.track.hits >= params_.min_hits){
                result.push_back(s.track);
            }
        }
        return result;
    }

    const Tracks& ObstacleTracker::all() const{
        return view_;
    }

    void ObstacleTracker::clear(){
        states_.clear();
        view_.clear();
        next_id_ = 0;
    }

} // namespace marrtino::obstacles
