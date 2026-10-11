#include "marrtino_core/control/trajectory_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>

namespace marrtino::control{

    namespace {

        constexpr double kFallbackPeriod = 0.05;

        const trajectory::TrajectorySample& restSample(){
            static const trajectory::TrajectorySample sample{};
            return sample;
        }

        double wrapToPi(double angle){
            return std::atan2(std::sin(angle), std::cos(angle));
        }

        double sinc(double x){
            return std::abs(x) < 1e-4 ? 1.0 : std::sin(x) / x;
        }

        double sanitize(double value){
            return std::isfinite(value) ? value : 0.0;
        }

        double saturate(double value, double limit){
            double v = sanitize(value);
            double l = std::abs(limit);
            return std::clamp(v, -l, l);
        }

    } // namespace

    TrajectoryTracker::TrajectoryTracker(TrackerParams params) : params_(params){}

    void TrajectoryTracker::setTrajectory(trajectory::Trajectory trajectory){
        trajectory_ = std::move(trajectory);
        index_ = 0;
        reference_time_ = 0.0;
        finished_ = false;
        active_ = !trajectory_.empty();
        last_v_ = 0.0;
        last_omega_ = 0.0;
        last_nominal_ = trajectory::Command{};
    }

    void TrajectoryTracker::reset(){
        trajectory_.clear();
        index_ = 0;
        reference_time_ = 0.0;
        active_ = false;
        finished_ = false;
        last_v_ = 0.0;
        last_omega_ = 0.0;
    }

    TrajectoryTracker::TrackingTerms TrajectoryTracker::computeTerms(const trajectory::Pose2D& pose, const trajectory::TrajectorySample& ref) const{
        double ex = ref.x - pose.x;
        double ey = ref.y - pose.y;
        TrackingTerms terms;
        terms.e1 = std::cos(pose.theta) * ex + std::sin(pose.theta) * ey;
        terms.e2 = -std::sin(pose.theta) * ex + std::cos(pose.theta) * ey;
        terms.e3 = wrapToPi(ref.theta - pose.theta);
        terms.k2 = params_.b;
        terms.v_des = ref.v;
        terms.omega_des = ref.omega;
        return terms;
    }

    trajectory::Command TrajectoryTracker::nominalControl(const trajectory::Pose2D& pose, const trajectory::TrajectorySample& ref) const{
        const TrackingTerms t = computeTerms(pose, ref);
        double e1 = t.e1;
        double e3 = t.e3;
        double e2 = t.e2;
        (void)e2;

        double v_des = t.v_des;
        double omega_des = t.omega_des;

        double a = std::sqrt(params_.b * v_des * v_des + omega_des * omega_des);
        double k1 = 2.0 * params_.zeta * a;
        double k2 = params_.b;
        double k3 = 2.0 * params_.zeta * a;

        double u1 = -k1 * e1;
        double u2 = -k2 * v_des * sinc(e3) * e2 - k3 * e3;

        double v = v_des * std::cos(e3) - u1;
        double omega = omega_des - u2;

        return trajectory::Command{saturate(v, params_.v_max), saturate(omega, params_.omega_max)};
    }

    trajectory::Command TrajectoryTracker::applyRateLimit(trajectory::Command command, double dt){
        command.v = sanitize(command.v);
        command.omega = sanitize(command.omega);

        double step = dt > 1e-4 ? dt : kFallbackPeriod;
        double maxDeltaV = params_.max_linear_accel * step;
        double maxDeltaW = params_.max_angular_accel * step;

        last_v_ += std::clamp(command.v - last_v_, -maxDeltaV, maxDeltaV);
        last_omega_ += std::clamp(command.omega - last_omega_, -maxDeltaW, maxDeltaW);

        last_v_ = sanitize(last_v_);
        last_omega_ = sanitize(last_omega_);
        return trajectory::Command{last_v_, last_omega_};
    }

    double TrajectoryTracker::referenceLag(const trajectory::Pose2D& pose) const{
        const trajectory::TrajectorySample& ref = trajectory_[index_];
        double ex = ref.x - pose.x;
        double ey = ref.y - pose.y;
        return std::sqrt(ex * ex + ey * ey);
    }

    void TrajectoryTracker::advanceReference(double dt){
        reference_time_ += dt;

        while(index_ + 1 < trajectory_.size() && trajectory_[index_ + 1].t <= reference_time_){
            ++index_;
        }

        if(index_ + 1 == trajectory_.size() && reference_time_ > trajectory_[index_].t){
            index_ = trajectory_.size();
        }
    }

    trajectory::Command TrajectoryTracker::update(const trajectory::Pose2D& pose, double dt){
        if(!active_ || trajectory_.empty()){
            return trajectory::Command{};
        }

        if(index_ >= trajectory_.size()){
            finished_ = true;
            active_ = false;
            last_v_ = 0.0;
            last_omega_ = 0.0;
            return trajectory::Command{};
        }

        const trajectory::TrajectorySample& goal = trajectory_.back();
        double gx = goal.x - pose.x;
        double gy = goal.y - pose.y;
        if(std::sqrt(gx * gx + gy * gy) < params_.goal_tolerance){
            index_ = trajectory_.size();
            finished_ = true;
            active_ = false;
            last_v_ = 0.0;
            last_omega_ = 0.0;
            return trajectory::Command{};
        }

        double step = std::clamp(sanitize(dt), 0.0, 0.5);

        terms_ = computeTerms(pose, trajectory_[index_]);

        trajectory::Command nominal = nominalControl(pose, trajectory_[index_]);
        last_nominal_ = nominal;
        if(filter_){
            nominal = filter_(nominal, pose, terms_);
        }

        trajectory::Command command = applyRateLimit(nominal, step);
        if(referenceLag(pose) < params_.reference_max_lag){
            advanceReference(step);
        }
        return command;
    }

    bool TrajectoryTracker::isActive() const{
        return active_;
    }

    bool TrajectoryTracker::isFinished() const{
        return finished_;
    }

    const TrajectoryTracker::TrackingTerms& TrajectoryTracker::terms() const{
        return terms_;
    }

    const trajectory::Command& TrajectoryTracker::lastNominal() const{
        return last_nominal_;
    }

    std::size_t TrajectoryTracker::referenceIndex() const{
        return index_;
    }

    void TrajectoryTracker::setSafetyFilter(SafetyFilter filter){
        filter_ = std::move(filter);
    }

    const trajectory::TrajectorySample& TrajectoryTracker::reference() const{
        if(trajectory_.empty()){
            return restSample();
        }
        return trajectory_[std::min(index_, trajectory_.size() - 1)];
    }

} // namespace marrtino::control
