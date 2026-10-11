#include "marrtino_core/io/plot_recorder.hpp"

#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <locale>
#include <sstream>

namespace marrtino::io{

    namespace {

        // Il punto decimale deve essere un PUNTO qualunque sia la locale di sistema:
        // con quella italiana la virgola spezzerebbe il CSV. E' lo stesso motivo per
        // cui il C# usa CultureInfo.InvariantCulture ovunque.
        std::string num(double v){
            if(!std::isfinite(v)){
                return "nan";
            }
            std::ostringstream os;
            os.imbue(std::locale::classic());
            os << std::setprecision(9) << v;
            return os.str();
        }

        std::string timestamp(){
            const std::time_t now = std::time(nullptr);
            std::tm tm{};
#ifdef _WIN32
            localtime_s(&tm, &now);
#else
            localtime_r(&now, &tm);
#endif
            std::ostringstream os;
            os << std::put_time(&tm, "%Y%m%d_%H%M%S");
            return os.str();
        }

    } // namespace

    PlotRecorder::PlotRecorder(double sample_period) : sample_period_(sample_period){}

    bool PlotRecorder::isTimeToSample(double now) const{
        if(last_sample_ < 0.0 || now - last_sample_ >= sample_period_){
            last_sample_ = now;
            return true;
        }
        return false;
    }

    void PlotRecorder::recordControl(double t, const trajectory::Command& commanded,
                                     const trajectory::Command& nominal,
                                     bool cbf_active, bool cbf_feasible,
                                     double min_barrier, double min_margin, double static_margin,
                                     double e1, double e2, double e3, double lyapunov){
        control_.push_back(ControlRow{t, commanded.v, commanded.omega, nominal.v, nominal.omega,
                                      cbf_active ? 1 : 0, cbf_feasible ? 1 : 0,
                                      min_barrier, min_margin, static_margin, e1, e2, e3, lyapunov});
    }

    void PlotRecorder::recordPose(double t, const trajectory::Pose2D& ground_truth,
                                  const trajectory::Pose2D& localized, const trajectory::Pose2D& odometry){
        poses_.push_back(PoseRow{t, ground_truth.x, ground_truth.y, ground_truth.theta,
                                 localized.x, localized.y, localized.theta,
                                 odometry.x, odometry.y, odometry.theta});
    }

    void PlotRecorder::recordPlan(int plan_id, double t_arm, const trajectory::Path& path){
        for(const auto& p : path){
            plans_.push_back(PlanRow{plan_id, t_arm, p.x, p.y});
        }
    }

    void PlotRecorder::recordObstacles(double t, const obstacles::Tracks& tracks){
        for(const auto& tr : tracks){
            obstacles_.push_back(ObstacleRow{t, tr.id, tr.cx, tr.cy, tr.r});
        }
    }

    void PlotRecorder::recordReplan(double t, const trajectory::Pose2D& pose, const std::string& trigger,
                                    bool success, double margin, int tracks, int failures,
                                    const std::string& reason){
        replans_.push_back(ReplanRow{t, pose.x, pose.y, trigger, success ? 1 : 0,
                                     margin, tracks, failures, reason});
    }

    void PlotRecorder::setLimit(const std::string& name, double value){
        limits_.emplace_back(name, value);
    }

    void PlotRecorder::setDistanceMap(const grid::Grid<float>& edt){
        const auto& g = edt.geometry();
        dmap_w_ = g.width();
        dmap_h_ = g.height();
        dmap_ox_ = static_cast<float>(g.origin().x);
        dmap_oy_ = static_cast<float>(g.origin().y);
        dmap_res_ = static_cast<float>(g.resolution());

        // Il plotter si aspetta METRI; la nostra EDT e' in celle.
        dmap_.resize(edt.size());
        for(std::size_t i = 0; i < edt.size(); ++i){
            const float d = edt[i];
            dmap_[i] = std::isfinite(d) ? d * dmap_res_ : 0.0f;
        }
        has_dmap_ = true;
    }

    bool PlotRecorder::empty() const{
        return control_.empty() && poses_.empty() && plans_.empty();
    }

    std::string PlotRecorder::dump(const std::filesystem::path& base_dir){
        if(empty()){
            return {};
        }

        std::error_code ec;
        if(run_dir_.empty()){
            run_dir_ = (base_dir / ("run_cpp_" + timestamp())).string();   // prefisso distinto da quello del C#
        }
        const std::filesystem::path dir = run_dir_;
        std::filesystem::create_directories(dir, ec);
        if(ec){
            return {};
        }

        const auto open = [&dir](const char* name) {
            return std::ofstream(dir / name, std::ios::out | std::ios::trunc);
        };

        {
            auto f = open("control.csv");
            f << "t,v_cmd,w_cmd,v_nom,w_nom,cbf_active,cbf_feasible,min_barrier,min_margin,static_margin,e1,e2,e3,V\n";
            for(const auto& r : control_){
                f << num(r.t) << ',' << num(r.v_cmd) << ',' << num(r.w_cmd) << ','
                  << num(r.v_nom) << ',' << num(r.w_nom) << ',' << r.active << ',' << r.feasible << ','
                  << num(r.min_barrier) << ',' << num(r.min_margin) << ',' << num(r.static_margin) << ','
                  << num(r.e1) << ',' << num(r.e2) << ',' << num(r.e3) << ',' << num(r.v_lyap) << '\n';
            }
        }
        {
            auto f = open("poses.csv");
            f << "t,gt_x,gt_y,gt_theta,icp_x,icp_y,icp_theta,odo_x,odo_y,odo_theta\n";
            for(const auto& r : poses_){
                f << num(r.t) << ',' << num(r.gx) << ',' << num(r.gy) << ',' << num(r.gth) << ','
                  << num(r.lx) << ',' << num(r.ly) << ',' << num(r.lth) << ','
                  << num(r.ox) << ',' << num(r.oy) << ',' << num(r.oth) << '\n';
            }
        }
        {
            auto f = open("plans.csv");
            f << "plan_id,t_arm,x,y\n";
            for(const auto& r : plans_){
                f << r.plan_id << ',' << num(r.t_arm) << ',' << num(r.x) << ',' << num(r.y) << '\n';
            }
        }
        {
            auto f = open("obstacles.csv");
            f << "t,id,cx,cy,r\n";
            for(const auto& r : obstacles_){
                f << num(r.t) << ',' << r.id << ',' << num(r.cx) << ',' << num(r.cy) << ',' << num(r.r) << '\n';
            }
        }
        {
            auto f = open("replans.csv");
            f << "t,robot_x,robot_y,trigger,success,margin,tracks,failures,reason\n";
            for(const auto& r : replans_){
                f << num(r.t) << ',' << num(r.x) << ',' << num(r.y) << ',' << r.trigger << ','
                  << r.success << ',' << num(r.margin) << ',' << r.tracks << ',' << r.failures << ','
                  << r.reason << '\n';
            }
        }
        {
            auto f = open("limits.csv");
            f << "name,value\n";
            for(const auto& kv : limits_){
                f << kv.first << ',' << num(kv.second) << '\n';
            }
        }

        if(has_dmap_){
            std::ofstream f(dir / "distancemap.bin", std::ios::binary | std::ios::trunc);
            f.write(reinterpret_cast<const char*>(&dmap_w_), sizeof(dmap_w_));
            f.write(reinterpret_cast<const char*>(&dmap_h_), sizeof(dmap_h_));
            f.write(reinterpret_cast<const char*>(&dmap_ox_), sizeof(dmap_ox_));
            f.write(reinterpret_cast<const char*>(&dmap_oy_), sizeof(dmap_oy_));
            f.write(reinterpret_cast<const char*>(&dmap_res_), sizeof(dmap_res_));
            f.write(reinterpret_cast<const char*>(dmap_.data()),
                    static_cast<std::streamsize>(dmap_.size() * sizeof(float)));
        }

        return dir.string();
    }

} // namespace marrtino::io
