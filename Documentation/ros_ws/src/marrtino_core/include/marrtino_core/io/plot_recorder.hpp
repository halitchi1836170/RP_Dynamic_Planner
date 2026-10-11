// Registrazione delle serie temporali per la figura di analisi, nello STESSO
// formato di PlotDataService.cs: cosi' tools/plot_run.py funziona senza modifiche
// sia sui run guidati dal C# sia su quelli guidati dal C++.
#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include "marrtino_core/grid/grid.hpp"
#include "marrtino_core/obstacles/types.hpp"
#include "marrtino_core/trajectory/types.hpp"

namespace marrtino::io {

    class PlotRecorder{
     public:
        explicit PlotRecorder(double sample_period = 0.05);

        [[nodiscard]] bool isTimeToSample(double now) const;

        void recordControl(double t,
                           const trajectory::Command& commanded,
                           const trajectory::Command& nominal,
                           bool cbf_active, bool cbf_feasible,
                           double min_barrier, double min_margin, double static_margin,
                           double e1, double e2, double e3, double lyapunov);

        void recordPose(double t, const trajectory::Pose2D& ground_truth,
                        const trajectory::Pose2D& localized, const trajectory::Pose2D& odometry);

        void recordPlan(int plan_id, double t_arm, const trajectory::Path& path);

        void recordObstacles(double t, const obstacles::Tracks& tracks);

        void recordReplan(double t, const trajectory::Pose2D& pose, const std::string& trigger,
                          bool success, double margin, int tracks, int failures,
                          const std::string& reason);

        void setLimit(const std::string& name, double value);

        // La EDT va salvata UNA volta: e' lo sfondo del campo di barriera nella figura.
        void setDistanceMap(const grid::Grid<float>& edt);

        // Scrive tutto in <dir>/run_<timestamp>/. La cartella viene scelta UNA volta
        // e poi riscritta: un salvataggio periodico non deve lasciare una scia di
        // cartelle. Ritorna il percorso, o stringa vuota se non c'era nulla da salvare.
        std::string dump(const std::filesystem::path& base_dir);

        [[nodiscard]] bool empty() const;

     private:
        struct ControlRow{ double t, v_cmd, w_cmd, v_nom, w_nom; int active, feasible;
                           double min_barrier, min_margin, static_margin, e1, e2, e3, v_lyap; };
        struct PoseRow{ double t, gx, gy, gth, lx, ly, lth, ox, oy, oth; };
        struct PlanRow{ int plan_id; double t_arm, x, y; };
        struct ObstacleRow{ double t; int id; double cx, cy, r; };
        struct ReplanRow{ double t, x, y; std::string trigger; int success;
                          double margin; int tracks, failures; std::string reason; };

        double sample_period_;
        mutable double last_sample_{-1.0};

        std::vector<ControlRow> control_;
        std::vector<PoseRow> poses_;
        std::vector<PlanRow> plans_;
        std::vector<ObstacleRow> obstacles_;
        std::vector<ReplanRow> replans_;
        std::vector<std::pair<std::string, double>> limits_;

        bool has_dmap_{false};
        int dmap_w_{0};
        int dmap_h_{0};
        float dmap_ox_{0.0f};
        float dmap_oy_{0.0f};
        float dmap_res_{0.0f};
        std::vector<float> dmap_;

        std::string run_dir_;   // fissata al primo dump, poi riusata
    };

} // namespace marrtino::io
