#pragma once

#include <array>
#include <vector>

#include "marrtino_core/grid/grid.hpp"
#include "marrtino_core/obstacles/types.hpp"
#include "marrtino_core/trajectory/types.hpp"

namespace marrtino::obstacles {

    struct DetectorParams{
        double detection_voxel_size = 0.05;
        double self_hit_radius = 0.28;
        double obs_tol = 0.12;              // tolleranza "spiegato dalla mappa" a distanza zero [m]
        double obs_tol_per_meter = 0.02;    // ...e la sua crescita col range
        double max_detection_range = 3.5;
        double obstacle_z_min = 0.15;
        double obstacle_z_max = 1.0;
        double max_elevation_deg = 12.0;
        double clustering_radius = 0.3;
        int    min_cluster_points = 4;
        int    min_cluster_points_near = 2;
        double near_cluster_range = 1.5;
        double cluster_margin = 0.05;
        double max_unexplained_fraction = 0.5;
        double min_suspicious_spread = 2.0;
        double laser_height = 0.335;        // quota del laser sul pavimento [m]
        // Unity pubblica su /point_cloud coordinate MONDO, non offset dal laser.
        bool   points_in_map_frame = true;

        bool   use_chassis_footprint = true;
        double chassis_front         = 0.04;    // [m] davanti al laser
        double chassis_rear          = 0.24;
        double chassis_half_width    = 0.19;
        double chassis_top_rel_laser = -0.125;  // tetto scocca relativo alla quota laser
        double chassis_margin        = 0.04;
    };

    class ObstacleDetector{
        public:

        explicit ObstacleDetector(DetectorParams params = {});

        // points: punti della scansione nel frame del LASER, gia' in convenzione ROS
        //         (x avanti, y a sinistra, z in alto).
        // laser_pose: posa del laser nel frame MAPPA.
        // edt: distance transform della mappa STATICA (mai gonfiata).
        // Ritorna i cerchi di ingombro in frame MAPPA.
        Detections detect(const std::vector<std::array<double,3>>& points,
                          const trajectory::Pose2D& laser_pose,
                          const grid::Grid<float>& edt);

        // Diagnostica dell'ultima chiamata (contatori per filtro), per il logging.
        struct Diagnostics{ int input{}, after_voxel{}, in_band{}, in_range{}, unexplained{}, clustered{}, rejected_self{}, rejected_self_points{}; bool pose_suspicious{}; };
        [[nodiscard]] const Diagnostics& diagnostics() const;

        [[nodiscard]] const DetectorParams& params() const { return params_; }

        private:
        DetectorParams params_{};
        Diagnostics diagnostics_{};
    };

} // namespace marrtino::obstacles
