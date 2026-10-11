#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <chrono>
#include <cstring>
#include <limits>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "marrtino_core/control/cbf_filter.hpp"
#include "marrtino_core/control/trajectory_tracker.hpp"
#include "marrtino_core/obstacles/detector.hpp"
#include "marrtino_core/obstacles/tracker.hpp"
#include "marrtino_core/replanning/replanner.hpp"
#include "marrtino_core/io/map_loader.hpp"
#include "marrtino_core/io/plot_recorder.hpp"
#include "marrtino_core/mapping/distance_transform.hpp"
#include "marrtino_core/planning/astar.hpp"
#include "marrtino_core/planning/path_smoother.hpp"
#include "marrtino_core/trajectory/spline.hpp"
#include "marrtino_core/trajectory/types.hpp"
#include "marrtino_core/trajectory/velocity_profile.hpp"

namespace marrtino_nav {

    using namespace std::chrono_literals;

    double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q){
        const double siny = 2.0 * (q.w * q.z + q.x * q.y);
        const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        return std::atan2(siny, cosy);
    }

    class NavNode : public rclcpp::Node{
     public:
        NavNode() : rclcpp::Node("marrtino_nav"){
            map_yaml_ = declare_parameter("map_yaml", std::string(""));
            frame_id_ = declare_parameter("frame_id", std::string("odom"));
            goal_x_ = declare_parameter("goal_x", 0.0);
            goal_y_ = declare_parameter("goal_y", 0.0);
            auto_plan_ = declare_parameter("auto_plan", true);

            const auto wx = declare_parameter("waypoints_x", std::vector<double>{});
            const auto wy = declare_parameter("waypoints_y", std::vector<double>{});
            if(wx.size() != wy.size()){
                RCLCPP_ERROR(get_logger(), "waypoints_x e waypoints_y hanno lunghezze diverse");
            }else{
                for(std::size_t i = 0; i < wx.size(); ++i){
                    waypoints_.push_back(marrtino::grid::Point2{wx[i], wy[i]});
                }
            }
            flip_omega_ = declare_parameter("flip_omega", true);
            mirror_odometry_ = declare_parameter("mirror_odometry", true);
            pose_topic_ = declare_parameter("pose_topic", std::string("/localized_pose"));


            // Tre interruttori indipendenti: permettono di isolare quale dei tre
            // livelli reattivi introduce un problema, senza indovinare.
            detection_enabled_  = declare_parameter("detection_enabled", true);
            cbf_enabled_        = declare_parameter("cbf_enabled", true);
            replanning_enabled_ = declare_parameter("replanning_enabled", true);
            max_futile_replans_ = static_cast<int>(declare_parameter("max_futile_replans", 3));
            futile_replan_progress_ = declare_parameter("futile_replan_progress", 0.30);
            diagnostics_period_ = declare_parameter("diagnostics_period", 2.0);
            record_plot_data_ = declare_parameter("record_plot_data", true);
            plot_sample_period_ = declare_parameter("plot_sample_period", 0.05);
            plot_dir_ = declare_parameter("plot_dir", std::string("plots"));
            recorder_ = std::make_unique<marrtino::io::PlotRecorder>(plot_sample_period_);

            // Tutti i parametri dei moduli, esposti come parametri ROS.
            // Generati dalla stessa tabella che produce nav_params.yaml.
            astar_params_.repulsion_gain = static_cast<float>(declare_parameter("astar_repulsion_gain", static_cast<double>(astar_params_.repulsion_gain)));
            astar_params_.epsilon = static_cast<float>(declare_parameter("astar_epsilon", static_cast<double>(astar_params_.epsilon)));
            astar_params_.start_escape_radius = static_cast<float>(declare_parameter("start_escape_radius", static_cast<double>(astar_params_.start_escape_radius)));
            astar_params_.relax_clearance = static_cast<float>(declare_parameter("waypoint_relax_clearance", static_cast<double>(astar_params_.relax_clearance)));
            astar_params_.relax_radius = static_cast<float>(declare_parameter("waypoint_relax_radius", static_cast<double>(astar_params_.relax_radius)));
            smoother_params_.tunnel_half_width = static_cast<float>(declare_parameter("tunnel_half_width", static_cast<double>(smoother_params_.tunnel_half_width)));
            smoother_params_.collinear_tolerance_deg = static_cast<float>(declare_parameter("collinear_tolerance_deg", static_cast<double>(smoother_params_.collinear_tolerance_deg)));
            smoother_params_.min_vertex_spacing = static_cast<float>(declare_parameter("min_vertex_spacing", static_cast<double>(smoother_params_.min_vertex_spacing)));
            spline_params_.samples_per_segment = static_cast<int>(declare_parameter("samples_per_segment", spline_params_.samples_per_segment));
            profile_params_.cruise_speed = declare_parameter("cruise_speed", profile_params_.cruise_speed);
            profile_params_.v_max = declare_parameter("v_max", profile_params_.v_max);
            profile_params_.v_min = declare_parameter("v_min", profile_params_.v_min);
            profile_params_.a_max = declare_parameter("a_max", profile_params_.a_max);
            profile_params_.omega_max = declare_parameter("profile_omega_max", profile_params_.omega_max);
            profile_params_.control_period = declare_parameter("control_period", profile_params_.control_period);
            tracker_params_.b = declare_parameter("b", tracker_params_.b);
            tracker_params_.zeta = declare_parameter("zeta", tracker_params_.zeta);
            tracker_params_.omega_max = declare_parameter("omega_max", tracker_params_.omega_max);
            tracker_params_.max_linear_accel = declare_parameter("max_linear_accel", tracker_params_.max_linear_accel);
            tracker_params_.max_angular_accel = declare_parameter("max_angular_accel", tracker_params_.max_angular_accel);
            tracker_params_.reference_max_lag = declare_parameter("reference_max_lag", tracker_params_.reference_max_lag);
            tracker_params_.goal_tolerance = declare_parameter("goal_tolerance", tracker_params_.goal_tolerance);
            cbf_params_.b_look_ahead = declare_parameter("b_look_ahead", cbf_params_.b_look_ahead);
            cbf_params_.r_safe_dynamic = declare_parameter("r_safe_dynamic", cbf_params_.r_safe_dynamic);
            cbf_params_.r_safe_static = declare_parameter("r_safe_static", cbf_params_.r_safe_static);
            cbf_params_.robot_body_radius = declare_parameter("robot_body_radius", cbf_params_.robot_body_radius);
            cbf_params_.alpha_dynamic = declare_parameter("alpha_dynamic", cbf_params_.alpha_dynamic);
            cbf_params_.alpha_static = declare_parameter("alpha_static", cbf_params_.alpha_static);
            cbf_params_.gamma_clf = declare_parameter("gamma_clf", cbf_params_.gamma_clf);
            cbf_params_.slack_penalty = declare_parameter("slack_penalty", cbf_params_.slack_penalty);
            cbf_params_.cbf_slack_penalty = declare_parameter("cbf_slack_penalty", cbf_params_.cbf_slack_penalty);
            cbf_params_.command_smoothing_weight = declare_parameter("command_smoothing_weight", cbf_params_.command_smoothing_weight);
            cbf_params_.v_deviation_weight = declare_parameter("v_deviation_weight", cbf_params_.v_deviation_weight);
            cbf_params_.w_deviation_weight = declare_parameter("w_deviation_weight", cbf_params_.w_deviation_weight);
            cbf_params_.static_activation_distance = declare_parameter("static_activation_distance", cbf_params_.static_activation_distance);
            cbf_params_.obstacle_activation_range = declare_parameter("obstacle_activation_range", cbf_params_.obstacle_activation_range);
            cbf_params_.gradient_step_cells = declare_parameter("gradient_step_cells", cbf_params_.gradient_step_cells);
            cbf_params_.activation_tolerance = declare_parameter("activation_tolerance", cbf_params_.activation_tolerance);
            cbf_params_.w_max_cbf = declare_parameter("w_max_cbf", cbf_params_.w_max_cbf);
            detector_params_.detection_voxel_size = declare_parameter("detection_voxel_size", detector_params_.detection_voxel_size);
            detector_params_.self_hit_radius = declare_parameter("self_hit_radius", detector_params_.self_hit_radius);
            detector_params_.obs_tol = declare_parameter("obs_tol", detector_params_.obs_tol);
            detector_params_.obs_tol_per_meter = declare_parameter("obs_tol_per_meter", detector_params_.obs_tol_per_meter);
            detector_params_.max_detection_range = declare_parameter("max_detection_range", detector_params_.max_detection_range);
            detector_params_.obstacle_z_min = declare_parameter("obstacle_z_min", detector_params_.obstacle_z_min);
            detector_params_.obstacle_z_max = declare_parameter("obstacle_z_max", detector_params_.obstacle_z_max);
            detector_params_.max_elevation_deg = declare_parameter("max_elevation_deg", detector_params_.max_elevation_deg);
            detector_params_.clustering_radius = declare_parameter("clustering_radius", detector_params_.clustering_radius);
            detector_params_.min_cluster_points = static_cast<int>(declare_parameter("min_cluster_points", detector_params_.min_cluster_points));
            detector_params_.min_cluster_points_near = static_cast<int>(declare_parameter("min_cluster_points_near", detector_params_.min_cluster_points_near));
            detector_params_.near_cluster_range = declare_parameter("near_cluster_range", detector_params_.near_cluster_range);
            detector_params_.cluster_margin = declare_parameter("cluster_margin", detector_params_.cluster_margin);
            detector_params_.max_unexplained_fraction = declare_parameter("max_unexplained_fraction", detector_params_.max_unexplained_fraction);
            detector_params_.min_suspicious_spread = declare_parameter("min_suspicious_spread", detector_params_.min_suspicious_spread);
            detector_params_.laser_height = declare_parameter("laser_height", detector_params_.laser_height);
            detector_params_.use_chassis_footprint = declare_parameter("use_chassis_footprint", detector_params_.use_chassis_footprint);
            detector_params_.chassis_front = declare_parameter("chassis_front", detector_params_.chassis_front);
            detector_params_.chassis_rear = declare_parameter("chassis_rear", detector_params_.chassis_rear);
            detector_params_.chassis_half_width = declare_parameter("chassis_half_width", detector_params_.chassis_half_width);
            detector_params_.chassis_top_rel_laser = declare_parameter("chassis_top_rel_laser", detector_params_.chassis_top_rel_laser);
            detector_params_.chassis_margin = declare_parameter("chassis_margin", detector_params_.chassis_margin);
            obstacle_tracker_params_.gate = declare_parameter("track_gate", obstacle_tracker_params_.gate);
            obstacle_tracker_params_.alpha_lowpass = declare_parameter("track_alpha_lowpass", obstacle_tracker_params_.alpha_lowpass);
            obstacle_tracker_params_.v_deadzone = declare_parameter("track_v_deadzone", obstacle_tracker_params_.v_deadzone);
            obstacle_tracker_params_.min_hits = static_cast<int>(declare_parameter("track_min_hits", obstacle_tracker_params_.min_hits));
            obstacle_tracker_params_.forget_time = declare_parameter("track_forget_time", obstacle_tracker_params_.forget_time);
            obstacle_tracker_params_.blind_zone_radius = declare_parameter("blind_zone_radius", obstacle_tracker_params_.blind_zone_radius);
            obstacle_tracker_params_.blind_zone_forget_factor = declare_parameter("blind_zone_forget_factor", obstacle_tracker_params_.blind_zone_forget_factor);
            obstacle_tracker_params_.blind_zone_growth_rate = declare_parameter("blind_zone_growth_rate", obstacle_tracker_params_.blind_zone_growth_rate);
            obstacle_tracker_params_.max_blind_growth = declare_parameter("max_blind_growth", obstacle_tracker_params_.max_blind_growth);
            obstacle_tracker_params_.radius_decay_per_update = declare_parameter("radius_decay_per_update", obstacle_tracker_params_.radius_decay_per_update);
            obstacle_tracker_params_.min_points_for_update = static_cast<int>(declare_parameter("min_points_for_update", obstacle_tracker_params_.min_points_for_update));
            replanner_params_.persistent_obstacle_age = declare_parameter("persistent_obstacle_age", replanner_params_.persistent_obstacle_age);
            replanner_params_.look_ahead_distance = declare_parameter("replan_look_ahead_distance", replanner_params_.look_ahead_distance);
            replanner_params_.obstacle_inflation_margin = declare_parameter("obstacle_inflation_margin", replanner_params_.obstacle_inflation_margin);
            replanner_params_.reduced_inflation_factor = declare_parameter("reduced_inflation_factor", replanner_params_.reduced_inflation_factor);
            replanner_params_.stall_window = declare_parameter("stall_window", replanner_params_.stall_window);
            replanner_params_.stall_min_progress = declare_parameter("stall_min_progress", replanner_params_.stall_min_progress);
            replanner_params_.replan_max_deviation = declare_parameter("replan_max_deviation", replanner_params_.replan_max_deviation);
            replanner_params_.deviation_window = declare_parameter("deviation_window", replanner_params_.deviation_window);
            replanner_params_.max_infeasible_steps = static_cast<int>(declare_parameter("max_infeasible_steps", replanner_params_.max_infeasible_steps));
            replanner_params_.min_cbf_engagement = declare_parameter("min_cbf_engagement", replanner_params_.min_cbf_engagement);
            replanner_params_.engagement_margin_threshold = declare_parameter("engagement_margin_threshold", replanner_params_.engagement_margin_threshold);
            replanner_params_.engagement_deviation_threshold = declare_parameter("engagement_deviation_threshold", replanner_params_.engagement_deviation_threshold);
            replanner_params_.min_replan_interval = declare_parameter("min_replan_interval", replanner_params_.min_replan_interval);
            replanner_params_.replan_cooldown = declare_parameter("replan_cooldown", replanner_params_.replan_cooldown);
            replanner_params_.replan_retry_period = declare_parameter("replan_retry_period", replanner_params_.replan_retry_period);
            replanner_params_.max_replan_failures = static_cast<int>(declare_parameter("max_replan_failures", replanner_params_.max_replan_failures));

            cbf_params_.v_max = tracker_params_.v_max;
            profile_params_.v_max = tracker_params_.v_max;

            // ================================================================
            //  Parametri DERIVATI.
            //  Solo relazioni reali, non accoppiamenti inventati: ognuna ha una
            //  giustificazione geometrica o di controllo. Con derive_parameters
            //  a false valgono i valori letti dallo yaml.
            // ================================================================
            const bool derive = declare_parameter("derive_parameters", true);

            const double safety_margin     = declare_parameter("safety_margin", 0.0);
            const double omega_nominal     = declare_parameter("omega_nominal", 0.8);
            const double cbf_omega_factor  = declare_parameter("cbf_omega_factor", 1.125);
            const double clamp_omega_factor= declare_parameter("clamp_omega_factor", 1.5);
            const double omega_slew_time   = declare_parameter("omega_slew_time", 0.8);
            const double v_slew_time       = declare_parameter("v_slew_time", 0.67);
            const double lag_safety_factor = declare_parameter("lag_safety_factor", 2.0);
            const double v_min_fraction    = declare_parameter("v_min_fraction", 0.625);
            const double path_clearance_margin = declare_parameter("path_clearance_margin", 0.02);

            if(derive){
                // 1-2. Geometria: la barriera agisce sul punto avanzato p_b, che sta
                //      b_la davanti al centro. Perche' sia la SCOCCA a essere garantita
                //      serve r_safe >= r_corpo + b_la (disuguaglianza triangolare inversa).
                // Ostacoli dinamici: si affrontano di petto, quindi al punto avanzato
                // si chiede il caso peggiore della disuguaglianza triangolare inversa.
                // Il non contatto e' comunque garantito dalla riga sul corpo.
                cbf_params_.r_safe_dynamic = cbf_params_.robot_body_radius + cbf_params_.b_look_ahead + safety_margin;

                // Mappa statica: lungo un corridoio si viaggia PARALLELI al muro, e il
                // punto avanzato ha la stessa clearance del centro. Chiedergli
                // r_corpo + b_la renderebbe impercorribili i passaggi stretti senza
                // aggiungere sicurezza reale, che viene dalla riga sul corpo.
                // Il filtro chiede al punto avanzato esattamente cio' che il
                // pianificatore ha garantito al percorso: cosi' non lo combatte.
                smoother_params_.tunnel_half_width =
                    static_cast<float>(cbf_params_.robot_body_radius + path_clearance_margin);
                cbf_params_.r_safe_static = smoother_params_.tunnel_half_width;

                // 3-5. Gerarchia dei tetti su omega: il profilo pianifica conservativo,
                //      il filtro puo' chiedere di piu' per evitare, la saturazione e' l'ultima rete.
                profile_params_.omega_max = omega_nominal;
                cbf_params_.w_max_cbf     = omega_nominal * cbf_omega_factor;
                tracker_params_.omega_max = omega_nominal * clamp_omega_factor;

                // 6-7. Slew come "tempo per andare da zero al massimo".
                tracker_params_.max_angular_accel = tracker_params_.omega_max / omega_slew_time;
                tracker_params_.max_linear_accel  = tracker_params_.v_max / v_slew_time;

                // 8. Quando la guardia sul ritardo congela il riferimento, il robot si
                //    assesta a un offset di equilibrio e1 = -v_des/k1. Con
                //    k1 = 2*zeta*sqrt(b*v^2) quell'offset vale 1/(2*zeta*sqrt(b)) e NON
                //    dipende dalla velocita'. Se la soglia e' sotto l'offset la guardia
                //    non si riapre piu' e il robot resta piantato.
                const double equilibrium_offset = 1.0 / (2.0 * tracker_params_.zeta * std::sqrt(tracker_params_.b));
                tracker_params_.reference_max_lag = lag_safety_factor * equilibrium_offset;

                // 9. Il pavimento del profilo e' una frazione della crociera.
                profile_params_.v_min = v_min_fraction * profile_params_.cruise_speed;

                cbf_params_.v_max = tracker_params_.v_max;
                profile_params_.v_max = tracker_params_.v_max;

                RCLCPP_INFO(get_logger(), "parametri derivati:");
                RCLCPP_INFO(get_logger(), "  r_safe dinamico   = %.3f m  (corpo %.2f + b_la %.2f + margine %.2f)",
                            cbf_params_.r_safe_dynamic, cbf_params_.robot_body_radius,
                            cbf_params_.b_look_ahead, safety_margin);
                RCLCPP_INFO(get_logger(), "  tunnel = r_safe statico = %.3f m  (corpo %.2f + margine percorso %.2f)",
                            static_cast<double>(smoother_params_.tunnel_half_width),
                            cbf_params_.robot_body_radius, path_clearance_margin);
                RCLCPP_INFO(get_logger(), "  garanzia di non contatto: riga sul CORPO a %.2f m, mappa e ostacoli",
                            cbf_params_.robot_body_radius);
                RCLCPP_INFO(get_logger(), "  omega prof/cbf/clamp = %.2f / %.2f / %.2f rad/s",
                            profile_params_.omega_max, cbf_params_.w_max_cbf, tracker_params_.omega_max);
                RCLCPP_INFO(get_logger(), "  slew lin/ang      = %.2f m/s^2 / %.2f rad/s^2",
                            tracker_params_.max_linear_accel, tracker_params_.max_angular_accel);
                RCLCPP_INFO(get_logger(), "  reference_max_lag = %.3f m  (offset di equilibrio %.3f m)",
                            tracker_params_.reference_max_lag, equilibrium_offset);
                RCLCPP_INFO(get_logger(), "  v_min             = %.3f m/s", profile_params_.v_min);

                // Coerenza fra pianificatore e filtro: se il tunnel LOS-PS e' piu' stretto
                // del raggio statico, il planner produce percorsi che il filtro rifiuta.
                if(smoother_params_.tunnel_half_width < cbf_params_.robot_body_radius){
                    RCLCPP_WARN(get_logger(),
                                "tunnel %.2f m < raggio della scocca %.2f m: il pianificatore accetterebbe "
                                "passaggi in cui il robot non entra.",
                                static_cast<double>(smoother_params_.tunnel_half_width),
                                cbf_params_.robot_body_radius);
                }
            }

            RCLCPP_INFO(get_logger(),
                        "sicurezza: r_safe_dyn=%.2f corpo=%.2f b_la=%.2f | relazione di progetto vuole r_safe >= %.2f",
                        cbf_params_.r_safe_dynamic, cbf_params_.robot_body_radius, cbf_params_.b_look_ahead,
                        cbf_params_.robot_body_radius + cbf_params_.b_look_ahead);

            tracker_ = std::make_unique<marrtino::control::TrajectoryTracker>(tracker_params_);
            detector_ = std::make_unique<marrtino::obstacles::ObstacleDetector>(detector_params_);
            obstacle_tracker_ = std::make_unique<marrtino::obstacles::ObstacleTracker>(obstacle_tracker_params_);
            cbf_ = std::make_unique<marrtino::control::CBFFilter>(cbf_params_);
            replanner_ = std::make_unique<marrtino::replanning::Replanner>(replanner_params_);

            if(!loadMap()){
                RCLCPP_ERROR(get_logger(), "mappa non caricata: il nodo resta inattivo");
                return;
            }

            rclcpp::QoS latched(1);
            latched.transient_local();

            map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/cpp/map", latched);
            path_pub_ = create_publisher<nav_msgs::msg::Path>("/cpp/planned_path", latched);
            trajectory_pub_ = create_publisher<nav_msgs::msg::Path>("/cpp/trajectory", latched);
            dmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/cpp/distance_map", latched);
            cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

            // La posa localizzata e' quella buona, ma Unity la pubblica SOLO mentre il
            // robot si muove: all'avvio non arriverebbe mai e il nodo non pianificherebbe.
            // Quindi /odometry serve solo per sbloccare il primo piano (a t=0 e' esatta),
            // poi appena arriva una posa localizzata si passa a quella e non si torna piu' indietro.
            loc_sub_ = create_subscription<nav_msgs::msg::Odometry>(
                pose_topic_, rclcpp::SensorDataQoS(),
                [this](nav_msgs::msg::Odometry::SharedPtr msg){ onPose(*msg, false); });

            odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
                "/odometry", rclcpp::SensorDataQoS(),
                [this](nav_msgs::msg::Odometry::SharedPtr msg){ onPose(*msg, true); });

            truth_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
                "/debug/true_pose", rclcpp::SensorDataQoS(),
                [this](sensor_msgs::msg::PointCloud2::SharedPtr msg){
                    if(msg->data.size() >= 8){
                        float xy[2];
                        std::memcpy(xy, msg->data.data(), sizeof(xy));
                        if(std::isfinite(xy[0]) && std::isfinite(xy[1])){
                            truth_.x = xy[0];
                            truth_.y = xy[1];
                            has_truth_ = true;
                        }
                    }
                });

            scan_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
                "/point_cloud", rclcpp::SensorDataQoS(),
                [this](sensor_msgs::msg::PointCloud2::SharedPtr msg){ onScan(*msg); });

            obstacles_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/cpp/obstacles", 10);

            goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
                "/goal_pose", 10,
                [this](geometry_msgs::msg::PoseStamped::SharedPtr msg){ onGoal(*msg); });

            cbf_->setStaticMap(&edt_);
            if(cbf_enabled_){
                tracker_->setSafetyFilter(
                    [this](const marrtino::trajectory::Command& u_nom,
                           const marrtino::trajectory::Pose2D& pose,
                           const marrtino::control::TrajectoryTracker::TrackingTerms& t){
                        return cbf_->filter(u_nom, pose, t.e1, t.e2, t.e3, t.k2, t.v_des, t.omega_des);
                    });
            }

            for(const auto& kv : std::vector<std::pair<std::string,double>>{
                    {"vMax", tracker_params_.v_max}, {"linearMeanVelocity", profile_params_.cruise_speed},
                    {"wMax", profile_params_.omega_max}, {"wMaxClamp", tracker_params_.omega_max},
                    {"wMaxCBF", cbf_params_.w_max_cbf}, {"aMax", profile_params_.a_max},
                    {"maxLinearAccelCommand", tracker_params_.max_linear_accel},
                    {"maxAngularAccelCommand", tracker_params_.max_angular_accel},
                    {"rSafeStatic", cbf_params_.r_safe_static}, {"rSafeDynamic", cbf_params_.r_safe_dynamic},
                    {"robotBodyRadius", cbf_params_.robot_body_radius}, {"bLookAhead", cbf_params_.b_look_ahead},
                    {"epsilonTunnelLoSPS", smoother_params_.tunnel_half_width},
                    {"commandSmoothingWeight", cbf_params_.command_smoothing_weight},
                    {"trackForgetTime", obstacle_tracker_params_.forget_time},
                    {"blindZoneRadius", obstacle_tracker_params_.blind_zone_radius}}){
                recorder_->setLimit(kv.first, kv.second);
            }

            recorder_->setDistanceMap(edt_);

            publishMap();
            publishDistanceMap();

            const auto period = std::chrono::duration<double>(profile_params_.control_period);
            timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                                       [this](){ onControlStep(); });

            dump_timer_ = create_wall_timer(30s, [this](){
                if(record_plot_data_ && !recorder_->empty()){
                    plot_dumped_ = false;
                    dumpPlotData();
                }
            });

            republish_timer_ = create_wall_timer(1s, [this](){
                publishMap();
                publishDistanceMap();
                if(!last_smoothed_.empty()){
                    publishPath(path_pub_, last_smoothed_);
                    publishPath(trajectory_pub_, last_dense_);
                }
            });

            RCLCPP_INFO(get_logger(), "pronto: mappa %dx%d a %.3f m/cella, in attesa di /odometry",
                        edt_.geometry().width(), edt_.geometry().height(), edt_.geometry().resolution());
        }

        ~NavNode() override{
            dumpPlotData();
        }

     private:
        bool loadMap(){
            if(map_yaml_.empty()){
                RCLCPP_ERROR(get_logger(), "parametro map_yaml non impostato");
                return false;
            }

            std::string error;
            auto loaded = marrtino::io::loadOccupancyMap(map_yaml_, &error);
            if(!loaded){
                RCLCPP_ERROR(get_logger(), "%s", error.c_str());
                return false;
            }

            map_ = std::move(*loaded);
            edt_ = marrtino::mapping::computeDistanceTransform(map_, marrtino::io::kOccupiedThreshold);
            map_ready_ = true;
            return true;
        }

        void publishMap(){
            const auto& geometry = map_.geometry();

            nav_msgs::msg::OccupancyGrid msg;
            msg.header.frame_id = frame_id_;
            msg.header.stamp = now();
            msg.info.resolution = static_cast<float>(geometry.resolution());
            msg.info.width = static_cast<unsigned int>(geometry.width());
            msg.info.height = static_cast<unsigned int>(geometry.height());
            msg.info.origin.position.x = geometry.origin().x;
            msg.info.origin.position.y = geometry.origin().y;
            msg.info.origin.orientation.w = 1.0;
            msg.data.assign(map_.cells().begin(), map_.cells().end());

            map_pub_->publish(msg);
        }

        void publishDistanceMap(){
            const auto& geometry = edt_.geometry();

            float max_distance = 0.0f;
            for(const float d : edt_.cells()){
                if(std::isfinite(d) && d > max_distance){
                    max_distance = d;
                }
            }
            if(max_distance <= 0.0f){
                max_distance = 1.0f;
            }

            nav_msgs::msg::OccupancyGrid msg;
            msg.header.frame_id = frame_id_;
            msg.header.stamp = now();
            msg.info.resolution = static_cast<float>(geometry.resolution());
            msg.info.width = static_cast<unsigned int>(geometry.width());
            msg.info.height = static_cast<unsigned int>(geometry.height());
            msg.info.origin.position.x = geometry.origin().x;
            msg.info.origin.position.y = geometry.origin().y;
            msg.info.origin.orientation.w = 1.0;

            msg.data.resize(edt_.size());
            for(std::size_t i = 0; i < edt_.size(); ++i){
                const float d = edt_[i];
                if(!std::isfinite(d)){
                    msg.data[i] = 0;
                }else{
                    msg.data[i] = static_cast<std::int8_t>(100.0f * d / max_distance);
                }
            }

            dmap_pub_->publish(msg);
        }

        void publishPath(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr& pub,
                         const marrtino::trajectory::Path& path){
            nav_msgs::msg::Path msg;
            msg.header.frame_id = frame_id_;
            msg.header.stamp = now();

            for(const auto& p : path){
                geometry_msgs::msg::PoseStamped pose;
                pose.header = msg.header;
                pose.pose.position.x = p.x;
                pose.pose.position.y = p.y;
                pose.pose.orientation.w = 1.0;
                msg.poses.push_back(pose);
            }

            pub->publish(msg);
        }

        void onPose(const nav_msgs::msg::Odometry& msg, bool from_odometry){
            if(from_odometry){
                // Serve comunque per il confronto nella figura: e' la curva di deriva.
                odom_pose_.x = msg.pose.pose.position.x;
                odom_pose_.y = mirror_odometry_ ? -msg.pose.pose.position.y : msg.pose.pose.position.y;
                const double yaw = yawFromQuaternion(msg.pose.pose.orientation);
                odom_pose_.theta = mirror_odometry_ ? -yaw : yaw;
                if(localized_){
                    return;
                }
            }
            if(!from_odometry && !localized_){
                localized_ = true;
                RCLCPP_INFO(get_logger(), "posa localizzata ricevuta su %s: si abbandona /odometry",
                            pose_topic_.c_str());
            }

            pose_.x = msg.pose.pose.position.x;
            pose_.y = msg.pose.pose.position.y;
            pose_.theta = yawFromQuaternion(msg.pose.pose.orientation);

            // Solo /odometry e' nel frame interno dell'odometria, specchiato in y e theta
            // (Orchestrator.cs:617). La posa localizzata e' gia' nel frame della mappa.
            if(from_odometry && mirror_odometry_){
                pose_.y = -pose_.y;
                pose_.theta = -pose_.theta;
            }

            if(!pose_ready_){
                pose_ready_ = true;
                RCLCPP_INFO(get_logger(), "prima posa (%s): (%.2f, %.2f, %.1f deg)",
                            from_odometry ? "/odometry" : pose_topic_.c_str(),
                            pose_.x, pose_.y, pose_.theta * 57.29577951308232);
                if(auto_plan_){
                    if(!waypoints_.empty()){
                        planThrough(waypoints_);
                    }else{
                        planTo(marrtino::grid::Point2{goal_x_, goal_y_});
                    }
                }
            }
        }

        void onScan(const sensor_msgs::msg::PointCloud2& msg){
            if(!detection_enabled_ || !map_ready_ || !pose_ready_){
                return;
            }

            // /point_cloud arriva gia' in convenzione ROS, ma nel frame del LASER.
            std::vector<std::array<double,3>> points;
            points.reserve(msg.width * msg.height);

            const std::size_t stride = msg.point_step;
            for(std::size_t i = 0; i + stride <= msg.data.size(); i += stride){
                float xyz[3];
                std::memcpy(xyz, msg.data.data() + i, sizeof(xyz));
                if(std::isfinite(xyz[0]) && std::isfinite(xyz[1]) && std::isfinite(xyz[2])){
                    points.push_back({static_cast<double>(xyz[0]),
                                      static_cast<double>(xyz[1]),
                                      static_cast<double>(xyz[2])});
                }
            }

            const double stamp = now().seconds();
            const auto detections = detector_->detect(points, pose_, edt_);
            obstacle_tracker_->update(detections, stamp, pose_);

            const auto confirmed = obstacle_tracker_->confirmed();
            if(cbf_enabled_){
                cbf_->setObstacles(confirmed);
            }
            publishObstacles(confirmed);

            const auto& d = detector_->diagnostics();
            if(d.pose_suspicious){
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                     "scansione scartata: posa sospetta (%d punti non spiegati)", d.unexplained);
            }

            if(stamp - last_diagnostics_ > diagnostics_period_){
                last_diagnostics_ = stamp;
                double nearest = std::numeric_limits<double>::infinity();
                for(const auto& t : confirmed){
                    nearest = std::min(nearest, std::hypot(t.cx - pose_.x, t.cy - pose_.y) - t.r);
                }
                RCLCPP_INFO(get_logger(),
                            "detect: in=%d voxel=%d banda=%d range=%d nonspiegati=%d cluster=%d autohit=%d | track=%zu piu_vicino=%.2f m",
                            d.input, d.after_voxel, d.in_band, d.in_range, d.unexplained, d.clustered,
                            d.rejected_self_points, confirmed.size(),
                            std::isfinite(nearest) ? nearest : -1.0);
            }
        }

        void publishObstacles(const marrtino::obstacles::Tracks& tracks){
            visualization_msgs::msg::MarkerArray array;

            visualization_msgs::msg::Marker clear;
            clear.header.frame_id = frame_id_;
            clear.action = visualization_msgs::msg::Marker::DELETEALL;
            array.markers.push_back(clear);

            for(const auto& t : tracks){
                visualization_msgs::msg::Marker m;
                m.header.frame_id = frame_id_;
                m.header.stamp = now();
                m.ns = "obstacles";
                m.id = t.id;
                m.type = visualization_msgs::msg::Marker::CYLINDER;
                m.action = visualization_msgs::msg::Marker::ADD;
                m.pose.position.x = t.cx;
                m.pose.position.y = t.cy;
                m.pose.orientation.w = 1.0;
                m.scale.x = 2.0 * t.r;
                m.scale.y = 2.0 * t.r;
                m.scale.z = 0.4;
                m.color.r = 1.0f;
                m.color.a = 0.5f;
                array.markers.push_back(m);
            }

            obstacles_pub_->publish(array);
        }

        void onGoal(const geometry_msgs::msg::PoseStamped& msg){
            planTo(marrtino::grid::Point2{msg.pose.position.x, msg.pose.position.y});
        }

        void planTo(const marrtino::grid::Point2& goal){
            planThrough({goal});
        }

        // Pianifica segmento per segmento, come DetermineGeometricTrajectoryFromWaypoints
        // nel C#. Ogni segmento riparte da dove il precedente e' ARRIVATO: se un
        // waypoint cade dentro un ostacolo viene rilassato, e ripartire dal nominale
        // spezzerebbe la spezzata.
        void planThrough(const marrtino::trajectory::Path& targets){
            planThroughOn(edt_, targets);
        }

        bool planThroughOn(const marrtino::grid::Grid<float>& edt, const marrtino::trajectory::Path& targets){
            if(!map_ready_ || !pose_ready_ || targets.empty()){
                return false;
            }

            const auto& geometry = edt.geometry();
            marrtino::grid::Cell from = geometry.worldToCell(marrtino::grid::Point2{pose_.x, pose_.y});

            marrtino::trajectory::Path stitched;
            int relaxed = 0;

            for(std::size_t i = 0; i < targets.size(); ++i){
                const marrtino::grid::Cell to = geometry.worldToCell(targets[i]);

                auto segment = marrtino::planning::findPath(edt, from, to, astar_params_);
                if(!segment){
                    RCLCPP_WARN(get_logger(), "nessun percorso per il waypoint %zu (%.2f, %.2f)",
                                i + 1, targets[i].x, targets[i].y);
                    return false;
                }
                if(segment->goal_was_relaxed){
                    ++relaxed;
                }

                const auto smoothed = marrtino::planning::smoothPath(edt, segment->path, smoother_params_);
                for(std::size_t k = (stitched.empty() ? 0u : 1u); k < smoothed.size(); ++k){
                    stitched.push_back(smoothed[k]);
                }

                from = segment->reached_goal;
            }

            if(stitched.size() < 2){
                RCLCPP_WARN(get_logger(), "spezzata degenere");
                return false;
            }

            const auto dense = marrtino::trajectory::buildSpline(stitched, spline_params_);
            auto trajectory = marrtino::trajectory::buildTrajectory(dense, profile_params_);
            if(trajectory.empty()){
                RCLCPP_WARN(get_logger(), "profilo di velocita' vuoto");
                return false;
            }

            last_smoothed_ = stitched;
            last_dense_ = dense;
            current_trajectory_ = trajectory;

            if(record_plot_data_){
                if(start_time_ < 0.0){
                    start_time_ = stamp_seconds(now());
                }
                recorder_->recordPlan(plan_id_++, stamp_seconds(now()) - start_time_, dense);
            }
            publishPath(path_pub_, stitched);
            publishPath(trajectory_pub_, dense);

            RCLCPP_INFO(get_logger(),
                        "piano su %zu waypoint (%d rilassati): %zu vertici -> %zu campioni, durata %.1f s",
                        targets.size(), relaxed, stitched.size(), trajectory.size(), trajectory.back().t);

            goal_reported_ = false;
            tracker_->setTrajectory(std::move(trajectory));
            return true;
        }

        static double stamp_seconds(const rclcpp::Time& t){
            return t.seconds();
        }

        double distanceToPlan() const{
            double best = std::numeric_limits<double>::infinity();
            for(const auto& p : last_dense_){
                const double dx = p.x - pose_.x;
                const double dy = p.y - pose_.y;
                best = std::min(best, std::sqrt(dx * dx + dy * dy));
            }
            return std::isfinite(best) ? best : 0.0;
        }

        // Ripianifica sulla mappa pristina con i soli track persistenti dipinti sopra,
        // scalando il margine se il primo tentativo non trova strada.
        void executeReplanning(marrtino::replanning::Trigger trigger, double stamp){
            RCLCPP_INFO(get_logger(), "replanning: trigger %d", static_cast<int>(trigger));

            const auto tracks = obstacle_tracker_->confirmed();
            const double base = replanner_->params().obstacle_inflation_margin;
            const double margins[3] = {base, base * replanner_->params().reduced_inflation_factor, 0.0};

            for(const double margin : margins){
                const auto inflated = marrtino::replanning::Replanner::inflate(map_, tracks, margin);
                const auto inflated_edt = marrtino::mapping::computeDistanceTransform(inflated, marrtino::io::kOccupiedThreshold);

                const auto remaining = remainingWaypoints();
                if(planThroughOn(inflated_edt, remaining)){
                    replanner_->notifyReplanResult(true, stamp);
                    if(record_plot_data_){
                        recorder_->recordReplan(stamp - start_time_, pose_, triggerName(trigger), true,
                                                margin, static_cast<int>(tracks.size()),
                                                replanner_->failureCount(), "none");
                    }
                    RCLCPP_INFO(get_logger(), "replanning riuscito con margine %.2f m", margin);
                    return;
                }
            }

            replanner_->notifyReplanResult(false, stamp);
            if(record_plot_data_){
                recorder_->recordReplan(stamp - start_time_, pose_, triggerName(trigger), false,
                                        0.0, static_cast<int>(tracks.size()),
                                        replanner_->failureCount(), "noPath");
            }
            RCLCPP_WARN(get_logger(), "replanning fallito su tutti i margini");
        }

        // I waypoint non ancora raggiunti, in ordine. L'ordine della missione non si tocca.
        marrtino::trajectory::Path remainingWaypoints() const{
            marrtino::trajectory::Path remaining;
            for(std::size_t i = next_waypoint_; i < waypoints_.size(); ++i){
                remaining.push_back(waypoints_[i]);
            }
            if(remaining.empty() && !waypoints_.empty()){
                remaining.push_back(waypoints_.back());
            }
            return remaining;
        }

        void updateNextWaypoint(){
            while(next_waypoint_ + 1 < waypoints_.size()){
                const double dx = waypoints_[next_waypoint_].x - pose_.x;
                const double dy = waypoints_[next_waypoint_].y - pose_.y;
                const double d2 = dx * dx + dy * dy;

                if(d2 < waypoint_reached_radius_ * waypoint_reached_radius_){
                    ++next_waypoint_;
                    continue;
                }

                // Il semipiano vale SOLO da vicino: su un percorso che si richiude,
                // un waypoint lontano risulta facilmente "oltrepassato" e verrebbe saltato.
                if(d2 < waypoint_passed_radius_ * waypoint_passed_radius_){
                    const double sx = waypoints_[next_waypoint_ + 1].x - waypoints_[next_waypoint_].x;
                    const double sy = waypoints_[next_waypoint_ + 1].y - waypoints_[next_waypoint_].y;
                    if((pose_.x - waypoints_[next_waypoint_].x) * sx +
                       (pose_.y - waypoints_[next_waypoint_].y) * sy > 0.0){
                        ++next_waypoint_;
                        continue;
                    }
                }
                break;
            }
        }

        static const char* triggerName(marrtino::replanning::Trigger t){
            switch(t){
                case marrtino::replanning::Trigger::PersistentObstacle: return "PersistentObstacle";
                case marrtino::replanning::Trigger::Stall:              return "Stall";
                case marrtino::replanning::Trigger::Deviation:          return "Deviation";
                case marrtino::replanning::Trigger::Infeasible:         return "Infeasible";
                default:                                                return "None";
            }
        }

        void dumpPlotData(){
            if(!record_plot_data_ || plot_dumped_ || recorder_->empty()){
                return;
            }
            plot_dumped_ = true;
            const std::string dir = recorder_->dump(plot_dir_);
            if(dir.empty()){
                RCLCPP_WARN(get_logger(), "salvataggio dei dati di plot fallito");
            }else{
                RCLCPP_INFO(get_logger(), "dati di plot salvati in %s", dir.c_str());
            }
        }

        void onControlStep(){
            if(!pose_ready_ || !tracker_->isActive()){
                return;
            }

            const rclcpp::Time stamp = now();
            double dt = profile_params_.control_period;
            if(last_step_.nanoseconds() > 0){
                dt = (stamp - last_step_).seconds();
            }
            last_step_ = stamp;

            updateNextWaypoint();

            const auto command = tracker_->update(pose_, dt);

            if(record_plot_data_ && recorder_->isTimeToSample(stamp_seconds(now()))){
                const auto& c = cbf_->diagnostics();
                const auto& tt = tracker_->terms();
                const double lyap = 0.5 * tt.k2 * (tt.e1*tt.e1 + tt.e2*tt.e2) + 0.5 * tt.e3*tt.e3;
                recorder_->recordControl(stamp_seconds(now()) - start_time_, command, tracker_->lastNominal(),
                                         cbf_enabled_ && c.active, c.feasible,
                                         c.min_barrier, c.min_margin, c.static_margin,
                                         tt.e1, tt.e2, tt.e3, lyap);
                recorder_->recordPose(stamp_seconds(now()) - start_time_,
                                      has_truth_ ? truth_ : pose_, pose_, odom_pose_);
                if(detection_enabled_){
                    recorder_->recordObstacles(stamp_seconds(now()) - start_time_,
                                               obstacle_tracker_->confirmed());
                }
            }

            if(cbf_enabled_ && stamp_seconds(now()) - last_cbf_log_ > diagnostics_period_){
                last_cbf_log_ = stamp_seconds(now());
                const auto& c = cbf_->diagnostics();
                RCLCPP_INFO(get_logger(),
                            "cbf: attivo=%d feasible=%d dev=%.4f righe=%d margine=%.3f slack=%.3g | u=(%.3f,%.3f)",
                            c.active, c.feasible, c.deviation, c.constraints, c.min_margin, c.slack,
                            command.v, command.omega);
            }

            if(replanning_enabled_){
                const auto& diag = cbf_->diagnostics();
                const double tsec = stamp_seconds(now());
                replanner_->notifyControlStep(diag.feasible, diag.deviation, diag.min_margin, tsec);
                replanner_->notifyProgress(pose_, distanceToPlan(), tsec);
                replanner_->setContext(obstacle_tracker_->confirmed(), current_trajectory_,
                                       tracker_->referenceIndex(), tsec);

                auto trigger = replanner_->evaluate(tsec);

                // Un piano nuovo che non sposta il robot non e' una soluzione: senza
                // questa guardia uno stallo fisico fa ripianificare ogni
                // min_replan_interval all'infinito. Dopo N tentativi con lo stesso
                // trigger e senza progresso si smette e lo si dichiara.
                if(trigger != marrtino::replanning::Trigger::None){
                    const double moved = std::hypot(pose_.x - last_replan_pose_.x,
                                                    pose_.y - last_replan_pose_.y);
                    if(trigger == last_replan_trigger_ && moved < futile_replan_progress_){
                        ++futile_replans_;
                    }else{
                        futile_replans_ = 0;
                    }

                    if(futile_replans_ >= max_futile_replans_){
                        if(!futile_reported_){
                            futile_reported_ = true;
                            RCLCPP_ERROR(get_logger(),
                                         "BLOCCATO: %d ripianificazioni con trigger %s senza progresso "
                                         "(%.2f m). Smetto di ripianificare: il problema non e' il piano.",
                                         futile_replans_, triggerName(trigger), moved);
                        }
                        trigger = marrtino::replanning::Trigger::None;
                    }else{
                        last_replan_trigger_ = trigger;
                        last_replan_pose_ = pose_;
                        futile_reported_ = false;
                    }
                }

                if(trigger != marrtino::replanning::Trigger::None){
                    RCLCPP_WARN(get_logger(),
                                "trigger %d a t=%.1f: ingaggio=%.2fs dev_cbf=%.4f margine=%.3f dist_piano=%.3f",
                                static_cast<int>(trigger), tsec, replanner_->cbfEngagementTime(),
                                diag.deviation, diag.min_margin, distanceToPlan());
                    executeReplanning(trigger, tsec);
                }
            }

            geometry_msgs::msg::Twist msg;
            msg.linear.x = command.v;
            msg.angular.z = flip_omega_ ? -command.omega : command.omega;
            cmd_pub_->publish(msg);

            if(tracker_->isFinished() && !goal_reported_){
                goal_reported_ = true;
                RCLCPP_INFO(get_logger(), "traiettoria completata");
                dumpPlotData();
            }
        }

        std::string map_yaml_;
        std::string frame_id_;
        double goal_x_{};
        double goal_y_{};
        bool auto_plan_{true};
        bool flip_omega_{true};
        bool mirror_odometry_{true};
        std::string pose_topic_;
        marrtino::trajectory::Path waypoints_;
        std::size_t next_waypoint_{0};
        double waypoint_reached_radius_{0.4};
        double waypoint_passed_radius_{1.5};
        bool detection_enabled_{true};
        bool cbf_enabled_{true};
        bool replanning_enabled_{true};
        int max_futile_replans_{3};
        double futile_replan_progress_{0.30};
        int futile_replans_{0};
        bool futile_reported_{false};
        marrtino::replanning::Trigger last_replan_trigger_{marrtino::replanning::Trigger::None};
        marrtino::trajectory::Pose2D last_replan_pose_{};
        double diagnostics_period_{2.0};
        double last_diagnostics_{0.0};
        double last_cbf_log_{0.0};
        bool record_plot_data_{true};
        double plot_sample_period_{0.05};
        std::string plot_dir_;
        double start_time_{-1.0};
        int plan_id_{0};
        bool plot_dumped_{false};
        std::unique_ptr<marrtino::io::PlotRecorder> recorder_;
        marrtino::trajectory::Trajectory current_trajectory_;

        marrtino::io::OccupancyGrid map_;
        marrtino::grid::Grid<float> edt_;
        bool map_ready_{false};

        marrtino::trajectory::Pose2D pose_{};
        bool pose_ready_{false};
        bool localized_{false};
        bool has_truth_{false};
        marrtino::trajectory::Pose2D truth_{};
        marrtino::trajectory::Pose2D odom_pose_{};
        bool goal_reported_{false};
        rclcpp::Time last_step_{0, 0, RCL_ROS_TIME};

        marrtino::planning::AStarParams astar_params_{};
        marrtino::planning::SmootherParams smoother_params_{};
        marrtino::trajectory::SplineParams spline_params_{};
        marrtino::trajectory::ProfileParams profile_params_{};
        marrtino::control::TrackerParams tracker_params_{};
        marrtino::obstacles::DetectorParams detector_params_{};
        marrtino::obstacles::TrackerParams obstacle_tracker_params_{};
        marrtino::control::CBFParams cbf_params_{};
        marrtino::replanning::ReplannerParams replanner_params_{};

        std::unique_ptr<marrtino::control::TrajectoryTracker> tracker_;
        std::unique_ptr<marrtino::obstacles::ObstacleDetector> detector_;
        std::unique_ptr<marrtino::obstacles::ObstacleTracker> obstacle_tracker_;
        std::unique_ptr<marrtino::control::CBFFilter> cbf_;
        std::unique_ptr<marrtino::replanning::Replanner> replanner_;

        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr dmap_pub_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr loc_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr truth_sub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_pub_;
        marrtino::trajectory::Path last_smoothed_;
        marrtino::trajectory::Path last_dense_;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr republish_timer_;
        rclcpp::TimerBase::SharedPtr dump_timer_;
    };

} // namespace marrtino_nav

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<marrtino_nav::NavNode>());
    rclcpp::shutdown();
    return 0;
}
