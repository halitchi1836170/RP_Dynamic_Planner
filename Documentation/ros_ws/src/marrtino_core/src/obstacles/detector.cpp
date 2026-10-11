#include "marrtino_core/obstacles/detector.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <unordered_map>
#include <vector>

namespace marrtino::obstacles{

    namespace {

        constexpr double kPi = 3.14159265358979323846;

        struct Key3{
            std::int64_t x{};
            std::int64_t y{};
            std::int64_t z{};
            bool operator==(const Key3& other) const{
                return x == other.x && y == other.y && z == other.z;
            }
        };

        struct Key3Hash{
            std::size_t operator()(const Key3& k) const{
                std::size_t h = static_cast<std::size_t>(k.x) * 73856093u;
                h ^= static_cast<std::size_t>(k.y) * 19349663u;
                h ^= static_cast<std::size_t>(k.z) * 83492791u;
                return h;
            }
        };

        struct Key2{
            std::int64_t x{};
            std::int64_t y{};
            bool operator==(const Key2& other) const{
                return x == other.x && y == other.y;
            }
        };

        struct Key2Hash{
            std::size_t operator()(const Key2& k) const{
                std::size_t h = static_cast<std::size_t>(k.x) * 73856093u;
                h ^= static_cast<std::size_t>(k.y) * 19349663u;
                return h;
            }
        };

        struct Candidate{
            double x{};
            double y{};
            double range{};
        };

        // Unity locale (+z avanti, +x destra, +y alto) -> ROS (+x avanti, +y sinistra, +z alto):
        // x_unity = -y_ros, y_unity = z_ros, z_unity = x_ros.
        bool isChassisHit(const std::array<double,3>& p, const DetectorParams& params){
            return std::abs(p[1]) <= params.chassis_half_width + params.chassis_margin
                && p[0] <= params.chassis_front + params.chassis_margin
                && p[0] >= -(params.chassis_rear + params.chassis_margin)
                && p[2] <= params.chassis_top_rel_laser + params.chassis_margin;
        }

        std::int64_t binIndex(double value, double size){
            return static_cast<std::int64_t>(std::floor(value / size));
        }

        std::vector<std::array<double,3>> voxelDownsample(const std::vector<std::array<double,3>>& points, double size){
            std::vector<std::array<double,3>> centroids;
            if(size <= 0.0){
                centroids = points;
                return centroids;
            }

            std::unordered_map<Key3, std::size_t, Key3Hash> lookup;
            std::vector<std::array<double,3>> sums;
            std::vector<int> counts;

            for(const std::array<double,3>& p : points){
                Key3 key{binIndex(p[0], size), binIndex(p[1], size), binIndex(p[2], size)};
                auto it = lookup.find(key);
                if(it == lookup.end()){
                    lookup.emplace(key, sums.size());
                    sums.push_back(p);
                    counts.push_back(1);
                }else{
                    std::array<double,3>& sum = sums[it->second];
                    sum[0] += p[0];
                    sum[1] += p[1];
                    sum[2] += p[2];
                    counts[it->second] += 1;
                }
            }

            centroids.reserve(sums.size());
            for(std::size_t i = 0; i < sums.size(); ++i){
                double n = static_cast<double>(counts[i]);
                centroids.push_back({sums[i][0] / n, sums[i][1] / n, sums[i][2] / n});
            }
            return centroids;
        }

        double getSpread(const std::vector<Candidate>& points){
            if(points.empty()){
                return 0.0;
            }

            double minX = points[0].x;
            double maxX = points[0].x;
            double minY = points[0].y;
            double maxY = points[0].y;

            for(const Candidate& p : points){
                minX = std::min(minX, p.x);
                maxX = std::max(maxX, p.x);
                minY = std::min(minY, p.y);
                maxY = std::max(maxY, p.y);
            }
            return std::max(maxX - minX, maxY - minY);
        }

        Detection clusterToCircle(const std::vector<Candidate>& cluster, double margin){
            double sx = 0.0;
            double sy = 0.0;
            for(const Candidate& p : cluster){
                sx += p.x;
                sy += p.y;
            }

            double n = static_cast<double>(cluster.size());
            double cx = sx / n;
            double cy = sy / n;

            double maxSqDist = 0.0;
            for(const Candidate& p : cluster){
                double dx = p.x - cx;
                double dy = p.y - cy;
                maxSqDist = std::max(maxSqDist, dx*dx + dy*dy);
            }

            Detection d{};
            d.cx = cx;
            d.cy = cy;
            d.r = std::sqrt(maxSqDist) + margin;
            d.points = static_cast<int>(cluster.size());
            return d;
        }

    } // namespace

    ObstacleDetector::ObstacleDetector(DetectorParams params) : params_(params){}

    const ObstacleDetector::Diagnostics& ObstacleDetector::diagnostics() const{
        return diagnostics_;
    }

    Detections ObstacleDetector::detect(const std::vector<std::array<double,3>>& points,
                                        const trajectory::Pose2D& laser_pose,
                                        const grid::Grid<float>& edt){

        Detections result;
        diagnostics_ = Diagnostics{};
        diagnostics_.input = static_cast<int>(points.size());

        std::vector<std::array<double,3>> downsampled = voxelDownsample(points, params_.detection_voxel_size);
        diagnostics_.after_voxel = static_cast<int>(downsampled.size());

        const double maxElevationRad = params_.max_elevation_deg * kPi / 180.0;
        const double cosTheta = std::cos(laser_pose.theta);
        const double sinTheta = std::sin(laser_pose.theta);

        std::vector<Candidate> candidates;
        candidates.reserve(downsampled.size());

        for(const std::array<double,3>& p : downsampled){

            // /point_cloud di Unity contiene gia' coordinate MONDO (LiDAR3D somma
            // l'origine del laser al raggio), non offset dal sensore. Si ricava quindi
            // il punto locale invertendo la posa, invece di applicarla.
            double mx;
            double my;
            double mz;
            double lx;
            double ly;

            if(params_.points_in_map_frame){
                mx = p[0];
                my = p[1];
                mz = p[2];
                const double dx = mx - laser_pose.x;
                const double dy = my - laser_pose.y;
                lx =  cosTheta*dx + sinTheta*dy;
                ly = -sinTheta*dx + cosTheta*dy;
            }else{
                lx = p[0];
                ly = p[1];
                mx = laser_pose.x + cosTheta*p[0] - sinTheta*p[1];
                my = laser_pose.y + sinTheta*p[0] + cosTheta*p[1];
                mz = params_.laser_height + p[2];
            }

            // Quota ASSOLUTA sul pavimento: fuori soffitti e pavimento.
            if(mz <= params_.obstacle_z_min || mz >= params_.obstacle_z_max){
                continue;
            }
            diagnostics_.in_band += 1;

            const double range = std::sqrt((mx - laser_pose.x)*(mx - laser_pose.x) +
                                           (my - laser_pose.y)*(my - laser_pose.y));

            // La sagoma della scocca e' definita nel frame del laser.
            const std::array<double,3> local{lx, ly, mz - params_.laser_height};
            if(params_.use_chassis_footprint ? isChassisHit(local, params_) : range < params_.self_hit_radius){
                diagnostics_.rejected_self_points += 1;
                continue;
            }

            if(range > params_.max_detection_range){
                continue;
            }
            diagnostics_.in_range += 1;

            // Solo i raggi verso l'ALTO sono soffitto: un gate simmetrico cancellerebbe
            // gli ostacoli piu' bassi del laser, che si vedono per forza con elevazione negativa.
            if(std::atan2(mz - params_.laser_height, range) > maxElevationRad){
                continue;
            }

            Candidate c{};
            c.x = mx;
            c.y = my;
            c.range = range;
            candidates.push_back(c);
        }

        const grid::GridGeometry& geometry = edt.geometry();
        const double resolution = geometry.resolution();

        std::vector<Candidate> unexplained;
        unexplained.reserve(candidates.size());

        for(const Candidate& c : candidates){
            grid::Cell cell = geometry.worldToCell(grid::Point2{c.x, c.y});
            if(!geometry.contains(cell)){
                continue;
            }

            double clearance = static_cast<double>(edt(cell)) * resolution;
            double tolAtRange = params_.obs_tol + params_.obs_tol_per_meter * c.range;
            if(clearance > tolAtRange){
                unexplained.push_back(c);
            }
        }
        diagnostics_.unexplained = static_cast<int>(unexplained.size());

        if(!candidates.empty()
           && static_cast<double>(unexplained.size()) / static_cast<double>(candidates.size()) > params_.max_unexplained_fraction
           && getSpread(unexplained) > params_.min_suspicious_spread){
            diagnostics_.pose_suspicious = true;
            return result;
        }

        if(unexplained.empty()){
            return result;
        }

        const double clusteringRadius = params_.clustering_radius > 0.0 ? params_.clustering_radius : 1e-3;

        std::unordered_map<Key2, std::size_t, Key2Hash> lookup;
        std::vector<Key2> keys;
        std::vector<std::vector<Candidate>> buckets;

        for(const Candidate& c : unexplained){
            Key2 key{binIndex(c.x, clusteringRadius), binIndex(c.y, clusteringRadius)};
            auto it = lookup.find(key);
            if(it == lookup.end()){
                lookup.emplace(key, buckets.size());
                keys.push_back(key);
                buckets.push_back({c});
            }else{
                buckets[it->second].push_back(c);
            }
        }

        std::vector<bool> visited(buckets.size(), false);

        for(std::size_t seed = 0; seed < buckets.size(); ++seed){
            if(visited[seed]){
                continue;
            }

            std::vector<Candidate> cluster;
            std::deque<std::size_t> frontier;
            frontier.push_back(seed);
            visited[seed] = true;

            while(!frontier.empty()){
                std::size_t current = frontier.front();
                frontier.pop_front();
                cluster.insert(cluster.end(), buckets[current].begin(), buckets[current].end());

                for(int dx = -1; dx <= 1; ++dx){
                    for(int dy = -1; dy <= 1; ++dy){
                        if(dx == 0 && dy == 0){
                            continue;
                        }

                        Key2 neighbor{keys[current].x + dx, keys[current].y + dy};
                        auto it = lookup.find(neighbor);
                        if(it != lookup.end() && !visited[it->second]){
                            visited[it->second] = true;
                            frontier.push_back(it->second);
                        }
                    }
                }
            }

            diagnostics_.clustered += 1;

            Detection circle = clusterToCircle(cluster, params_.cluster_margin);

            double dx = circle.cx - laser_pose.x;
            double dy = circle.cy - laser_pose.y;
            double sqDistToLaser = dx*dx + dy*dy;

            if(sqDistToLaser < circle.r * circle.r){
                diagnostics_.rejected_self += 1;
                continue;
            }

            int required = std::sqrt(sqDistToLaser) < params_.near_cluster_range
                           ? params_.min_cluster_points_near
                           : params_.min_cluster_points;

            if(circle.points < required){
                continue;
            }

            result.push_back(circle);
        }

        return result;
    }

} // namespace marrtino::obstacles
