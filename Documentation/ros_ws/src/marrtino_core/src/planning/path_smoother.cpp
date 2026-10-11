#include "marrtino_core/planning/path_smoother.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace marrtino::planning{

    namespace {

        constexpr double kRadToDeg = 57.29577951308232;

        bool lineOfSightClear(const grid::Grid<float>& edt, grid::Point2 a, grid::Point2 b, float tunnel_half_width){
            const double resolution = edt.geometry().resolution();

            const double dx = b.x - a.x;
            const double dy = b.y - a.y;
            const double length = std::sqrt(dx * dx + dy * dy);

            const int samples = std::max(1, static_cast<int>(std::ceil(length / (0.5 * resolution))));

            for(int s = 0; s <= samples; ++s){
                const double u = static_cast<double>(s) / static_cast<double>(samples);
                const grid::Point2 p{a.x + u * dx, a.y + u * dy};

                const grid::Cell c = edt.geometry().worldToCell(p);
                if(!edt.geometry().contains(c)){
                    return false;
                }
                if(edt(c) * static_cast<float>(resolution) < tunnel_half_width){
                    return false;
                }
            }
            return true;
        }

        trajectory::Path stringPull(const grid::Grid<float>& edt, const trajectory::Path& points, float tunnel_half_width){
            const std::size_t n = points.size();
            if(n <= 2){
                return points;
            }

            trajectory::Path kept;
            kept.push_back(points[0]);

            std::size_t anchor = 0;
            for(std::size_t j = 2; j < n; ++j){
                if(!lineOfSightClear(edt, points[anchor], points[j], tunnel_half_width)){
                    kept.push_back(points[j - 1]);
                    anchor = j - 1;
                }
            }

            kept.push_back(points[n - 1]);
            return kept;
        }

        trajectory::Path simplifyNearCollinearVertices(const grid::Grid<float>& edt, const trajectory::Path& points,
                                                        float collinear_tolerance_deg, float tunnel_half_width){
            if(points.size() < 3){
                return points;
            }

            trajectory::Path kept;
            kept.push_back(points[0]);

            for(std::size_t i = 1; i + 1 < points.size(); ++i){
                const grid::Point2 previous = kept.back();
                const grid::Point2 current = points[i];
                const grid::Point2 next = points[i + 1];

                const double a1 = std::atan2(current.y - previous.y, current.x - previous.x);
                const double a2 = std::atan2(next.y - current.y, next.x - current.x);
                const double turn = std::abs(std::atan2(std::sin(a2 - a1), std::cos(a2 - a1))) * kRadToDeg;

                if(turn < collinear_tolerance_deg && lineOfSightClear(edt, previous, next, tunnel_half_width)){
                    continue;
                }
                kept.push_back(current);
            }

            kept.push_back(points.back());
            return kept;
        }

        trajectory::Path removeTooCloseConsecutivePoints(const grid::Grid<float>& edt, const trajectory::Path& points,
                                                            float min_vertex_spacing){
            if(points.empty()){
                return points;
            }

            const double min_spacing = std::max(static_cast<double>(min_vertex_spacing), 0.5 * edt.geometry().resolution());

            trajectory::Path cleaned;
            for(const grid::Point2& p : points){
                if(!cleaned.empty()){
                    const double dx = p.x - cleaned.back().x;
                    const double dy = p.y - cleaned.back().y;
                    if(std::sqrt(dx * dx + dy * dy) < min_spacing){
                        continue;
                    }
                }
                cleaned.push_back(p);
            }

            const grid::Point2 goal = points.back();
            if(!cleaned.empty() && (cleaned.back().x != goal.x || cleaned.back().y != goal.y)){
                cleaned.back() = goal;
            }

            return cleaned;
        }

    } // namespace

    trajectory::Path smoothPath(const grid::Grid<float>& edt,
                                const std::vector<grid::Cell>& path,
                                const SmootherParams& params){
        trajectory::Path world;
        world.reserve(path.size());
        for(const grid::Cell& c : path){
            world.push_back(edt.geometry().cellToWorld(c));
        }

        if(world.size() <= 2){
            return world;
        }

        trajectory::Path result = stringPull(edt, world, params.tunnel_half_width);
        result = simplifyNearCollinearVertices(edt, result, params.collinear_tolerance_deg, params.tunnel_half_width);
        result = removeTooCloseConsecutivePoints(edt, result, params.min_vertex_spacing);
        return result;
    }

} // namespace marrtino::planning
