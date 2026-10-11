#include "marrtino_core/trajectory/velocity_profile.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace marrtino::trajectory{

    namespace {

        Path dropDuplicates(const Path& path){
            Path out;
            out.reserve(path.size());
            for(const grid::Point2& p : path){
                if(out.empty()){
                    out.push_back(p);
                    continue;
                }
                double dx = p.x - out.back().x;
                double dy = p.y - out.back().y;
                if(dx * dx + dy * dy > 1e-18){
                    out.push_back(p);
                }
            }
            return out;
        }

        double circumscribedCurvature(const grid::Point2& p0, const grid::Point2& p1, const grid::Point2& p2){
            double ax = p1.x - p0.x;
            double ay = p1.y - p0.y;
            double bx = p2.x - p1.x;
            double by = p2.y - p1.y;
            double cx = p2.x - p0.x;
            double cy = p2.y - p0.y;

            double la = std::sqrt(ax * ax + ay * ay);
            double lb = std::sqrt(bx * bx + by * by);
            double lc = std::sqrt(cx * cx + cy * cy);
            double den = la * lb * lc;

            if(den < 1e-12){
                return 0.0;
            }
            double kappa = 2.0 * (ax * by - ay * bx) / den;
            return std::isfinite(kappa) ? kappa : 0.0;
        }

        double wrapToPi(double a){
            return std::atan2(std::sin(a), std::cos(a));
        }

        // Ricampiona la spezzata a passo d'arco costante. Senza, i campioni della
        // spline sono spaziati da 1e-5 a 0.12 m e le differenze finite che ne
        // ricavano theta e curvatura diventano inutilizzabili.
        Path resampleUniform(const Path& pts, double step){
            Path out;
            if(pts.size() < 2 || !(step > 1e-9)){
                return pts;
            }

            out.push_back(pts.front());
            double carry = 0.0;

            for(std::size_t i = 1; i < pts.size(); ++i){
                double dx = pts[i].x - pts[i - 1].x;
                double dy = pts[i].y - pts[i - 1].y;
                double seg = std::sqrt(dx * dx + dy * dy);
                if(seg < 1e-12){
                    continue;
                }

                double pos = step - carry;
                while(pos <= seg){
                    const double a = pos / seg;
                    out.push_back(grid::Point2{pts[i - 1].x + a * dx, pts[i - 1].y + a * dy});
                    pos += step;
                }
                carry = seg - (pos - step);
            }

            if(out.size() < 2){
                return pts;
            }
            out.push_back(pts.back());
            return out;
        }

    } // namespace

    Trajectory buildTrajectory(const Path& path, const ProfileParams& params){
        Trajectory trajectory;

        Path pts = resampleUniform(dropDuplicates(path), params.geometry_step);
        pts = dropDuplicates(pts);
        if(pts.empty()){
            return trajectory;
        }

        const std::size_t n = pts.size();
        if(n < 2){
            trajectory.push_back(TrajectorySample{0.0, pts[0].x, pts[0].y, 0.0, 0.0, 0.0, 0.0});
            return trajectory;
        }

        std::vector<double> arc(n, 0.0);
        std::vector<double> theta(n, 0.0);
        std::vector<double> kappa(n, 0.0);

        for(std::size_t i = 1; i < n; ++i){
            double dx = pts[i].x - pts[i - 1].x;
            double dy = pts[i].y - pts[i - 1].y;
            arc[i] = arc[i - 1] + std::sqrt(dx * dx + dy * dy);
            theta[i - 1] = std::atan2(dy, dx);
        }
        theta[n - 1] = theta[n - 2];

        for(std::size_t i = 1; i + 1 < n; ++i){
            kappa[i] = circumscribedCurvature(pts[i - 1], pts[i], pts[i + 1]);
        }

        const double length = arc[n - 1];
        const double cruise = std::min(params.cruise_speed, params.v_max);

        std::vector<double> profile(n, cruise);
        for(std::size_t i = 0; i < n; ++i){
            double k = std::abs(kappa[i]);
            profile[i] = k > 1e-4 ? std::min(cruise, params.omega_max / k) : cruise;
        }

        profile[0] = 0.0;
        for(std::size_t i = 1; i < n; ++i){
            double ds = arc[i] - arc[i - 1];
            profile[i] = std::min(profile[i], std::sqrt(profile[i - 1] * profile[i - 1] + 2.0 * params.a_max * ds));
        }

        profile[n - 1] = 0.0;
        for(std::size_t i = n - 1; i-- > 0;){
            double ds = arc[i + 1] - arc[i];
            profile[i] = std::min(profile[i], std::sqrt(profile[i + 1] * profile[i + 1] + 2.0 * params.a_max * ds));
        }

        for(std::size_t i = 0; i < n; ++i){
            profile[i] = std::max(profile[i], params.v_min);
        }

        const double dt = params.control_period > 1e-9 ? params.control_period : 0.05;
        double s = 0.0;
        double t = 0.0;
        std::size_t j = 0;

        while(s < length){
            while(j + 2 < n && arc[j + 1] < s){
                ++j;
            }
            double seg = arc[j + 1] - arc[j];
            double a = seg > 1e-9 ? (s - arc[j]) / seg : 0.0;

            double x = pts[j].x + a * (pts[j + 1].x - pts[j].x);
            double y = pts[j].y + a * (pts[j + 1].y - pts[j].y);
            double v = profile[j] + a * (profile[j + 1] - profile[j]);

            // theta e kappa vanno INTERPOLATI come x, y e v: prenderli dal campione
            // piu' vicino li fa avanzare a gradini, e un salto di assetto diventa
            // uno strappo di omega attraverso il termine -k3*e3 del controllore.
            const double th = theta[j] + a * wrapToPi(theta[j + 1] - theta[j]);
            const double kp = kappa[j] + a * (kappa[j + 1] - kappa[j]);

            trajectory.push_back(TrajectorySample{t, x, y, wrapToPi(th), v, kp * v, kp});

            double step = v * dt;
            if(!(step > 1e-12)){
                break;
            }
            s += step;
            t += dt;
        }

        trajectory.push_back(TrajectorySample{t, pts[n - 1].x, pts[n - 1].y, theta[n - 1], 0.0, 0.0, 0.0});
        return trajectory;
    }

} // namespace marrtino::trajectory
