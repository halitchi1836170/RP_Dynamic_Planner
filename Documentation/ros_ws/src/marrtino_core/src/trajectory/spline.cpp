#include "marrtino_core/trajectory/spline.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include "marrtino_core/math/tridiagonal_solver.hpp"

namespace marrtino::trajectory{

    namespace {

        constexpr double kMinInterval = 1e-9;

        struct Segment{
            double dt{};
            double a{};
            double b{};
            double c{};
            double d{};
        };

        Path mergeZeroLengthIntervals(const Path& vertices){
            Path merged;
            for(const grid::Point2& p : vertices){
                if(!merged.empty()){
                    const double dx = p.x - merged.back().x;
                    const double dy = p.y - merged.back().y;
                    if(std::sqrt(dx * dx + dy * dy) < kMinInterval){
                        continue;
                    }
                }
                merged.push_back(p);
            }
            return merged;
        }

        std::vector<double> cumulativeChordLength(const Path& vertices){
            std::vector<double> t(vertices.size(), 0.0);
            for(std::size_t i = 1; i < vertices.size(); ++i){
                const double dx = vertices[i].x - vertices[i - 1].x;
                const double dy = vertices[i].y - vertices[i - 1].y;
                t[i] = t[i - 1] + std::sqrt(dx * dx + dy * dy);
            }
            return t;
        }

        // I due nodi virtuali (uno dopo il primo nodo, uno prima dell'ultimo) sono i gradi di liberta'
        // in piu' che permettono di imporre velocita' e accelerazione agli estremi: vengono messi a
        // metta' del primo e dell'ultimo intervallo, e la loro posizione e' l'incognita nota che
        // completeQVectorTerms finisce di calcolare dopo la soluzione del sistema.
        std::vector<double> augmentWithVirtualKnots(const std::vector<double>& t){
            const std::size_t n = t.size();
            std::vector<double> augmented;
            augmented.reserve(n + 2);

            augmented.push_back(t[0]);
            augmented.push_back(0.5 * (t[0] + t[1]));
            for(std::size_t i = 1; i + 1 < n; ++i){
                augmented.push_back(t[i]);
            }
            augmented.push_back(0.5 * (t[n - 2] + t[n - 1]));
            augmented.push_back(t[n - 1]);
            return augmented;
        }

        std::vector<double> intervalsOf(const std::vector<double>& t){
            std::vector<double> dt(t.size() - 1, 0.0);
            for(std::size_t i = 0; i + 1 < t.size(); ++i){
                dt[i] = t[i + 1] - t[i];
            }
            return dt;
        }

        void buildABCVectors(const std::vector<double>& dt, std::vector<double>& a, std::vector<double>& b, std::vector<double>& c){
            const std::size_t n = dt.size() - 1;

            a.assign(n, 0.0);
            b.assign(n, 0.0);
            c.assign(n, 0.0);

            b[0] = (dt[0] + dt[1]) * (2.0 + dt[0] / dt[1]);
            c[0] = dt[1];

            a[1] = dt[1] - dt[0] * dt[0] / dt[1];
            b[1] = 2.0 * (dt[1] + dt[2]);
            c[1] = dt[2];

            for(std::size_t k = 2; k + 3 <= n; ++k){
                a[k] = dt[k];
                b[k] = 2.0 * (dt[k] + dt[k + 1]);
                c[k] = dt[k + 1];
            }

            a[n - 2] = dt[n - 2];
            b[n - 2] = 2.0 * (dt[n - 2] + dt[n - 1]);
            c[n - 2] = dt[n - 1] - dt[n] * dt[n] / dt[n - 1];

            a[n - 1] = dt[n - 1];
            b[n - 1] = (dt[n - 1] + dt[n]) * (2.0 + dt[n] / dt[n - 1]);
        }

        std::vector<double> incompleteQVector(const std::vector<double>& q, double virtual_start, double virtual_final){
            const std::size_t n = q.size();

            std::vector<double> result(n + 2, 0.0);
            result[0] = q[0];
            result[1] = virtual_start;
            for(std::size_t j = 2; j <= n - 1; ++j){
                result[j] = q[j - 1];
            }
            result[n] = virtual_final;
            result[n + 1] = q[n - 1];
            return result;
        }

        std::vector<double> buildDVector(const std::vector<double>& q, const std::vector<double>& dt,
                                            double initial_acc, double final_acc){
            const std::size_t n = q.size() - 2;

            std::vector<double> d(n, 0.0);
            d[0] = 6.0 * q[0] / dt[0] + 6.0 * q[2] / dt[1] - 6.0 * q[1] * (dt[0] + dt[1]) / (dt[0] * dt[1]) - dt[0] * initial_acc;
            d[1] = 6.0 * (q[1] - q[2]) / dt[1] + 6.0 * (q[3] - q[2]) / dt[2];

            for(std::size_t k = 2; k + 3 <= n; ++k){
                d[k] = 6.0 * (q[k] - q[k + 1]) / dt[k] + 6.0 * (q[k + 2] - q[k + 1]) / dt[k + 1];
            }

            d[n - 2] = 6.0 * (q[n - 2] - q[n - 1]) / dt[n - 2] + 6.0 * (q[n] - q[n - 1]) / dt[n - 1];
            d[n - 1] = 6.0 * q[n - 1] / dt[n - 1] + 6.0 * q[n + 1] / dt[n]
                        - 6.0 * q[n] * (dt[n - 1] + dt[n]) / (dt[n - 1] * dt[n]) - dt[n] * final_acc;
            return d;
        }

        std::vector<double> augmentAccelerations(const std::vector<double>& acc, double initial_acc, double final_acc){
            std::vector<double> result(acc.size() + 2, 0.0);
            result.front() = initial_acc;
            result.back() = final_acc;
            for(std::size_t i = 0; i < acc.size(); ++i){
                result[i + 1] = acc[i];
            }
            return result;
        }

        void completeQVectorTerms(std::vector<double>& q, const std::vector<double>& dt, const std::vector<double>& acc){
            const std::size_t m = q.size();
            q[1] += (dt[0] * dt[0] / 6.0) * acc[1];
            q[m - 2] += (dt[m - 2] * dt[m - 2] / 6.0) * acc[m - 2];
        }

        std::vector<Segment> splineSegments(const std::vector<double>& q, const std::vector<double>& acc, const std::vector<double>& dt){
            std::vector<Segment> segments;
            segments.reserve(dt.size());

            for(std::size_t k = 0; k < dt.size(); ++k){
                const double delta = q[k + 1] - q[k];
                const double delta_acc = acc[k + 1] - acc[k];

                Segment s;
                s.dt = dt[k];
                s.d = q[k];
                s.c = (delta / dt[k]) - (dt[k] / 6.0) * (delta_acc + 3.0 * acc[k]);
                s.b = acc[k] / 2.0;
                s.a = delta_acc / (6.0 * dt[k]);
                segments.push_back(s);
            }
            return segments;
        }

        double evaluate(const Segment& s, double tau){
            return s.a * tau * tau * tau + s.b * tau * tau + s.c * tau + s.d;
        }

        std::vector<Segment> solveCoordinate(const std::vector<double>& values, const std::vector<double>& dt,
                                                const std::vector<double>& a, const std::vector<double>& b,
                                                const std::vector<double>& c){
            const std::size_t n = values.size();
            const std::size_t last = dt.size() - 1;

            const double initial_vel = 0.0;
            const double final_vel = 0.0;
            const double initial_acc = 0.0;
            const double final_acc = 0.0;

            const double virtual_start = values[0] + dt[0] * initial_vel + (dt[0] * dt[0]) / 3.0 * initial_acc;
            const double virtual_final = values[n - 1] - dt[last] * final_vel + (dt[last] * dt[last]) / 3.0 * final_acc;

            std::vector<double> q = incompleteQVector(values, virtual_start, virtual_final);
            const std::vector<double> d = buildDVector(q, dt, initial_acc, final_acc);
            const std::vector<double> acc = augmentAccelerations(math::SolveTridiagonal(a, b, c, d), initial_acc, final_acc);

            completeQVectorTerms(q, dt, acc);
            return splineSegments(q, acc, dt);
        }

        Path sampleStraightSegment(const Path& vertices, int samples){
            Path out;
            out.reserve(static_cast<std::size_t>(samples) + 1);
            for(int s = 0; s < samples; ++s){
                const double u = static_cast<double>(s) / static_cast<double>(samples);
                out.push_back(grid::Point2{vertices[0].x + u * (vertices[1].x - vertices[0].x),
                                            vertices[0].y + u * (vertices[1].y - vertices[0].y)});
            }
            out.push_back(vertices[1]);
            return out;
        }

        bool isFinitePath(const Path& path){
            for(const grid::Point2& p : path){
                if(!std::isfinite(p.x) || !std::isfinite(p.y)){
                    return false;
                }
            }
            return true;
        }

    } // namespace

    Path buildSpline(const Path& vertices, const SplineParams& params){
        if(vertices.size() < 2){
            return vertices;
        }

        const Path knots = mergeZeroLengthIntervals(vertices);
        if(knots.size() < 2){
            return vertices;
        }

        const int samples = std::max(1, params.samples_per_segment);
        if(knots.size() == 2){
            return sampleStraightSegment(knots, samples);
        }

        const std::vector<double> dt = intervalsOf(augmentWithVirtualKnots(cumulativeChordLength(knots)));
        for(const double& h : dt){
            if(h < kMinInterval){
                return knots;
            }
        }

        std::vector<double> xs(knots.size(), 0.0);
        std::vector<double> ys(knots.size(), 0.0);
        for(std::size_t i = 0; i < knots.size(); ++i){
            xs[i] = knots[i].x;
            ys[i] = knots[i].y;
        }

        std::vector<double> a;
        std::vector<double> b;
        std::vector<double> c;
        buildABCVectors(dt, a, b, c);

        const std::vector<Segment> x_spline = solveCoordinate(xs, dt, a, b, c);
        const std::vector<Segment> y_spline = solveCoordinate(ys, dt, a, b, c);

        Path out;
        out.reserve(x_spline.size() * static_cast<std::size_t>(samples) + 1);
        for(std::size_t k = 0; k < x_spline.size(); ++k){
            for(int s = 0; s < samples; ++s){
                const double tau = (static_cast<double>(s) / static_cast<double>(samples)) * x_spline[k].dt;
                out.push_back(grid::Point2{evaluate(x_spline[k], tau), evaluate(y_spline[k], tau)});
            }
        }
        out.push_back(knots.back());

        if(!isFinitePath(out)){
            return knots;
        }
        return out;
    }

} // namespace marrtino::trajectory
