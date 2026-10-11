#include "marrtino_core/control/cbf_filter.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

#include <qpOASES.hpp>

namespace marrtino::control{

    namespace {

        constexpr int kVariables = 4;

        struct Row{
            std::array<double, kVariables> a{};
            double lower{};
        };

        struct FieldSample{
            double d{};
            double gx{};
            double gy{};
            bool valid{};
        };

        std::pair<double, bool> distanceAt(const grid::Grid<float>& edt, double x, double y){
            grid::Cell cell = edt.geometry().worldToCell(grid::Point2{x, y});
            if(!edt.geometry().contains(cell)){
                return {0.0, false};
            }
            return {static_cast<double>(edt(cell)) * edt.geometry().resolution(), true};
        }

        // Sotto questa norma del gradiente si e' sulla cresta del campo: direzione
        // priva di significato. 0.5 e' meta' del valore eikonale corretto.
        constexpr double kEikonalMinNorm = 0.5;

        FieldSample sampleDistanceField(const grid::Grid<float>& edt, double x, double y, double gradient_step_cells){
            const double step = gradient_step_cells * edt.geometry().resolution();

            std::pair<double, bool> center = distanceAt(edt, x, y);
            std::pair<double, bool> x_plus = distanceAt(edt, x + step, y);
            std::pair<double, bool> x_minus = distanceAt(edt, x - step, y);
            std::pair<double, bool> y_plus = distanceAt(edt, x, y + step);
            std::pair<double, bool> y_minus = distanceAt(edt, x, y - step);

            if(!center.second || !x_plus.second || !x_minus.second || !y_plus.second || !y_minus.second){
                return FieldSample{};
            }
            if(step <= 0.0){
                return FieldSample{};
            }

            FieldSample sample;
            sample.d = center.first;
            double gx = (x_plus.first - x_minus.first) / (2.0 * step);
            double gy = (y_plus.first - y_minus.first) / (2.0 * step);

            // Una distance transform soddisfa l'equazione eikonale: |grad d| = 1
            // ovunque TRANNE sull'asse mediale, dove il campo ha una piega. In una
            // porta il robot sta proprio li', a meta' fra i due stipiti: le
            // differenze centrate scavalcano la cresta, i due lati si annullano e
            // |grad d| collassa con direzione instabile. Imporre un vincolo su una
            // direzione che si ribalta a ogni passo fa oscillare il QP.
            //
            // Quindi: si normalizza (la norma corretta e' 1, non quella misurata) e
            // se la norma e' troppo lontana da 1 il campione si scarta. Non e' una
            // perdita di sicurezza: sull'asse mediale si e' nel punto PIU' LONTANO
            // dagli ostacoli, e la riga sul corpo resta comunque attiva.
            const double norm = std::sqrt(gx * gx + gy * gy);
            if(!(norm > kEikonalMinNorm)){
                return FieldSample{};
            }
            gx /= norm;
            gy /= norm;

            sample.gx = gx;
            sample.gy = gy;
            sample.valid = true;
            return sample;
        }

    } // namespace

    CBFFilter::CBFFilter(CBFParams params) : params_(params){
        reset();
    }

    void CBFFilter::setObstacles(obstacles::Tracks tracks){
        tracks_ = std::move(tracks);
    }

    void CBFFilter::setStaticMap(const grid::Grid<float>* edt){
        edt_ = edt;
    }

    void CBFFilter::reset(){
        last_solution_ = trajectory::Command{};
        diagnostics_ = CBFDiagnostics{};
        diagnostics_.min_barrier = std::numeric_limits<double>::infinity();
        diagnostics_.min_margin = std::numeric_limits<double>::infinity();
        diagnostics_.static_margin = std::numeric_limits<double>::infinity();
    }

    const CBFDiagnostics& CBFFilter::diagnostics() const{
        return diagnostics_;
    }

    const CBFParams& CBFFilter::params() const{
        return params_;
    }

    trajectory::Command CBFFilter::filter(const trajectory::Command& u_nom,
                                          const trajectory::Pose2D& pose,
                                          double e1, double e2, double e3, double k2,
                                          double v_des, double omega_des){

        const double cos_t = std::cos(pose.theta);
        const double sin_t = std::sin(pose.theta);
        const double b_la = params_.b_look_ahead;

        const double pbx = pose.x + b_la * cos_t;
        const double pby = pose.y + b_la * sin_t;

        diagnostics_.min_barrier = std::numeric_limits<double>::infinity();
        diagnostics_.min_margin = std::numeric_limits<double>::infinity();
        diagnostics_.static_margin = std::numeric_limits<double>::infinity();
        diagnostics_.feasible = true;
        diagnostics_.active = false;
        diagnostics_.slack = 0.0;
        diagnostics_.deviation = 0.0;

        std::vector<Row> rows;

        const double V = 0.5 * k2 * (e1 * e1 + e2 * e2) + 0.5 * e3 * e3;
        const double a0 = k2 * v_des * (e1 * std::cos(e3) + e2 * std::sin(e3)) + e3 * omega_des;
        const double nominal_decay = k2 * e1 * u_nom.v + e3 * u_nom.omega;
        rows.push_back(Row{{k2 * e1, e3, 1.0, 0.0}, std::min(params_.gamma_clf * V + a0, nominal_decay)});

        const std::size_t rows_before_barriers = rows.size();

        for(const obstacles::Track& track : tracks_){
            const double dx = pbx - track.cx;
            const double dy = pby - track.cy;
            const double R = track.r + params_.r_safe_dynamic;
            const double margin = std::sqrt(dx * dx + dy * dy) - R;
            if(margin > params_.obstacle_activation_range){
                continue;
            }

            const double h = dx * dx + dy * dy - R * R;
            diagnostics_.min_barrier = std::min(diagnostics_.min_barrier, h);
            diagnostics_.min_margin = std::min(diagnostics_.min_margin, margin);

            const double av = 2.0 * (dx * cos_t + dy * sin_t);
            const double aw = 2.0 * b_la * (-dx * sin_t + dy * cos_t);
            rows.push_back(Row{{av, aw, 0.0, 1.0}, -params_.alpha_dynamic * h + 2.0 * (dx * track.vx + dy * track.vy)});

            if(params_.robot_body_radius <= 0.0){
                continue;
            }

            const double bx = pose.x - track.cx;
            const double by = pose.y - track.cy;
            const double r_body = track.r + params_.robot_body_radius;
            const double h_body = bx * bx + by * by - r_body * r_body;
            diagnostics_.min_barrier = std::min(diagnostics_.min_barrier, h_body);

            rows.push_back(Row{{2.0 * (bx * cos_t + by * sin_t), 0.0, 0.0, 1.0},
                               -params_.alpha_dynamic * h_body + 2.0 * (bx * track.vx + by * track.vy)});
        }

        if(edt_ != nullptr){
            FieldSample field = sampleDistanceField(*edt_, pbx, pby, params_.gradient_step_cells);
            if(field.valid){
                const double h = field.d - params_.r_safe_static;
                diagnostics_.static_margin = h;
                if(h <= params_.static_activation_distance){
                    const double av = field.gx * cos_t + field.gy * sin_t;
                    const double aw = b_la * (-field.gx * sin_t + field.gy * cos_t);
                    rows.push_back(Row{{av, aw, 0.0, 1.0}, -params_.alpha_static * h});
                }
            }

            // Barriera statica sul CORPO, speculare a quella dinamica: e' QUESTA a
            // garantire il non contatto con la mappa. Senza, la garanzia deve cadere
            // tutta su r_safe_static, che va allora gonfiato fino a r_corpo + b_la e
            // rende impercorribili i passaggi stretti.
            // Niente dipendenza da omega: il centro del robot ha grado relativo zero
            // rispetto alla sterzata, esattamente come nel caso dinamico.
            if(params_.robot_body_radius > 0.0){
                FieldSample body = sampleDistanceField(*edt_, pose.x, pose.y, params_.gradient_step_cells);
                if(body.valid){
                    const double h_body = body.d - params_.robot_body_radius;
                    diagnostics_.static_margin = std::min(diagnostics_.static_margin, h_body);
                    if(h_body <= params_.static_activation_distance){
                        rows.push_back(Row{{body.gx * cos_t + body.gy * sin_t, 0.0, 0.0, 1.0},
                                           -params_.alpha_static * h_body});
                    }
                }
            }
        }

        const double smoothing = rows.size() > rows_before_barriers ? params_.command_smoothing_weight : 0.0;

        diagnostics_.constraints = static_cast<int>(rows.size());

        const int n_constraints = static_cast<int>(rows.size());

        std::array<qpOASES::real_t, kVariables * kVariables> H{};
        H[0] = params_.v_deviation_weight + smoothing;
        H[5] = params_.w_deviation_weight + smoothing;
        H[10] = params_.slack_penalty;
        H[15] = params_.cbf_slack_penalty;

        std::array<qpOASES::real_t, kVariables> g{};
        g[0] = -(params_.v_deviation_weight * u_nom.v + smoothing * last_solution_.v);
        g[1] = -(params_.w_deviation_weight * u_nom.omega + smoothing * last_solution_.omega);

        const qpOASES::real_t infinity = qpOASES::INFTY;

        std::array<qpOASES::real_t, kVariables> lb{-params_.v_max, -params_.w_max_cbf, 0.0, 0.0};
        std::array<qpOASES::real_t, kVariables> ub{params_.v_max, params_.w_max_cbf, infinity, infinity};

        std::vector<qpOASES::real_t> A(static_cast<std::size_t>(n_constraints) * kVariables, 0.0);
        std::vector<qpOASES::real_t> lbA(static_cast<std::size_t>(n_constraints), 0.0);
        std::vector<qpOASES::real_t> ubA(static_cast<std::size_t>(n_constraints), infinity);

        for(std::size_t i = 0; i < rows.size(); ++i){
            for(std::size_t j = 0; j < kVariables; ++j){
                const double value = rows[i].a[j];
                if(!std::isfinite(value)){
                    diagnostics_.feasible = false;
                    diagnostics_.slack = std::numeric_limits<double>::infinity();
                    return u_nom;
                }
                A[i * kVariables + j] = value;
            }
            if(!std::isfinite(rows[i].lower)){
                diagnostics_.feasible = false;
                diagnostics_.slack = std::numeric_limits<double>::infinity();
                return u_nom;
            }
            lbA[i] = rows[i].lower;
        }

        qpOASES::Options options;
        options.printLevel = qpOASES::PL_NONE;

        qpOASES::QProblem solver(kVariables, n_constraints);
        solver.setOptions(options);

        qpOASES::int_t working_set_recalculations = 200;
        qpOASES::returnValue status = solver.init(H.data(), g.data(), A.data(),
                                                 lb.data(), ub.data(),
                                                 lbA.data(), ubA.data(),
                                                 working_set_recalculations);

        if(status != qpOASES::SUCCESSFUL_RETURN || !solver.isSolved()){
            diagnostics_.feasible = false;
            diagnostics_.slack = std::numeric_limits<double>::infinity();
            return u_nom;
        }

        std::array<qpOASES::real_t, kVariables> solution{};
        if(solver.getPrimalSolution(solution.data()) != qpOASES::SUCCESSFUL_RETURN){
            diagnostics_.feasible = false;
            diagnostics_.slack = std::numeric_limits<double>::infinity();
            return u_nom;
        }

        const double v = static_cast<double>(solution[0]);
        const double omega = static_cast<double>(solution[1]);

        if(!std::isfinite(v) || !std::isfinite(omega)){
            diagnostics_.feasible = false;
            diagnostics_.slack = std::numeric_limits<double>::infinity();
            return u_nom;
        }

        trajectory::Command command{std::clamp(v, -params_.v_max, params_.v_max),
                                    std::clamp(omega, -params_.w_max_cbf, params_.w_max_cbf)};

        last_solution_ = command;

        const double dv = command.v - u_nom.v;
        const double dw = command.omega - u_nom.omega;
        diagnostics_.deviation = std::sqrt(dv * dv + dw * dw);
        diagnostics_.slack = std::isfinite(solution[3]) ? static_cast<double>(solution[3]) : std::numeric_limits<double>::infinity();
        diagnostics_.active = diagnostics_.deviation > params_.activation_tolerance;

        return command;
    }

} // namespace marrtino::control
