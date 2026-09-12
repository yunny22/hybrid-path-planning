#pragma once

#include <vector>
#include <cmath>
#include <algorithm>
#include <utility>
#include <carla_msgs/msg/carla_ego_vehicle_control.hpp>

struct Waypoint {
    double x;
    double y;
    double z;
    double yaw;
};

class PurePursuit {
public:
    double wheelbase = 2.88;

    void reset() {
        prev_steer_ = 0.0;
        speed_error_int_ = 0.0;
    }

    void calculate_control(
        double cur_v,
        double target_v,
        const std::vector<std::pair<double,double>>& local_path,
        carla_msgs::msg::CarlaEgoVehicleControl& msg,
        bool force_launch = false)
    {
        if (target_v <= 0.05 || local_path.size() < 2) {
            speed_error_int_ = 0.0;
            msg.throttle = 0.0f;
            msg.brake = 0.6f;
            msg.steer = 0.0f;
            return;
        }

        std::vector<std::pair<double,double>> path = sanitize_path(local_path);
        if (path.size() < 2) {
            msg.throttle = 0.0f;
            msg.brake = 0.5f;
            msg.steer = 0.0f;
            return;
        }

        const double preview_lat = preview_lateral(path, 16.0);
        const double preview_curv = preview_curvature(path, 14);
        double lookahead = compute_lookahead(cur_v, preview_lat, preview_curv);
        if (force_launch) lookahead = std::max(2.8, lookahead - 1.0);

        size_t tgt_idx = path.size() - 1;
        std::pair<double,double> target_pt = target_by_arc(path, lookahead, tgt_idx);
        const double target_heading = heading_at(path, tgt_idx);
        const double near_heading = heading_at(path, std::min<size_t>(5, path.size() - 1));
        const double cte = lateral_at_x(path, std::min(lookahead * 0.65, 3.8));

        const double alpha = std::atan2(target_pt.second, std::max(target_pt.first, 0.5));
        const double ld_eff = std::max(std::hypot(target_pt.first, target_pt.second), 1.8);
        const double pp_term = std::atan2(2.0 * wheelbase * std::sin(alpha), ld_eff);
        const double heading_term = 0.66 * target_heading + 0.34 * near_heading;
        const double cte_term = std::atan2(0.95 * cte, cur_v + 4.0);
        const double ff_term = std::clamp(estimate_curvature_ff(path, tgt_idx) * wheelbase, -0.14, 0.14);

        double raw_steer = 0.82 * pp_term + 0.26 * heading_term + 0.30 * cte_term + 0.18 * ff_term;
        if (force_launch) raw_steer *= 0.90;
        raw_steer = std::clamp(raw_steer, -0.82, 0.82);

        const double steer_rate_limit = force_launch ? 0.16 : 0.10;
        const double steer_delta = std::clamp(raw_steer - prev_steer_, -steer_rate_limit, steer_rate_limit);
        const double filtered_steer = 0.72 * prev_steer_ + 0.28 * (prev_steer_ + steer_delta);

        prev_steer_ = std::clamp(filtered_steer, -0.82, 0.82);
        msg.steer = static_cast<float>(-prev_steer_);

        double speed_cap = target_v;
        if (!force_launch) {
            if (std::abs(prev_steer_) > 0.50) {
                speed_cap = std::min(speed_cap, 18.0 / 3.6);
            } else if (std::abs(prev_steer_) > 0.35) {
                speed_cap = std::min(speed_cap, 24.0 / 3.6);
            }
            if (preview_curv > 0.075) {
                speed_cap = std::min(speed_cap, 23.0 / 3.6);
            }
        }

        const double speed_err = speed_cap - cur_v;
        speed_error_int_ = std::clamp(speed_error_int_ + speed_err * 0.03, -1.2, 1.2);
        double accel_cmd = 0.40 * speed_err + 0.08 * speed_error_int_;
        if (force_launch) accel_cmd += 0.35;

        if (accel_cmd >= 0.0) {
            msg.throttle = static_cast<float>(std::clamp(0.16 + 0.26 * accel_cmd, 0.0, 0.90));
            msg.brake = 0.0f;
            if (!force_launch && std::abs(prev_steer_) > 0.48) {
                msg.throttle = std::min(msg.throttle, 0.30f);
            }
            if (force_launch) {
                msg.throttle = std::max(msg.throttle, 0.30f);
            }
        } else {
            msg.throttle = 0.0f;
            msg.brake = static_cast<float>(std::clamp(-0.34 * accel_cmd, 0.0, 0.46));
            if (force_launch) {
                msg.brake = 0.0f;
                msg.throttle = 0.30f;
            }
        }
    }

private:
    double prev_steer_ = 0.0;
    double speed_error_int_ = 0.0;

    static std::vector<std::pair<double,double>> sanitize_path(
        const std::vector<std::pair<double,double>>& input)
    {
        std::vector<std::pair<double,double>> out;
        out.reserve(input.size());

        double last_x = -1e9;
        for (const auto& pt : input) {
            if (pt.first < 0.05) continue;
            if (!out.empty() && pt.first < last_x - 0.15) continue;
            if (!out.empty() &&
                std::hypot(pt.first - out.back().first, pt.second - out.back().second) < 0.10) {
                continue;
            }
            out.push_back(pt);
            last_x = pt.first;
        }
        return out;
    }

    static double lateral_at_x(
        const std::vector<std::pair<double,double>>& path,
        double xq)
    {
        if (path.empty()) return 0.0;
        if (xq <= path.front().first) return path.front().second;

        for (size_t i = 1; i < path.size(); ++i) {
            if (xq <= path[i].first) {
                const auto& p0 = path[i - 1];
                const auto& p1 = path[i];
                const double dx = std::max(p1.first - p0.first, 1e-3);
                const double t = std::clamp((xq - p0.first) / dx, 0.0, 1.0);
                return p0.second + (p1.second - p0.second) * t;
            }
        }
        return path.back().second;
    }

    static double heading_at(
        const std::vector<std::pair<double,double>>& path,
        size_t idx)
    {
        if (path.size() < 2) return 0.0;
        const size_t i0 = (idx == 0) ? 0 : idx - 1;
        const size_t i1 = std::min(idx + 1, path.size() - 1);
        return std::atan2(path[i1].second - path[i0].second,
                          std::max(path[i1].first - path[i0].first, 1e-3));
    }

    static double preview_lateral(
        const std::vector<std::pair<double,double>>& path,
        double x_limit)
    {
        double v = 0.0;
        for (const auto& pt : path) {
            if (pt.first > x_limit) break;
            v = std::max(v, std::abs(pt.second));
        }
        return v;
    }

    static double preview_curvature(
        const std::vector<std::pair<double,double>>& path,
        size_t max_idx)
    {
        if (path.size() < 4) return 0.0;

        const size_t n = std::min(max_idx, path.size() - 2);
        double max_curv = 0.0;

        for (size_t i = 1; i < n; ++i) {
            const double h0 = heading_at(path, i - 1);
            const double h1 = heading_at(path, i + 1);
            const double ds = std::max(
                std::hypot(path[i + 1].first - path[i - 1].first,
                           path[i + 1].second - path[i - 1].second),
                0.2);
            max_curv = std::max(max_curv, std::abs(h1 - h0) / ds);
        }
        return max_curv;
    }

    static double compute_lookahead(double cur_v, double preview_lat, double preview_curv)
    {
        double ld = 3.8 + 0.24 * cur_v;
        ld -= 0.22 * std::min(preview_lat, 3.0);
        ld -= 4.5 * std::min(preview_curv, 0.09);
        return std::clamp(ld, 3.0, 8.6);
    }

    static std::pair<double,double> target_by_arc(
        const std::vector<std::pair<double,double>>& path,
        double lookahead,
        size_t& out_idx)
    {
        double accum = 0.0;
        double px = 0.0, py = 0.0;

        for (size_t i = 0; i < path.size(); ++i) {
            accum += std::hypot(path[i].first - px, path[i].second - py);
            px = path[i].first;
            py = path[i].second;
            if (accum >= lookahead) {
                out_idx = i;
                return path[i];
            }
        }

        out_idx = path.size() - 1;
        return path.back();
    }

    static double estimate_curvature_ff(
        const std::vector<std::pair<double,double>>& path,
        size_t idx)
    {
        if (path.size() < 3) return 0.0;

        const size_t i0 = (idx == 0) ? 0 : idx - 1;
        const size_t i2 = std::min(idx + 1, path.size() - 1);

        const double x1 = path[i0].first;
        const double y1 = path[i0].second;
        const double x2 = path[idx].first;
        const double y2 = path[idx].second;
        const double x3 = path[i2].first;
        const double y3 = path[i2].second;

        const double a = std::hypot(x2 - x1, y2 - y1);
        const double b = std::hypot(x3 - x2, y3 - y2);
        const double c = std::hypot(x3 - x1, y3 - y1);
        const double area2 = std::abs((x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1));
        const double denom = std::max(a * b * c, 1e-3);

        return 2.0 * area2 / denom;
    }
};