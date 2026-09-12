#pragma once

#include <vector>
#include <cmath>
#include <algorithm>
#include <utility>
#include <limits>

#include "pure_pursuit.hpp"

struct Obstacle {
    double track_id = -1.0;
    double x = 0.0, y = 0.0, size_x = 0.0, size_y = 0.0, size_z = 0.0;
    double type_id = 0.0, speed = 0.0, camera_confirmed = 0.0, motion_state = 0.0;
};

struct LocalObstacle {
    double track_id = -1.0;
    double x = 0.0, y = 0.0, size_x = 0.0, size_y = 0.0, size_z = 0.0;
    double type_id = 0.0, speed = 0.0, camera_confirmed = 0.0, motion_state = 0.0;
};

class LatticePlanner {
private:
    bool has_prev_offset_ = false;
    double prev_offset_ = 0.0;

    struct PathPose { double x, y, yaw, s; };
    static constexpr double ONE_LANE_SHIFT = 2.8; // 차선폭 약간 축소하여 오버스티어 방지
    static constexpr double TWO_LANE_SHIFT = 5.6;
    static constexpr double STOP_CLOSE_X = 5.5;

    static double normalize_angle(double a) {
        while (a > M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }

    static double point_to_path_distance(double px, double py, const std::vector<std::pair<double,double>>& path) {
        double best = std::numeric_limits<double>::max();
        for (const auto& pt : path) best = std::min(best, std::hypot(pt.first - px, pt.second - py));
        return best;
    }

    static double quintic_blend(double t) {
        t = std::clamp(t, 0.0, 1.0);
        return 6.0 * std::pow(t, 5) - 15.0 * std::pow(t, 4) + 10.0 * std::pow(t, 3);
    }

    static std::vector<PathPose> build_arc_path(const std::vector<std::pair<double,double>>& center_path, double spacing = 0.35) {
        std::vector<PathPose> out; if (center_path.size() < 2) return out;
        std::vector<std::pair<double,double>> dense; dense.push_back(center_path.front());
        for (size_t i = 1; i < center_path.size(); ++i) {
            const auto& p0 = center_path[i - 1]; const auto& p1 = center_path[i];
            const double seg = std::hypot(p1.first - p0.first, p1.second - p0.second);
            const int steps = std::max(1, static_cast<int>(std::ceil(seg / std::max(0.10, spacing))));
            for (int k = 1; k <= steps; ++k) {
                const double t = static_cast<double>(k) / static_cast<double>(steps);
                dense.push_back({ p0.first + (p1.first - p0.first) * t, p0.second + (p1.second - p0.second) * t });
            }
        }
        out.reserve(dense.size()); double s = 0.0;
        for (size_t i = 0; i < dense.size(); ++i) {
            if (i > 0) s += std::hypot(dense[i].first - dense[i - 1].first, dense[i].second - dense[i - 1].second);
            const size_t i0 = (i == 0) ? 0 : i - 1; const size_t i1 = std::min(i + 1, dense.size() - 1);
            const double yaw = std::atan2(dense[i1].second - dense[i0].second, dense[i1].first - dense[i0].first + 1e-6);
            out.push_back({dense[i].first, dense[i].second, yaw, s});
        }
        return out;
    }

    static void smooth_path(std::vector<std::pair<double,double>>& path, int iterations = 3) {
        if (path.size() < 5) return;
        for (int it = 0; it < iterations; ++it) {
            auto copy = path;
            for (size_t i = 2; i + 2 < path.size(); ++i) {
                copy[i].first = 0.08 * path[i - 2].first + 0.22 * path[i - 1].first + 0.40 * path[i].first + 0.22 * path[i + 1].first + 0.08 * path[i + 2].first;
                copy[i].second = 0.08 * path[i - 2].second + 0.22 * path[i - 1].second + 0.40 * path[i].second + 0.22 * path[i + 1].second + 0.08 * path[i + 2].second;
            }
            path.swap(copy);
        }
    }

    static std::vector<std::pair<double,double>> sanitize_forward_path(const std::vector<std::pair<double,double>>& input) {
        std::vector<std::pair<double,double>> out; out.reserve(input.size()); double last_x = -1e9;
        for (const auto& pt : input) {
            if (pt.first < 0.0) continue;
            if (!out.empty() && pt.first < last_x - 0.10) continue;
            if (!out.empty() && std::hypot(pt.first - out.back().first, pt.second - out.back().second) < 0.10) continue;
            out.push_back(pt); last_x = pt.first;
        }
        return out;
    }

    static double offset_profile(double s, double target_offset, double shift_start, double shift_end, double hold_end, double return_end, bool allow_return) {
        if (s <= shift_start) return 0.0;
        if (s < shift_end) {
            const double t = (s - shift_start) / std::max(shift_end - shift_start, 1e-6);
            return target_offset * quintic_blend(t);
        }
        if (!allow_return || s <= hold_end) return target_offset;
        if (s < return_end) {
            const double t = (s - hold_end) / std::max(return_end - hold_end, 1e-6);
            return target_offset * (1.0 - quintic_blend(t));
        }
        return 0.0;
    }

    static std::vector<std::pair<double,double>> build_lane_change_path(
        const std::vector<std::pair<double,double>>& center_path, double target_offset, double obstacle_x, double obstacle_half_x, bool allow_return) {
        std::vector<std::pair<double,double>> candidate;
        if (center_path.size() < 2) return candidate;
        const auto arc_path = build_arc_path(center_path, 0.30);
        if (arc_path.size() < 8) return candidate;

        const double total_s = arc_path.back().s;
        const double obs_s = std::clamp(obstacle_x, 0.0, std::max(total_s - 8.0, 0.0));
        
        // ★ 조향 부드럽게 만들기: 차선 변경 구간을 대폭 길게 설정하여 완만하게 조향하도록 수정
        const double shift_start = 1.0;
        const double shift_len = 15.0; // 기존 8.5m -> 15.0m로 연장
        const double shift_end = std::clamp(obs_s - std::max(5.0, obstacle_half_x + 2.5), shift_start + 5.0, shift_start + shift_len);
        
        const double hold_end = std::min(total_s - 4.0, obs_s + std::max(8.0, obstacle_half_x + 6.0));
        const double return_len = 20.0; // 복귀 시에도 훨씬 더 길게 (15.0m -> 20.0m) 완만하게 복귀
        const double return_end = std::min(total_s, hold_end + return_len);

        candidate.reserve(arc_path.size());
        for (const auto& pose : arc_path) {
            const double d = offset_profile(pose.s, target_offset, shift_start, shift_end, hold_end, return_end, allow_return);
            const double nx = -std::sin(pose.yaw), ny = std::cos(pose.yaw);
            candidate.push_back({pose.x + d * nx, pose.y + d * ny});
        }
        smooth_path(candidate, 4); // 스무딩 횟수 증가
        return sanitize_forward_path(candidate);
    }

public:
    void reset_memory() { has_prev_offset_ = false; prev_offset_ = 0.0; }

    bool plan(
        const std::vector<std::pair<double,double>>& center_path, const std::vector<LocalObstacle>& local_obstacles,
        std::vector<std::pair<double,double>>& out_path, bool& should_stop, double& selected_offset, double& recommended_speed,
        bool use_fixed_offset = false, double fixed_offset = 0.0, bool allow_return = false,
        double force_obs_x = -100.0, double force_obs_half_x = 2.0) 
    {
        out_path.clear(); should_stop = false; selected_offset = 0.0;
        if (center_path.size() < 8) { should_stop = true; return false; }

        bool found_blocking = false; LocalObstacle blocking{};
        double best_obs_x = std::numeric_limits<double>::max(); double blocking_half_x = 2.0;

        for (const auto& obs : local_obstacles) {
            if (obs.x < -3.0 || obs.x > 42.0) continue;
            const bool pedestrian_like = (obs.type_id == 1.0), vehicle_dynamic = (obs.type_id == 2.0 && obs.motion_state == 2.0);
            const bool dynamic_like = pedestrian_like || vehicle_dynamic;
            const double gate = dynamic_like ? 2.8 : 2.2;
            if (point_to_path_distance(obs.x, obs.y, center_path) > gate) continue;

            if (dynamic_like && !use_fixed_offset && obs.x > -0.5 && obs.x < 16.0) { should_stop = true; return false; }
            if (!dynamic_like && obs.x > 1.0 && obs.x < best_obs_x) {
                best_obs_x = obs.x; blocking = obs; blocking_half_x = std::max(2.0, obs.size_x * 0.5 + 1.0); found_blocking = true;
            }
        }

        if (use_fixed_offset) {
            double safe_offset = fixed_offset;
            if (std::abs(safe_offset) < 0.5) safe_offset = found_blocking ? ((blocking.y >= 0.0) ? -ONE_LANE_SHIFT : ONE_LANE_SHIFT) : ONE_LANE_SHIFT;
            safe_offset = std::clamp(safe_offset, -TWO_LANE_SHIFT, TWO_LANE_SHIFT);

            const double ref_x = (force_obs_x > -50.0) ? force_obs_x : (found_blocking ? blocking.x : 18.0);
            const double ref_half_x = (force_obs_x > -50.0) ? force_obs_half_x : (found_blocking ? blocking_half_x : 2.0);

            auto committed_candidate = build_lane_change_path(center_path, safe_offset, ref_x, ref_half_x, allow_return);
            if (committed_candidate.size() >= 8) {
                out_path = committed_candidate; selected_offset = safe_offset;
                prev_offset_ = selected_offset; has_prev_offset_ = true;
                return true;
            }
        }

        if (!found_blocking) { out_path = center_path; return true; }
        if (!use_fixed_offset && blocking.x < STOP_CLOSE_X) { should_stop = true; return false; }
        return false;
    }
};