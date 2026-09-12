#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <carla_msgs/msg/carla_ego_vehicle_control.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>
#include <utility>

#include "pure_pursuit.hpp"
#include "lattice_planner.hpp"
#include "traffic_manager.hpp"

enum class DriveMode {
    PURE_PURSUIT,
    FOLLOW_LEAD,
    AVOID_HOLD,
    AVOID_RETURN,
    OBSTACLE_STOP
};

class MasterControlNode : public rclcpp::Node {
public:
    MasterControlNode() : Node("master_control_node") {
        // The release does not contain a competition route. Pass route_file at runtime.
        this->declare_parameter<std::string>("route_file", "");
        this->declare_parameter<std::string>("route_csv_frame", "ros");
        this->declare_parameter<bool>("use_il_normal_control", true);
        this->declare_parameter<bool>("enable_unsigned_stopline_stop", false);
        this->declare_parameter<double>("il_timeout_sec", 0.5);
        this->declare_parameter<std::string>(
            "il_control_topic", "/carla/ego_vehicle/vehicle_control_cmd_il");

        use_il_normal_control_ = this->get_parameter("use_il_normal_control").as_bool();
        enable_unsigned_stopline_stop_ = this->get_parameter("enable_unsigned_stopline_stop").as_bool();
        il_timeout_sec_ = this->get_parameter("il_timeout_sec").as_double();
        const auto il_control_topic = this->get_parameter("il_control_topic").as_string();

        control_pub_ = this->create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>(
            "/carla/ego_vehicle/vehicle_control_cmd", 10);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/lattice_path", 10);
        mode_pub_ = this->create_publisher<std_msgs::msg::String>("/hybrid_control/mode", 10);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/carla/ego_vehicle/odometry", 10,
            std::bind(&MasterControlNode::odom_callback, this, std::placeholders::_1));

        light_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/perception/traffic_light", 10,
            std::bind(&MasterControlNode::light_callback, this, std::placeholders::_1));

        stop_line_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/perception/stop_line", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) { stop_line_detected_ = msg->data; });

        obstacle_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/detected_obstacles", 10,
            std::bind(&MasterControlNode::obstacle_callback, this, std::placeholders::_1));

        il_control_sub_ = this->create_subscription<carla_msgs::msg::CarlaEgoVehicleControl>(
            il_control_topic, 10,
            std::bind(&MasterControlNode::il_control_callback, this, std::placeholders::_1));

        const auto route_file = this->get_parameter("route_file").as_string();
        const auto route_csv_frame = this->get_parameter("route_csv_frame").as_string();
        load_waypoints(route_file, route_csv_frame);
        RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints from %s (csv_frame=%s)",
                    waypoints_.size(), route_file.c_str(), route_csv_frame.c_str());
        RCLCPP_INFO(this->get_logger(),
                    "Master control node started: BC IL normal control + lattice fallback enabled");
        last_light_update_time_ = this->now();
    }

private:
    struct TrackedStaticObstacle {
        bool valid = false;
        int track_id = -1;
        double global_x = 0.0;
        double global_y = 0.0;
        double size_x = 4.5;
        double size_y = 2.0;
        double size_z = 1.7;
    };

    struct LeadVehicle {
        bool valid = false;
        int track_id = -1;
        double local_x = 0.0;
        double local_y = 0.0;
        double global_x = 0.0;
        double global_y = 0.0;
        double speed = 0.0;
        double size_x = 4.5;
        double size_y = 2.0;
    };

    struct Intrusion {
        bool detected = false;
        int track_id = -1;
        double x = 0.0;
        double y = 0.0;
        double dist = 1e9;
    };

    rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr control_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr light_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_line_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr obstacle_sub_;
    rclcpp::Subscription<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr il_control_sub_;

    std::vector<Waypoint> waypoints_;
    std::vector<Obstacle> obstacles_;
    std::vector<std::pair<double, double>> last_valid_command_path_;

    carla_msgs::msg::CarlaEgoVehicleControl latest_il_control_;
    rclcpp::Time last_il_control_time_;
    bool il_control_received_ = false;
    bool use_il_normal_control_ = true;
    bool enable_unsigned_stopline_stop_ = false;
    double il_timeout_sec_ = 0.5;

    std::string current_light_ = "UNKNOWN";
    bool stop_line_detected_ = false;

    DriveMode current_mode_ = DriveMode::PURE_PURSUIT;
    DriveMode resume_mode_after_stop_ = DriveMode::PURE_PURSUIT;

    TrackedStaticObstacle tracked_static_;
    LeadVehicle tracked_lead_;

    bool avoidance_committed_ = false;
    bool avoid_target_locked_ = false;
    double committed_offset_ = 0.0;
    int route_progress_idx_ = 0;
    int static_seen_count_ = 0;
    int dynamic_clear_count_ = 0;
    int return_finish_count_ = 0;
    int stop_hold_count_ = 0;
    int lead_lost_count_ = 0;
    int pending_static_track_id_ = -1;
    int same_static_track_count_ = 0;

    // signalized stop with lead vehicle
    bool red_light_follow_hold_ = false;

    // unsignalized stop-line latch
    bool prev_stop_line_detected_ = false;
    bool unsigned_stop_hold_active_ = false;
    int unsigned_stop_hold_ticks_ = 0;

    rclcpp::Time last_light_update_time_;
    bool light_initialized_ = false;
    int continuous_light_seen_count_ = 0;
    bool passed_signalized_intersection_ = false;
    int stop_line_ignore_ticks_ = 0;
    int stop_line_ignore_after_green_ticks_ = 0;

    const double TRAFFIC_LIGHT_TIMEOUT_SEC = 0.7;
    const double LIGHT_TIMEOUT_SEC = 0.45;
    const int CONTINUOUS_LIGHT_CONFIRM = 5;
    const int STOP_LINE_IGNORE_TICKS = 35;
    const int STOP_LINE_IGNORE_AFTER_GREEN_TICKS = 50;  // 10Hz 기준 약 5초

    const double MERGE_FRONT_CHECK = 18.0;
    const double MERGE_REAR_CHECK = 12.0;
    const double MERGE_PATH_BLOCK_DIST = 1.6;
    const double MERGE_MOVING_MIN_SPEED = 0.6;
    const int MERGE_SAFE_CONFIRM_COUNT = 3;
    const int MERGE_CLEAR_CONFIRM = 3;
    const double MERGE_BLOCKER_MIN_SPEED = 1.0;      // moving vehicle in target lane
    const double MERGE_BLOCKER_LOOKAHEAD = 18.0;
    const double MERGE_BLOCKER_PATH_DIST = 1.7;

    const int UNSIG_STOP_HOLD_TICKS = 30;            // ~3 sec at 10Hz
    const double RED_LIGHT_LEAD_LOOKAHEAD = 22.0;
    int merge_safe_count_ = 0;

    const int STATIC_CONFIRM_COUNT = 3;
    const int DYNAMIC_CLEAR_THRESHOLD = 4;   // 기존 6 -> 4
    const int RETURN_FINISH_THRESHOLD = 8;
    const int STOP_HOLD_TICKS = 2;           // 기존 4 -> 2
    const int LEAD_LOST_THRESHOLD = 8;

    const double HIGHWAY_SPEED = 35.0 / 3.6;
    const double FOLLOW_MAX_SPEED = 28.0 / 3.6;
    const double AVOID_SPEED = 10.0 / 3.6;
    const double RETURN_SPEED = 12.0 / 3.6;
    const double FRONT_BUMPER_X = 3.5;

    PurePursuit pure_pursuit_;
    LatticePlanner lattice_;
    TrafficManager traffic_;

    void load_waypoints(const std::string& path, const std::string& csv_frame) {
        if (path.empty()) {
            RCLCPP_ERROR(this->get_logger(),
                         "route_file is required; pass a CSV with x,y,z,yaw columns");
            return;
        }
        std::ifstream file(path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open waypoint file: %s", path.c_str());
            return;
        }

        std::string line;
        std::getline(file, line);
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string val;
            Waypoint wp{};
            std::getline(ss, val, ','); wp.x = std::stod(val);
            std::getline(ss, val, ','); wp.y = std::stod(val);
            std::getline(ss, val, ','); wp.z = std::stod(val);
            std::getline(ss, val, ','); wp.yaw = std::stod(val);
            if (csv_frame == "carla") {
                wp.y = -wp.y;
                wp.yaw = -wp.yaw;
            }
            waypoints_.push_back(wp);
        }
    }

    void obstacle_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        obstacles_.clear();
        const auto& d = msg->data;
        if (d.size() >= 10 && d.size() % 10 == 0) {
            for (size_t i = 0; i < d.size(); i += 10) {
                Obstacle obs{};
                obs.track_id = d[i + 0];
                obs.x = d[i + 1];
                obs.y = d[i + 2];
                obs.size_x = d[i + 3];
                obs.size_y = d[i + 4];
                obs.size_z = d[i + 5];
                obs.type_id = d[i + 6];
                obs.speed = d[i + 7];
                obs.camera_confirmed = d[i + 8];
                obs.motion_state = d[i + 9];
                obstacles_.push_back(obs);
            }
        }
    }

    void il_control_callback(const carla_msgs::msg::CarlaEgoVehicleControl::SharedPtr msg) {
        latest_il_control_ = *msg;
        latest_il_control_.steer = std::clamp(latest_il_control_.steer, -1.0f, 1.0f);
        latest_il_control_.throttle = std::clamp(latest_il_control_.throttle, 0.0f, 1.0f);
        latest_il_control_.brake = std::clamp(latest_il_control_.brake, 0.0f, 1.0f);
        latest_il_control_.hand_brake = false;
        latest_il_control_.manual_gear_shift = false;
        il_control_received_ = true;
        last_il_control_time_ = this->now();
    }

    bool has_fresh_il_control() const {
        if (!il_control_received_) return false;
        return (this->now() - last_il_control_time_).seconds() <= il_timeout_sec_;
    }

    std::string current_mode_name(bool using_il, bool traffic_stop_active) const {
        if (using_il) return "BC_IL";
        if (traffic_stop_active) return "TRAFFIC_STOP";

        switch (current_mode_) {
            case DriveMode::PURE_PURSUIT: return "PURE_PURSUIT_FALLBACK";
            case DriveMode::FOLLOW_LEAD: return "FOLLOW_LEAD";
            case DriveMode::AVOID_HOLD: return "LATTICE_AVOID";
            case DriveMode::AVOID_RETURN: return "LATTICE_RETURN";
            case DriveMode::OBSTACLE_STOP: return "OBSTACLE_STOP";
        }
        return "UNKNOWN";
    }

    void publish_mode_state(const std::string& mode) {
        std_msgs::msg::String msg;
        msg.data = mode;
        mode_pub_->publish(msg);
    }

    void light_callback(const std_msgs::msg::String::SharedPtr msg) {
        std::string raw = msg->data;
        std::transform(raw.begin(), raw.end(), raw.begin(), ::toupper);

        last_light_update_time_ = this->now();
        light_initialized_ = true;

        if (raw.find("RED") != std::string::npos) {
            current_light_ = "RED";
        }
        else if (raw.find("YELLOW") != std::string::npos) {
            current_light_ = "YELLOW";
        }
        else if (raw.find("GREEN") != std::string::npos) {
            current_light_ = "GREEN";
            stop_line_ignore_after_green_ticks_ = STOP_LINE_IGNORE_AFTER_GREEN_TICKS;
        }
        else {
            current_light_ = "UNKNOWN";
        }
    }

    int find_reference_waypoint_index(double cur_x, double cur_y, double yaw) {
        if (waypoints_.empty()) return 0;

        int start_idx = std::max(0, route_progress_idx_ - 5);
        int end_idx = std::min(static_cast<int>(waypoints_.size()) - 1, route_progress_idx_ + 160);

        double hx = std::cos(yaw);
        double hy = std::sin(yaw);
        double best_cost = std::numeric_limits<double>::max();
        int best_idx = route_progress_idx_;

        for (int i = start_idx; i <= end_idx; ++i) {
            double dx = waypoints_[i].x - cur_x;
            double dy = waypoints_[i].y - cur_y;
            double along = dx * hx + dy * hy;
            double lateral = -dx * hy + dy * hx;
            if (along < -3.0) continue;
            double cost = std::hypot(dx, dy) + 0.04 * std::abs(lateral);
            if (along < 0.0) cost += 8.0;
            if (cost < best_cost) {
                best_cost = cost;
                best_idx = i;
            }
        }

        route_progress_idx_ = std::clamp(best_idx, route_progress_idx_,
                                         static_cast<int>(waypoints_.size()) - 1);
        return route_progress_idx_;
    }

    std::vector<std::pair<double, double>> build_center_path(
        int ref_idx, double cur_x, double cur_y, double yaw) const
    {
        std::vector<std::pair<double, double>> raw;
        raw.push_back({0.0, 0.0});

        int end_idx = std::min(static_cast<int>(waypoints_.size()) - 1, ref_idx + 180);
        double last_x = -1e9;
        double last_y = 0.0;

        for (int i = ref_idx; i <= end_idx; ++i) {
            double dx = waypoints_[i].x - cur_x;
            double dy = waypoints_[i].y - cur_y;
            double lx = dx * std::cos(-yaw) - dy * std::sin(-yaw);
            double ly = dx * std::sin(-yaw) + dy * std::cos(-yaw);

            if (lx < 0.35 || std::abs(std::atan2(ly, std::max(lx, 0.1))) > 1.20) continue;
            if (!raw.empty() && (lx < last_x - 0.12 || std::hypot(lx - last_x, ly - last_y) < 0.18)) continue;

            raw.push_back({lx, ly});
            last_x = lx;
            last_y = ly;

            if (lx > 65.0) break;
        }
        return raw;
    }

    std::vector<LocalObstacle> build_local_obstacles(double cur_x, double cur_y, double yaw) const {
        std::vector<LocalObstacle> out;
        out.reserve(obstacles_.size());

        for (const auto& obs : obstacles_) {
            double dx = obs.x - cur_x;
            double dy = obs.y - cur_y;

            LocalObstacle lo{};
            lo.track_id = obs.track_id;
            lo.x = dx * std::cos(-yaw) - dy * std::sin(-yaw);
            lo.y = dx * std::sin(-yaw) + dy * std::cos(-yaw);
            lo.size_x = obs.size_x;
            lo.size_y = obs.size_y;
            lo.size_z = obs.size_z;
            lo.type_id = obs.type_id;
            lo.speed = obs.speed;
            lo.camera_confirmed = obs.camera_confirmed;
            lo.motion_state = obs.motion_state;
            out.push_back(lo);
        }
        return out;
    }

    static double point_to_path_distance(
        double px, double py,
        const std::vector<std::pair<double, double>>& path)
    {
        double best = std::numeric_limits<double>::max();
        for (const auto& pt : path) {
            best = std::min(best, std::hypot(pt.first - px, pt.second - py));
        }
        return best;
    }

    static double near_path_lateral(
        const std::vector<std::pair<double, double>>& path, double x_limit)
    {
        double best = 1e9;
        double best_lat = 0.0;

        for (const auto& pt : path) {
            if (pt.first < 1.0) continue;
            if (pt.first > x_limit) break;

            double d = std::abs(pt.first - 6.0);
            if (d < best) {
                best = d;
                best_lat = pt.second;
            }
        }

        if (best >= 1e8 && !path.empty()) return path.back().second;
        return best_lat;
    }

    void update_tracked_static_from_track() {
        if (!tracked_static_.valid) return;

        for (const auto& obs : obstacles_) {
            if (static_cast<int>(obs.track_id) == tracked_static_.track_id) {
                tracked_static_.global_x = obs.x;
                tracked_static_.global_y = obs.y;
                tracked_static_.size_x = obs.size_x;
                tracked_static_.size_y = obs.size_y;
                tracked_static_.size_z = obs.size_z;
                return;
            }
        }
    }

    bool select_blocking_static_obstacle(
        const std::vector<std::pair<double, double>>& center_path,
        const std::vector<LocalObstacle>& local_obs,
        TrackedStaticObstacle& out_tracked,
        double& suggested_offset)
    {
        bool found = false;
        double best_cost = std::numeric_limits<double>::max();
        size_t best_idx = 0;

        for (size_t i = 0; i < local_obs.size(); ++i) {
            const auto& obs = local_obs[i];
            if (obs.camera_confirmed < 0.5) continue;
            if (obs.type_id != 2.0) continue;
            if (obs.x < 4.0 || obs.x > 35.0) continue;

            // STATIC definition aligned with lidar_perception:
            // stopped or near-stopped vehicle
            if (obs.motion_state != 0.0) continue;
            if (obs.speed > 2.0) continue;   // ~2.9 km/h

            // 같은 차선 중앙에 있는 차량은 정지해 있어도 회피 장애물이 아니라
            // 앞차 추종 대상으로 유지한다.
            if (obs.x > 0.5 && std::abs(obs.y) < 1.8) {
                continue;
            }

            double min_dist = point_to_path_distance(obs.x, obs.y, center_path);

            if (min_dist > 1.7) continue;

            double cost = obs.x + 0.5 * std::abs(obs.y);
            if (cost < best_cost) {
                best_cost = cost;
                best_idx = i;
                found = true;
            }
        }

        if (!found) return false;

        const auto& gobs = obstacles_[best_idx];
        out_tracked.valid = true;
        out_tracked.track_id = static_cast<int>(gobs.track_id);
        out_tracked.global_x = gobs.x;
        out_tracked.global_y = gobs.y;
        out_tracked.size_x = gobs.size_x;
        out_tracked.size_y = gobs.size_y;
        out_tracked.size_z = gobs.size_z;

        suggested_offset = (local_obs[best_idx].y >= 0.0) ? -4.5 : 4.5;
        if (std::abs(local_obs[best_idx].y) < 0.8) suggested_offset = 3.6;

        return true;
    }

    bool select_lead_vehicle(
        const std::vector<std::pair<double, double>>& center_path,
        const std::vector<LocalObstacle>& local_obs,
        LeadVehicle& out_lead)
    {
        bool found = false;
        double best_score = std::numeric_limits<double>::max();
        size_t best_idx = 0;

        for (size_t i = 0; i < local_obs.size(); ++i) {
            const auto& obs = local_obs[i];
            if (obs.camera_confirmed < 0.5) continue;
            if (obs.type_id != 2.0) continue;
            if (obs.x < 4.0 || obs.x > 32.0) continue;
            if (point_to_path_distance(obs.x, obs.y, center_path) > 1.6) continue;
            if (obs.motion_state != 1.0) continue;   // only SLOW_MOVING_IN_LANE
            if (obs.speed < 2.0) continue;
            if (obs.speed > 6.0) continue;

            const double score = obs.x + 0.3 * std::abs(obs.y);
            if (score < best_score) {
                best_score = score;
                best_idx = i;
                found = true;
            }
        }

        if (!found) return false;

        const auto& lobs = local_obs[best_idx];
        const auto& gobs = obstacles_[best_idx];
        out_lead.valid = true;
        out_lead.track_id = static_cast<int>(gobs.track_id);
        out_lead.local_x = lobs.x;
        out_lead.local_y = lobs.y;
        out_lead.global_x = gobs.x;
        out_lead.global_y = gobs.y;
        out_lead.speed = gobs.speed;
        out_lead.size_x = gobs.size_x;
        out_lead.size_y = gobs.size_y;
        return true;
    }

    bool select_forward_vehicle_on_path(
        const std::vector<std::pair<double, double>>& path,
        const std::vector<LocalObstacle>& local_obs,
        LeadVehicle& out_lead,
        int ignore_track_id = -1)
    {
        bool found = false;
        double best_score = std::numeric_limits<double>::max();
        size_t best_idx = 0;

        for (size_t i = 0; i < local_obs.size(); ++i) {
            const auto& obs = local_obs[i];
            if (static_cast<int>(obs.track_id) == ignore_track_id) continue;
            if (obs.camera_confirmed < 0.5) continue;
            if (obs.type_id != 2.0) continue;
            if (obs.x < 3.0 || obs.x > 28.0) continue;
            if (point_to_path_distance(obs.x, obs.y, path) > 1.8) continue;

            double state_penalty = 0.0;
            if (obs.motion_state == 1.0) state_penalty = 0.0;
            else if (obs.motion_state == 3.0 || obs.motion_state == 4.0) state_penalty = 1.5;
            else if (obs.motion_state == 0.0) state_penalty = 0.5;
            else state_penalty = 3.0;

            const double speed_penalty = std::max(0.0, 1.0 - obs.speed) * 0.6;
            const double score = obs.x + state_penalty + speed_penalty + 0.35 * std::abs(obs.y);
            if (score < best_score) {
                best_score = score;
                best_idx = i;
                found = true;
            }
        }

        if (!found) return false;

        const auto& lobs = local_obs[best_idx];
        const auto& gobs = obstacles_[best_idx];
        out_lead.valid = true;
        out_lead.track_id = static_cast<int>(gobs.track_id);
        out_lead.local_x = lobs.x;
        out_lead.local_y = lobs.y;
        out_lead.global_x = gobs.x;
        out_lead.global_y = gobs.y;
        out_lead.speed = gobs.speed;
        out_lead.size_x = gobs.size_x;
        out_lead.size_y = gobs.size_y;
        return true;
    }

    void apply_follow_speed_control(
        const LeadVehicle& lead,
        double cur_v,
        double& target_speed,
        bool& should_stop,
        double desired_gap_bias = 5.0,
        double hard_stop_gap = 3.8) const
    {
        const double gap = lead.local_x - 0.5 * lead.size_x - FRONT_BUMPER_X;
        const double desired_gap = std::max(8.0, 1.10 * cur_v + desired_gap_bias);
        double follow_speed = lead.speed + 0.32 * (gap - desired_gap);
        follow_speed = std::clamp(follow_speed, 0.0, FOLLOW_MAX_SPEED);
        target_speed = std::min(target_speed, follow_speed);

        if (gap < hard_stop_gap) {
            should_stop = true;
            target_speed = 0.0;
        }
    }

    void apply_red_light_lead_stop_control(
        const LeadVehicle& lead,
        double cur_v,
        double& target_speed,
        bool& should_stop) const
    {
        // 빨간불 정지 시에는 일반 follow보다 더 촘촘한 gap 사용
        apply_follow_speed_control(lead, cur_v, target_speed, should_stop, 1.8, 2.6);

        const double gap = lead.local_x - 0.5 * lead.size_x - FRONT_BUMPER_X;

        // 앞차가 거의 정지했고 충분히 가까우면 나도 확실히 정지
        if (lead.speed < 0.6 && gap < 6.0) {
            should_stop = true;
            target_speed = 0.0;
        }
    }

    bool has_signal_context() const {
        return current_light_ == "RED" || current_light_ == "YELLOW" || current_light_ == "GREEN";
    }

    bool select_signal_queue_lead(
        const std::vector<std::pair<double, double>>& center_path,
        const std::vector<LocalObstacle>& local_obs,
        LeadVehicle& out_lead,
        int ignore_track_id = -1) const
    {
        return is_vehicle_on_path_ahead(
            center_path,
            local_obs,
            out_lead,
            ignore_track_id,
            0.5,   // x_min
            24.0,  // x_max
            1.8    // path_dist_thresh
        );
    }

    void apply_red_light_queue_stop_control(
        const LeadVehicle& lead,
        double& target_speed,
        bool& should_stop) const
    {
        // 차량 중심 간 거리 기준
        const double center_distance = lead.local_x;

        // 너무 멀면 멈추지 말고 저속 접근
        if (center_distance > 11.0) {
            should_stop = false;
            target_speed = std::min(target_speed, 8.0 / 3.6);
            return;
        }

        // 정지 목표 구간
        if (center_distance > 5.0) {
            should_stop = false;
            target_speed = std::min(target_speed, 3.0 / 3.6);
            return;
        }

        // 목표 거리 도달 시 확실히 정지
        should_stop = true;
        target_speed = 0.0;
    }

    bool is_red_like_signal() const {
        return current_light_ == "RED" || current_light_ == "YELLOW";
    }

    bool is_vehicle_on_path_ahead(
        const std::vector<std::pair<double, double>>& path,
        const std::vector<LocalObstacle>& local_obs,
        LeadVehicle& out_lead,
        int ignore_track_id = -1,
        double x_min = 0.5,
        double x_max = 25.0,
        double path_dist_thresh = 1.8) const
    {
        bool found = false;
        double best_score = std::numeric_limits<double>::max();
        size_t best_idx = 0;

        for (size_t i = 0; i < local_obs.size(); ++i) {
            const auto& obs = local_obs[i];
            if (static_cast<int>(obs.track_id) == ignore_track_id) continue;
            if (obs.camera_confirmed < 0.5) continue;
            if (obs.type_id != 2.0) continue;
            if (obs.x < x_min || obs.x > x_max) continue;
            if (point_to_path_distance(obs.x, obs.y, path) > path_dist_thresh) continue;

            const double score = obs.x + 0.25 * std::abs(obs.y);
            if (score < best_score) {
                best_score = score;
                best_idx = i;
                found = true;
            }
        }

        if (!found) return false;

        const auto& lobs = local_obs[best_idx];
        const auto& gobs = obstacles_[best_idx];
        out_lead.valid = true;
        out_lead.track_id = static_cast<int>(gobs.track_id);
        out_lead.local_x = lobs.x;
        out_lead.local_y = lobs.y;
        out_lead.global_x = gobs.x;
        out_lead.global_y = gobs.y;
        out_lead.speed = gobs.speed;
        out_lead.size_x = gobs.size_x;
        out_lead.size_y = gobs.size_y;
        return true;
    }

    void handle_unsigned_stopline_latch(bool right_turn_ahead) {
        const bool effective_stop_line = effective_stop_line_detected();
        const bool rising_edge = effective_stop_line && !prev_stop_line_detected_;

        if (enable_unsigned_stopline_stop_ &&
            !right_turn_ahead &&
            rising_edge &&
            current_light_ == "UNKNOWN")
        {
            unsigned_stop_hold_active_ = true;
            unsigned_stop_hold_ticks_ = UNSIG_STOP_HOLD_TICKS;
        }

        prev_stop_line_detected_ = effective_stop_line;
    }

    void apply_unsigned_stopline_hold(bool& should_stop, double& target_speed) {
        if (!unsigned_stop_hold_active_) return;
        if (!traffic_stop_allowed()) return;

        should_stop = true;
        target_speed = 0.0;

        if (unsigned_stop_hold_ticks_ > 0) {
            unsigned_stop_hold_ticks_--;
        } else {
            unsigned_stop_hold_active_ = false;
        }
    }

    bool traffic_stop_allowed() const {
        return current_mode_ != DriveMode::AVOID_HOLD &&
               current_mode_ != DriveMode::AVOID_RETURN &&
               current_mode_ != DriveMode::OBSTACLE_STOP;
    }

    static double normalize_angle(double a) {
        while (a > M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }

    void refresh_light_tracking_state() {
        if (!light_initialized_) {
            current_light_ = "UNKNOWN";
            continuous_light_seen_count_ = 0;
            return;
        }

        const double dt = (this->now() - last_light_update_time_).seconds();
        if (dt > LIGHT_TIMEOUT_SEC) {
            current_light_ = "UNKNOWN";
        }

        if (current_light_ != "UNKNOWN") {
            continuous_light_seen_count_++;
            if (continuous_light_seen_count_ >= CONTINUOUS_LIGHT_CONFIRM) {
                passed_signalized_intersection_ = true;
            }
        } else {
            if (passed_signalized_intersection_) {
                stop_line_ignore_ticks_ = STOP_LINE_IGNORE_TICKS;
                passed_signalized_intersection_ = false;
            }
            continuous_light_seen_count_ = 0;
        }

        if (stop_line_ignore_ticks_ > 0) {
            stop_line_ignore_ticks_--;
        }
    }

    bool effective_stop_line_detected() const {
        if (stop_line_ignore_after_green_ticks_ > 0) {
            return false;
        }
        if (stop_line_ignore_ticks_ > 0) {
            return false;
        }
        return stop_line_detected_;
    }

    static double path_lateral_at_x(
        const std::vector<std::pair<double, double>>& path,
        double x_query)
    {
        if (path.empty()) return 0.0;
        if (path.size() == 1) return path.front().second;

        for (size_t i = 1; i < path.size(); ++i) {
            const auto& p0 = path[i - 1];
            const auto& p1 = path[i];
            if ((p0.first <= x_query && x_query <= p1.first) ||
                (p1.first <= x_query && x_query <= p0.first))
            {
                const double dx = p1.first - p0.first;
                if (std::abs(dx) < 1e-6) return p1.second;
                const double t = (x_query - p0.first) / dx;
                return p0.second + t * (p1.second - p0.second);
            }
        }

        if (x_query <= path.front().first) return path.front().second;
        return path.back().second;
    }

    bool is_vehicle_occupying_path_corridor(
        const LocalObstacle& obs,
        const std::vector<std::pair<double, double>>& path,
        double front_limit,
        double rear_limit,
        double corridor_half_width) const
    {
        if (obs.camera_confirmed < 0.5) return false;
        if (obs.type_id != 2.0) return false;

        const double rear_x = obs.x - 0.5 * obs.size_x;
        const double front_x = obs.x + 0.5 * obs.size_x;

        if (front_x < -rear_limit) return false;
        if (rear_x > front_limit) return false;

        const double ref_y = path_lateral_at_x(path, std::max(0.0, obs.x));
        const double half_width = corridor_half_width + 0.5 * obs.size_y;
        return std::abs(obs.y - ref_y) < half_width;
    }

    bool is_merge_target_lane_safe(
        const std::vector<std::pair<double, double>>& path,
        const std::vector<LocalObstacle>& local_obs,
        int ignore_track_id) const
    {
        for (const auto& obs : local_obs) {
            if (static_cast<int>(obs.track_id) == ignore_track_id) continue;
            if (!is_vehicle_occupying_path_corridor(
                    obs, path, MERGE_FRONT_CHECK, MERGE_REAR_CHECK, MERGE_PATH_BLOCK_DIST))
            {
                continue;
            }

            if (obs.motion_state == 0.0 && obs.speed < 0.3) {
                return false;
            }

            if (obs.speed >= MERGE_MOVING_MIN_SPEED) {
                return false;
            }

            if (obs.motion_state == 1.0 || obs.motion_state == 2.0 || obs.motion_state == 3.0) {
                return false;
            }
        }
        return true;
    }

    bool is_right_turn_ahead(int ref_idx) const {
        if (waypoints_.size() < 10 || ref_idx < 0 || ref_idx >= static_cast<int>(waypoints_.size()) - 5) {
            return false;
        }

        const int look_idx = std::min(ref_idx + 22, static_cast<int>(waypoints_.size()) - 1);
        const double yaw_now = waypoints_[ref_idx].yaw * M_PI / 180.0;
        const double yaw_ahead = waypoints_[look_idx].yaw * M_PI / 180.0;
        const double yaw_delta = normalize_angle(yaw_ahead - yaw_now);

        return yaw_delta < (-20.0 * M_PI / 180.0);
    }

    Intrusion detect_pedestrian_intrusion(
        const std::vector<LocalObstacle>& local_obs,
        const std::vector<std::pair<double, double>>& path,
        int ignore_id = -1) const
    {
        Intrusion intr;
        for (const auto& obs : local_obs) {
            if (static_cast<int>(obs.track_id) == ignore_id) continue;
            if (obs.camera_confirmed < 0.5) continue;
            if (obs.type_id != 1.0) continue;
            if (obs.x < -1.0 || obs.x > 14.0) continue;

            double min_dist = point_to_path_distance(obs.x, obs.y, path);
            double dist = std::hypot(obs.x, obs.y);
            if (min_dist < 2.2 && dist < intr.dist) {
                intr.detected = true;
                intr.track_id = static_cast<int>(obs.track_id);
                intr.x = obs.x;
                intr.y = obs.y;
                intr.dist = dist;
            }

        }
        return intr;
    }

    Intrusion detect_cutin_intrusion(
        const std::vector<LocalObstacle>& local_obs,
        const std::vector<std::pair<double, double>>& path,
        int ignore_id = -1) const
    {
        Intrusion intr;
        for (const auto& obs : local_obs) {
            if (static_cast<int>(obs.track_id) == ignore_id) continue;
            if (obs.camera_confirmed < 0.5) continue;
            if (obs.type_id != 2.0 || obs.motion_state != 2.0) continue;
            if (obs.x < -1.0 || obs.x > 12.0) continue;

            double min_dist = point_to_path_distance(obs.x, obs.y, path);
            double dist = std::hypot(obs.x, obs.y);
            if (min_dist < 2.2 && dist < intr.dist) {
                intr.detected = true;
                intr.track_id = static_cast<int>(obs.track_id);
                intr.x = obs.x;
                intr.y = obs.y;
                intr.dist = dist;
            }
        }
        return intr;
    }

    std::vector<std::pair<double, double>> sanitize_command_path(
        const std::vector<std::pair<double, double>>& candidate,
        const std::vector<std::pair<double, double>>& center_path)
    {
        if (candidate.size() >= 2) {
            last_valid_command_path_ = candidate;
            return candidate;
        }

        if (center_path.size() >= 2) {
            last_valid_command_path_ = center_path;
            return center_path;
        }

        if (last_valid_command_path_.size() >= 2) {
            return last_valid_command_path_;
        }

        std::vector<std::pair<double, double>> fallback;
        fallback.push_back({0.0, 0.0});
        fallback.push_back({3.0, 0.0});
        return fallback;
    }

    void publish_path(const std::vector<std::pair<double, double>>& path) {
        nav_msgs::msg::Path msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "ego_vehicle";

        for (const auto& pt : path) {
            geometry_msgs::msg::PoseStamped ps;
            ps.pose.position.x = pt.first;
            ps.pose.position.y = pt.second;
            msg.poses.push_back(ps);
        }

        path_pub_->publish(msg);
    }

    void enter_obstacle_stop(DriveMode resume_mode) {
        current_mode_ = DriveMode::OBSTACLE_STOP;
        resume_mode_after_stop_ = resume_mode;
        stop_hold_count_ = STOP_HOLD_TICKS;
        dynamic_clear_count_ = 0;
        pure_pursuit_.reset();
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (waypoints_.empty()) return;

        const double cur_x = msg->pose.pose.position.x;
        const double cur_y = msg->pose.pose.position.y;
        const double cur_v = std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y);

        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        refresh_light_tracking_state();

        int ref_idx = find_reference_waypoint_index(cur_x, cur_y, yaw);
        auto center_path_raw = build_center_path(ref_idx, cur_x, cur_y, yaw);
        auto center_path = sanitize_command_path(center_path_raw, center_path_raw);
        auto local_obs = build_local_obstacles(cur_x, cur_y, yaw);

        std::vector<std::pair<double, double>> command_path = center_path;
        bool should_stop = false;
        double target_speed = HIGHWAY_SPEED;

        LeadVehicle lead_candidate;
        TrackedStaticObstacle static_candidate;
        double suggested_offset = 0.0;

        if (stop_line_ignore_after_green_ticks_ > 0) {
            stop_line_ignore_after_green_ticks_--;
        }

        const bool right_turn_ahead = is_right_turn_ahead(ref_idx);
        handle_unsigned_stopline_latch(right_turn_ahead);
        bool has_lead = select_lead_vehicle(center_path, local_obs, lead_candidate);
        bool has_static = select_blocking_static_obstacle(center_path, local_obs, static_candidate, suggested_offset);
        LeadVehicle signal_queue_lead;
        const bool has_signal_queue_lead =
            has_signal_context() &&
            select_signal_queue_lead(center_path, local_obs, signal_queue_lead, tracked_static_.track_id);

        if (has_static) {
            if (pending_static_track_id_ == static_candidate.track_id) {
                same_static_track_count_++;
            } else {
                pending_static_track_id_ = static_candidate.track_id;
                same_static_track_count_ = 1;
            }
        } else {
            pending_static_track_id_ = -1;
            same_static_track_count_ = 0;
        }

        Intrusion ped_intr = detect_pedestrian_intrusion(local_obs, center_path, tracked_static_.track_id);
        Intrusion cutin_intr = detect_cutin_intrusion(local_obs, center_path, tracked_static_.track_id);
        bool dynamic_intrusion = ped_intr.detected || cutin_intr.detected;

        // 1) pedestrian / cut-in stop has top priority
        if (current_mode_ != DriveMode::OBSTACLE_STOP && dynamic_intrusion) {
            enter_obstacle_stop(current_mode_);
        }

        // 2) mode handling
        if (current_mode_ == DriveMode::PURE_PURSUIT) {
            if (has_signal_queue_lead) {
                tracked_lead_ = signal_queue_lead;
                lead_lost_count_ = 0;
                current_mode_ = DriveMode::FOLLOW_LEAD;
            }
            else if (has_static) {
                const double dist_to_obs = std::hypot(
                    static_candidate.global_x - cur_x,
                    static_candidate.global_y - cur_y);

                target_speed = std::min(target_speed, std::max(AVOID_SPEED, dist_to_obs * 0.22));
                static_seen_count_++;

                if (static_seen_count_ >= STATIC_CONFIRM_COUNT) {
                    tracked_static_ = static_candidate;
                    avoidance_committed_ = true;
                    committed_offset_ = suggested_offset;
                    current_mode_ = DriveMode::AVOID_HOLD;
                    lattice_.reset_memory();
                    pure_pursuit_.reset();
                }
            } else {
                static_seen_count_ = 0;

                if (!has_lead) {
                    has_lead = select_forward_vehicle_on_path(center_path, local_obs, lead_candidate, tracked_static_.track_id);
                }

                if (has_lead) {
                    tracked_lead_ = lead_candidate;
                    lead_lost_count_ = 0;
                    current_mode_ = DriveMode::FOLLOW_LEAD;
                }
            }
        }

        if (current_mode_ == DriveMode::FOLLOW_LEAD) {
            LeadVehicle queue_lead;
            const bool queue_vehicle_present =
                has_signal_context() &&
                select_signal_queue_lead(center_path, local_obs, queue_lead, tracked_static_.track_id);

            // 신호등이 보이고 전방 차량이 있으면 회피 금지, 무조건 같은 차선 follow
            if (queue_vehicle_present) {
                tracked_lead_ = queue_lead;
                lead_lost_count_ = 0;

                if (current_light_ == "RED" || current_light_ == "YELLOW") {
                    apply_red_light_queue_stop_control(tracked_lead_, target_speed, should_stop);
                } else {
                    // GREEN이어도 바로 추월 금지, 앞차 출발하면 따라감
                    target_speed = std::min(target_speed, HIGHWAY_SPEED);
                    apply_follow_speed_control(tracked_lead_, cur_v, target_speed, should_stop, 3.5, 3.0);
                }
            }
            else if (has_static) {
                tracked_static_ = static_candidate;
                avoidance_committed_ = true;
                committed_offset_ = suggested_offset;
                current_mode_ = DriveMode::AVOID_HOLD;
                lattice_.reset_memory();
                pure_pursuit_.reset();
            } else if (has_lead) {
                tracked_lead_ = lead_candidate;
                lead_lost_count_ = 0;

                target_speed = std::min(target_speed, HIGHWAY_SPEED);
                apply_follow_speed_control(tracked_lead_, cur_v, target_speed, should_stop);
            } else {
                LeadVehicle fallback_lead;
                if (select_forward_vehicle_on_path(center_path, local_obs, fallback_lead, tracked_static_.track_id)) {
                    tracked_lead_ = fallback_lead;
                    lead_lost_count_ = 0;
                    target_speed = std::min(target_speed, HIGHWAY_SPEED);
                    apply_follow_speed_control(tracked_lead_, cur_v, target_speed, should_stop);
                } else {
                    lead_lost_count_++;
                    if (lead_lost_count_ >= LEAD_LOST_THRESHOLD) {
                        tracked_lead_.valid = false;
                        current_mode_ = DriveMode::PURE_PURSUIT;
                        pure_pursuit_.reset();
                    }
                }
            }
        }

        if (current_mode_ == DriveMode::AVOID_HOLD) {
            update_tracked_static_from_track();

            double dx = tracked_static_.global_x - cur_x;
            double dy = tracked_static_.global_y - cur_y;
            double tracked_local_x = dx * std::cos(-yaw) - dy * std::sin(-yaw);
            double tracked_half_x = std::max(2.0, tracked_static_.size_x * 0.5 + 1.0);

            bool planner_stop = false;
            double selected_offset = committed_offset_;
            double dummy_speed = 0.0;

            lattice_.plan(
                center_path,
                local_obs,
                command_path,
                planner_stop,
                selected_offset,
                dummy_speed,
                true,
                committed_offset_,
                false,
                tracked_local_x,
                tracked_half_x
            );

            command_path = sanitize_command_path(command_path, center_path);
            target_speed = AVOID_SPEED;
            committed_offset_ = selected_offset;

            LeadVehicle merge_lead;
            if (select_forward_vehicle_on_path(command_path, local_obs, merge_lead, tracked_static_.track_id)) {
                apply_follow_speed_control(merge_lead, cur_v, target_speed, should_stop, 4.5, 4.0);
            }

            if (tracked_local_x < -(FRONT_BUMPER_X + tracked_static_.size_x * 0.5 + 0.8)) {
                current_mode_ = DriveMode::AVOID_RETURN;
                return_finish_count_ = 0;
                pure_pursuit_.reset();
            }
        }

        if (current_mode_ == DriveMode::AVOID_RETURN) {
            command_path = sanitize_command_path(center_path, center_path);
            target_speed = RETURN_SPEED;

            LeadVehicle return_lead;
            if (select_forward_vehicle_on_path(command_path, local_obs, return_lead, tracked_static_.track_id)) {
                apply_follow_speed_control(return_lead, cur_v, target_speed, should_stop, 4.0, 4.0);
            }

            double near_lat = std::abs(near_path_lateral(command_path, 10.0));
            if (near_lat < 0.30 && cur_v > 1.0) {
                return_finish_count_++;
            } else {
                return_finish_count_ = 0;
            }

            if (return_finish_count_ >= RETURN_FINISH_THRESHOLD) {
                current_mode_ = DriveMode::PURE_PURSUIT;
                tracked_static_.valid = false;
                avoidance_committed_ = false;
                avoid_target_locked_ = false;
                committed_offset_ = 0.0;
                pure_pursuit_.reset();
            }
        }

        if (current_mode_ == DriveMode::OBSTACLE_STOP) {
            should_stop = true;
            target_speed = 0.0;

            if (resume_mode_after_stop_ == DriveMode::AVOID_HOLD && last_valid_command_path_.size() >= 2) {
                command_path = last_valid_command_path_;
            } else {
                command_path = sanitize_command_path(center_path, center_path);
            }

            Intrusion ped_now = detect_pedestrian_intrusion(local_obs, command_path, tracked_static_.track_id);
            Intrusion cutin_now = detect_cutin_intrusion(local_obs, command_path, tracked_static_.track_id);

            if (!ped_now.detected && !cutin_now.detected) {
                dynamic_clear_count_++;
            } else {
                dynamic_clear_count_ = 0;
            }

            if (stop_hold_count_ > 0) {
                stop_hold_count_--;
            }

            if (dynamic_clear_count_ >= DYNAMIC_CLEAR_THRESHOLD && stop_hold_count_ == 0) {
                current_mode_ = resume_mode_after_stop_;
                merge_safe_count_ = 0;
                pure_pursuit_.reset();
            }
        }

        command_path = sanitize_command_path(command_path, center_path);
        publish_path(command_path);
        apply_unsigned_stopline_hold(should_stop, target_speed);

        const bool allow_traffic_stop = traffic_stop_allowed();
        if (allow_traffic_stop) {
            if (!right_turn_ahead) {
                traffic_.evaluate(
                    cur_x,
                    cur_y,
                    effective_stop_line_detected(),
                    current_light_,
                    target_speed,
                    this
                );
            }
        }

        const bool traffic_stop_active =
            allow_traffic_stop &&
            (target_speed <= 0.05 ||
             traffic_.current_state_ == DriveState::STOPPED_AT_SIGNAL ||
             traffic_.current_state_ == DriveState::STOPPED_3_SEC);

        const bool stopline_hazard =
            allow_traffic_stop &&
            effective_stop_line_detected() &&
            (current_light_ == "RED" || current_light_ == "YELLOW");

        const bool use_il_control =
            use_il_normal_control_ &&
            has_fresh_il_control() &&
            current_mode_ == DriveMode::PURE_PURSUIT &&
            !should_stop &&
            !traffic_stop_active &&
            !stopline_hazard;

        carla_msgs::msg::CarlaEgoVehicleControl ctrl;
        if (use_il_control) {
            ctrl = latest_il_control_;
        } else if (should_stop || command_path.size() < 2 || traffic_stop_active || stopline_hazard) {
            ctrl.throttle = 0.0f;
            ctrl.brake = std::clamp(static_cast<float>(0.35 + 0.06 * cur_v), 0.35f, 0.85f);
            ctrl.steer = 0.0f;
        } else {
            const bool force_launch = false;
            pure_pursuit_.calculate_control(cur_v, target_speed, command_path, ctrl, force_launch);
        }

        ctrl.hand_brake = false;
        ctrl.manual_gear_shift = false;
        publish_mode_state(current_mode_name(use_il_control, traffic_stop_active || stopline_hazard));
        control_pub_->publish(ctrl);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MasterControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
