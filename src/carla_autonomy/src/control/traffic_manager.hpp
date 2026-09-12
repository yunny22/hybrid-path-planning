#pragma once
#include <string>
#include <vector>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

enum class DriveState { NORMAL_DRIVING, STOPPED_AT_SIGNAL, STOPPED_3_SEC };
struct VirtualStopLine { double x, y, z, yaw_deg; };

class TrafficManager {
public:
    DriveState current_state_ = DriveState::NORMAL_DRIVING;
    rclcpp::Time intersection_pass_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time stop_start_time_{0, 0, RCL_ROS_TIME};
    
    // 인식이 잘 안 되는 특정 2곳의 가상 정지선 좌표
    std::vector<VirtualStopLine> virtual_stop_lines_ = { {359.2042, 172.1120, 0.1959, -180.3261}, {192.1718, 245.1355, 0.0387, -358.4047} };

    // 가상 정지선 7m 이내인지 확인
    bool is_near_virtual_stop_line(double x, double y) {
        for (const auto& vsl : virtual_stop_lines_) {
            if (std::hypot(vsl.x - x, vsl.y - y) < 7.0) return true;
        }
        return false;
    }

    void evaluate(double current_x, double current_y, bool stop_line_detected, const std::string& current_light, double& target_speed, rclcpp::Node* node) {
        
        // 🌟 [오류 해결] 
        // 카메라가 실제 정지선을 인식했거나 OR 가상 정지선 7m 이내에 들어왔거나
        // 둘 중 하나만 참이어도 정지 구역으로 판단합니다.
        bool near_stop = stop_line_detected || is_near_virtual_stop_line(current_x, current_y);
        
        // 교차로를 갓 통과한 직후 5초간은 바닥의 노이즈를 무시
        if ((node->now() - intersection_pass_time_).seconds() < 5.0) near_stop = false;

        if (target_speed > 0.0) {
            if (current_state_ == DriveState::NORMAL_DRIVING) {
                if (near_stop) {
                    if (current_light == "RED" || current_light == "YELLOW") {
                        current_state_ = DriveState::STOPPED_AT_SIGNAL;
                        stop_start_time_ = node->now();
                    }
                }
            } else if (current_state_ == DriveState::STOPPED_AT_SIGNAL) {
                target_speed = 0.0;
                // 🌟 신호등이 초록불이 되거나, 
                // 정지선을 너무 넘어서 신호등이 안 보이는(UNKNOWN) 상태로 7초가 지나면 갇히지 않고 강제 출발!
                if (current_light == "GREEN" || (current_light == "UNKNOWN" && (node->now() - stop_start_time_).seconds() > 7.0)) { 
                    current_state_ = DriveState::NORMAL_DRIVING; 
                    intersection_pass_time_ = node->now(); 
                }
            } else if (current_state_ == DriveState::STOPPED_3_SEC) {
                target_speed = 0.0;
                if ((node->now() - stop_start_time_).seconds() >= 3.0) { 
                    current_state_ = DriveState::NORMAL_DRIVING; 
                    intersection_pass_time_ = node->now(); 
                }
            }
        }
    }
};
