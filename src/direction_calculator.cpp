#include "direction_indicator/direction_calculator.h"

DirectionCalculator::DirectionCalculator() : tfListener(tfBuffer) {
    ros::NodeHandle nh("~");

    // 调整默认阈值，使其更容易触发转向状态
    double default_threshold = 0.2;  // 降低阈值，使其更容易检测到转向
    nh.param("/direction_indicator/turn_threshold_degrees", turnThreshold, default_threshold);
    turnThreshold = turnThreshold * M_PI / 180.0;

    // 获取其他参数
    nh.param("/direction_indicator/linear_velocity_threshold", linear_velocity_threshold_, 0.05);
    nh.param("/direction_indicator/angular_velocity_threshold", angular_velocity_threshold_, 0.1);
    nh.param("/direction_indicator/early_warning_distance", early_warning_distance_, 1.5);

    // 输出初始化参数
    ROS_INFO("DirectionCalculator initialized with:");
    ROS_INFO("- Turn threshold: %.2f rad (%.2f degrees)", turnThreshold, turnThreshold * 180.0 / M_PI);
    ROS_INFO("- Linear velocity threshold: %.2f m/s", linear_velocity_threshold_);
    ROS_INFO("- Angular velocity threshold: %.2f rad/s", angular_velocity_threshold_);
    ROS_INFO("- Early warning distance: %.2f m", early_warning_distance_);

    cmd_vel_sub_ = nh.subscribe("/cmd_vel", 1, &DirectionCalculator::cmdVelCallback, this);
}


void DirectionCalculator::cmdVelCallback(const geometry_msgs::TwistConstPtr& msg) {
    std::lock_guard<std::mutex> lock(velocity_mutex_);
    current_vel_ = *msg;
}

bool DirectionCalculator::isRotatingInPlace(const nav_msgs::Path& path) const {
    std::lock_guard<std::mutex> lock(velocity_mutex_);
    bool is_still = std::abs(current_vel_.linear.x) < linear_velocity_threshold_ &&
                   std::abs(current_vel_.linear.y) < linear_velocity_threshold_;
    bool is_rotating = std::abs(current_vel_.angular.z) > angular_velocity_threshold_;
    return is_still && is_rotating;
}

DirectionCalculator::Direction DirectionCalculator::calculateDirection(
    const nav_msgs::Path& path, double lookAheadDistance) {

    if (path.poses.size() < 2) {
        ROS_DEBUG("Path too short");
        return Direction::UNKNOWN;
    }

    // 检查是否在原地旋转
    if (isRotatingInPlace(path)) {
        std::lock_guard<std::mutex> lock(velocity_mutex_);
        if (std::abs(current_vel_.angular.z) > angular_velocity_threshold_) {
            ROS_INFO_THROTTLE(1.0, "Rotating in place: %.2f rad/s", current_vel_.angular.z);
            return current_vel_.angular.z > 0 ? Direction::ROTATE_LEFT : Direction::ROTATE_RIGHT;
        }
    }

    try {
        // 获取未来路径点
        std::vector<geometry_msgs::Point> future_path = extractFuturePath(path, lookAheadDistance);
        if (future_path.empty()) {
            ROS_DEBUG("No future path points extracted");
            return Direction::UNKNOWN;
        }

        // 计算曲率
        double max_curvature = calculateMaxCurvature(future_path);
        ROS_INFO_THROTTLE(1.0, "Current max curvature: %.2f (threshold: %.2f)",
                         max_curvature, turnThreshold);

        // 调整曲率阈值判断
        if (max_curvature < turnThreshold) {
            ROS_DEBUG_THROTTLE(1.0, "Moving straight (curvature below threshold)");
            return Direction::STRAIGHT;
        } else {
            // 确定转向方向
            Direction turn_direction = determineTurnDirection(future_path);
            ROS_INFO_THROTTLE(1.0, "Turn detected: %s",
                            turn_direction == Direction::LEFT ? "LEFT" : "RIGHT");
            return turn_direction;
        }

    } catch (tf2::TransformException& ex) {
        ROS_WARN("Transform lookup failed: %s", ex.what());
        return Direction::UNKNOWN;
    }
}

std::vector<geometry_msgs::Point> DirectionCalculator::extractFuturePath(
    const nav_msgs::Path& path, double lookAheadDistance) const {
    std::vector<geometry_msgs::Point> future_path;

    if (path.poses.empty()) {
        return future_path;
    }

    try {
        geometry_msgs::TransformStamped transform = tfBuffer.lookupTransform(
            path.header.frame_id, "base_link", ros::Time(0), ros::Duration(1.0));

        geometry_msgs::Point robot_pos;
        robot_pos.x = transform.transform.translation.x;
        robot_pos.y = transform.transform.translation.y;
        robot_pos.z = transform.transform.translation.z;

        double accumulated_distance = 0;
        for (size_t i = 0; i < path.poses.size() - 1; ++i) {
            const auto& p1 = path.poses[i].pose.position;
            const auto& p2 = path.poses[i + 1].pose.position;

            double segment_length = std::sqrt(
                std::pow(p2.x - p1.x, 2) +
                std::pow(p2.y - p1.y, 2));

            accumulated_distance += segment_length;
            if (accumulated_distance <= lookAheadDistance) {
                future_path.push_back(p2);
            } else {
                break;
            }
        }
    } catch (tf2::TransformException& ex) {
        ROS_WARN("Transform lookup failed in extractFuturePath: %s", ex.what());
    }

    return future_path;
}

double DirectionCalculator::calculateMaxCurvature(
    const std::vector<geometry_msgs::Point>& path) const {

    if (path.size() < 3) {
        ROS_DEBUG("Path too short for curvature calculation");
        return 0.0;
    }

    double max_curvature = 0.0;

    // 用于调试的点记录
    std::vector<double> curvatures;

    for (size_t i = 1; i < path.size() - 1; ++i) {
        const auto& p1 = path[i - 1];
        const auto& p2 = path[i];
        const auto& p3 = path[i + 1];

        // 计算两个向量
        double dx1 = p2.x - p1.x;
        double dy1 = p2.y - p1.y;
        double dx2 = p3.x - p2.x;
        double dy2 = p3.y - p2.y;

        // 计算向量长度
        double l1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
        double l2 = std::sqrt(dx2 * dx2 + dy2 * dy2);

        // 防止除零
        if (l1 < 1e-6 || l2 < 1e-6) {
            continue;
        }

        // 计算角度变化（弧度）
        double angle_change = std::atan2(dy2, dx2) - std::atan2(dy1, dx1);

        // 标准化角度到 [-π, π]
        while (angle_change > M_PI) angle_change -= 2 * M_PI;
        while (angle_change < -M_PI) angle_change += 2 * M_PI;

        // 计算曲率（角度变化/路径长度）
        double curvature = std::abs(angle_change) / ((l1 + l2) / 2.0);
        curvatures.push_back(curvature);
        max_curvature = std::max(max_curvature, curvature);

        // 添加调试信息
        ROS_DEBUG("Point %zu: angle_change=%.2f, curvature=%.2f",
                  i, angle_change, curvature);
    }

    // 输出曲率统计信息
    if (!curvatures.empty()) {
        double avg_curvature = 0.0;
        for (double c : curvatures) {
            avg_curvature += c;
        }
        avg_curvature /= curvatures.size();

        ROS_DEBUG("Curvature stats - Max: %.2f, Avg: %.2f, Threshold: %.2f",
                  max_curvature, avg_curvature, turnThreshold);
    }

    return max_curvature;
}

DirectionCalculator::Direction DirectionCalculator::determineTurnDirection(
    const std::vector<geometry_msgs::Point>& path) const {
    if (path.size() < 2) {
        return Direction::STRAIGHT;
    }

    const auto& start = path.front();
    const auto& end = path.back();

    double angle = atan2(end.y - start.y, end.x - start.x);
    if (angle > 0) {
        return Direction::LEFT;
    } else {
        return Direction::RIGHT;
    }
}

bool DirectionCalculator::findLookAheadPoint(
    const nav_msgs::Path& path,
    double lookAheadDistance,
    geometry_msgs::Point& point) const {

    if (path.poses.empty()) {
        return false;
    }

    try {
        geometry_msgs::TransformStamped transform;
        {
            ros::Time now = ros::Time::now();
            if ((now - last_transform_time_).toSec() > transform_cache_duration_) {
                transform = tfBuffer.lookupTransform(
                    path.header.frame_id, "base_link", ros::Time(0), ros::Duration(1.0));
                cached_transform_ = transform;
                last_transform_time_ = now;
            } else {
                transform = cached_transform_;
            }
        }

        geometry_msgs::Point robot_pos;
        robot_pos.x = transform.transform.translation.x;
        robot_pos.y = transform.transform.translation.y;

        double accumulated_distance = 0.0;
        for (size_t i = 0; i < path.poses.size() - 1; ++i) {
            const auto& p1 = path.poses[i].pose.position;
            const auto& p2 = path.poses[i + 1].pose.position;

            double segment_length = std::hypot(p2.x - p1.x, p2.y - p1.y);
            accumulated_distance += segment_length;

            if (accumulated_distance >= lookAheadDistance) {
                // 计算插值点
                double overshoot = accumulated_distance - lookAheadDistance;
                double ratio = (segment_length - overshoot) / segment_length;

                point.x = p1.x + (p2.x - p1.x) * ratio;
                point.y = p1.y + (p2.y - p1.y) * ratio;
                point.z = p1.z + (p2.z - p1.z) * ratio;
                return true;
            }
        }

        // 如果没找到合适的点，返回路径终点
        if (!path.poses.empty()) {
            point = path.poses.back().pose.position;
            return true;
        }

    } catch (tf2::TransformException& ex) {
        ROS_WARN_THROTTLE(1.0, "Transform lookup failed in findLookAheadPoint: %s", ex.what());
    }

    return false;
}

bool DirectionCalculator::isNearGoal(
    const nav_msgs::Path& path,
    double threshold) const {

    if (path.poses.empty()) {
        return false;
    }

    try {
        geometry_msgs::TransformStamped transform;
        {
            ros::Time now = ros::Time::now();
            if ((now - last_transform_time_).toSec() > transform_cache_duration_) {
                transform = tfBuffer.lookupTransform(
                    path.header.frame_id, "base_link", ros::Time(0), ros::Duration(1.0));
                cached_transform_ = transform;
                last_transform_time_ = now;
            } else {
                transform = cached_transform_;
            }
        }

        const auto& goal = path.poses.back().pose.position;
        const auto& robot_pos = transform.transform.translation;

        double distance = std::hypot(
            goal.x - robot_pos.x,
            goal.y - robot_pos.y
        );

        return distance <= threshold;

    } catch (tf2::TransformException& ex) {
        ROS_WARN_THROTTLE(1.0, "Transform lookup failed in isNearGoal: %s", ex.what());
        return false;
    }
}
