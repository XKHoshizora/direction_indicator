#include "direction_indicator/direction_calculator.h"

DirectionCalculator::DirectionCalculator() : tfListener(tfBuffer) {
    ros::NodeHandle nh("~");

    // 从参数服务器获取配置
    nh.param("/direction_indicator/turn_threshold_degrees", turnThreshold, 20.0);
    turnThreshold = turnThreshold * M_PI / 180.0;

    nh.param("/direction_indicator/linear_velocity_threshold", linear_velocity_threshold_, 0.05);
    nh.param("/direction_indicator/angular_velocity_threshold", angular_velocity_threshold_, 0.1);
    nh.param("/direction_indicator/early_warning_distance", early_warning_distance_, 1.5);

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

    if (path.poses.size() < 3) {
        return Direction::UNKNOWN;
    }

    if (isRotatingInPlace(path)) {
        std::lock_guard<std::mutex> lock(velocity_mutex_);
        return current_vel_.angular.z > 0 ? Direction::ROTATE_LEFT : Direction::ROTATE_RIGHT;
    }

    try {
        // 使用缓存的transform
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

        std::vector<geometry_msgs::Point> future_path = extractFuturePath(path, lookAheadDistance);
        if (future_path.empty()) {
            return Direction::UNKNOWN;
        }

        double max_curvature = calculateMaxCurvature(future_path);
        return (max_curvature < turnThreshold) ? Direction::STRAIGHT : determineTurnDirection(future_path);

    } catch (tf2::TransformException& ex) {
        ROS_WARN_THROTTLE(1.0, "Transform lookup failed: %s", ex.what());
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

    if (path.size() < 3) return 0.0;

    double max_curvature = 0.0;
    std::vector<double> segment_lengths(path.size() - 1);

    // 预计算段长度
    for (size_t i = 0; i < path.size() - 1; ++i) {
        const auto& p1 = path[i];
        const auto& p2 = path[i + 1];
        segment_lengths[i] = std::hypot(p2.x - p1.x, p2.y - p1.y);
    }

    for (size_t i = 1; i < path.size() - 1; ++i) {
        const auto& p1 = path[i - 1];
        const auto& p2 = path[i];
        const auto& p3 = path[i + 1];

        // 使用已计算的段长度
        double d1 = segment_lengths[i - 1];
        double d2 = segment_lengths[i];

        if (d1 < 1e-6 || d2 < 1e-6) continue;

        // 计算曲率
        double cross_product = std::abs((p2.x - p1.x) * (p3.y - p2.y) -
                                      (p2.y - p1.y) * (p3.x - p2.x));
        double curvature = cross_product / (d1 * d2 * (d1 + d2));
        max_curvature = std::max(max_curvature, curvature);
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
