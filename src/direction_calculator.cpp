// src/direction_calculator.cpp
#include "direction_indicator/direction_calculator.h"
#include <cmath>

DirectionCalculator::DirectionCalculator() : tfListener(tfBuffer) {
    ros::NodeHandle nh("~");

    // 从参数服务器获取配置
    double turn_threshold_degrees;
    nh.param("/direction_indicator/turn_threshold_degrees", turn_threshold_degrees, 20.0);
    turnThreshold = turn_threshold_degrees * M_PI / 180.0;  // 转换为弧度

    // 获取速度阈值
    nh.param("/direction_indicator/linear_velocity_threshold", linear_velocity_threshold_, 0.05);
    nh.param("/direction_indicator/angular_velocity_threshold", angular_velocity_threshold_, 0.1);

    // 获取预警距离
    nh.param("/direction_indicator/early_warning_distance", early_warning_distance_, 1.5);

    // 初始化速度订阅
    cmd_vel_sub_ = nh.subscribe("/cmd_vel", 1, &DirectionCalculator::cmdVelCallback, this);
}

void DirectionCalculator::cmdVelCallback(const geometry_msgs::TwistConstPtr& msg) {
    current_vel_ = *msg;
}

bool DirectionCalculator::isRotatingInPlace(const nav_msgs::Path& path, double angular_velocity_threshold) {
    if (!velocity_initialized_) {
        return false;
    }

    // 检查线速度是否接近0且角速度大于阈值
    bool is_still = std::abs(current_vel_.linear.x) < linear_velocity_threshold_ &&
                   std::abs(current_vel_.linear.y) < linear_velocity_threshold_;
    bool is_rotating = std::abs(current_vel_.angular.z) > angular_velocity_threshold_;

    return is_still && is_rotating;
}

DirectionCalculator::Direction DirectionCalculator::calculateDirection(const nav_msgs::Path& path, double lookAheadDistance) {
    // 首先检查原地转向
    if (isRotatingInPlace(path)) {
        // 根据角速度方向判断转向
        if (current_vel_.angular.z > 0) {
            return Direction::LEFT;
        } else if (current_vel_.angular.z < 0) {
            return Direction::RIGHT;
        }
    }

    if (path.poses.size() < 3) {
        return Direction::UNKNOWN;
    }

    try {
        // 获取机器人当前位姿(在路径坐标系中)
        geometry_msgs::TransformStamped transform =
            tfBuffer.lookupTransform(path.header.frame_id, "base_link",
                                   ros::Time(0), ros::Duration(1.0));

        // 计算前瞻点
        geometry_msgs::Point lookAheadPoint;
        if (!findLookAheadPoint(path, lookAheadDistance, lookAheadPoint)) {
            return Direction::UNKNOWN;
        }

        // 计算机器人当前位置到前瞻点的角度
        double angle = atan2(lookAheadPoint.y - transform.transform.translation.y,
                           lookAheadPoint.x - transform.transform.translation.x);

        // 获取机器人当前朝向
        double yaw = tf2::getYaw(transform.transform.rotation);

        // 计算转向角度(相对于机器人当前朝向)
        double turnAngle = angle - yaw;

        // 将角度归一化到 [-π, π] 范围内
        while (turnAngle > M_PI) turnAngle -= 2 * M_PI;
        while (turnAngle < -M_PI) turnAngle += 2 * M_PI;

        // 根据转向角度判断方向
        if (std::abs(turnAngle) < turnThreshold) {
            return Direction::STRAIGHT;
        } else if (turnAngle > 0) {
            return Direction::LEFT;
        } else {
            return Direction::RIGHT;
        }
    }
    catch (tf2::TransformException& ex) {
        ROS_WARN("Transform lookup failed: %s", ex.what());
        return Direction::UNKNOWN;
    }
}

bool DirectionCalculator::findLookAheadPoint(const nav_msgs::Path& path,
                                           double lookAheadDistance,
                                           geometry_msgs::Point& point) {
    if (path.poses.empty()) {
        return false;
    }

    // 获取机器人当前位置
    geometry_msgs::TransformStamped transform;
    try {
        transform = tfBuffer.lookupTransform(path.header.frame_id, "base_link",
                                           ros::Time(0), ros::Duration(1.0));
    }
    catch (tf2::TransformException& ex) {
        ROS_WARN("Transform lookup failed: %s", ex.what());
        return false;
    }

    // 将 Vector3 转换为 Point
    geometry_msgs::Point robot_pos;
    robot_pos.x = transform.transform.translation.x;
    robot_pos.y = transform.transform.translation.y;
    robot_pos.z = transform.transform.translation.z;

    // 找到路径上距离机器人最近的点
    size_t closest_point_idx = 0;
    double min_distance = std::numeric_limits<double>::max();

    for (size_t i = 0; i < path.poses.size(); i++) {
        const auto& path_point = path.poses[i].pose.position;
        double dx = path_point.x - robot_pos.x;
        double dy = path_point.y - robot_pos.y;
        double distance = std::sqrt(dx*dx + dy*dy);

        if (distance < min_distance) {
            min_distance = distance;
            closest_point_idx = i;
        }
    }

    // 从最近点开始,找到前瞻距离的点
    double accumulated_distance = 0;
    for (size_t i = closest_point_idx; i < path.poses.size() - 1; i++) {
        const auto& p1 = path.poses[i].pose.position;
        const auto& p2 = path.poses[i+1].pose.position;

        double segment_length = std::sqrt(
            std::pow(p2.x - p1.x, 2) +
            std::pow(p2.y - p1.y, 2)
        );

        if (accumulated_distance + segment_length >= lookAheadDistance) {
            // 在这个线段上插值得到精确的前瞻点
            double remaining = lookAheadDistance - accumulated_distance;
            double ratio = remaining / segment_length;

            point.x = p1.x + (p2.x - p1.x) * ratio;
            point.y = p1.y + (p2.y - p1.y) * ratio;
            point.z = 0;

            return true;
        }

        accumulated_distance += segment_length;
    }

    // 如果找不到合适的前瞻点,使用路径终点
    if (!path.poses.empty()) {
        point = path.poses.back().pose.position;
        return true;
    }

    return false;
}

bool DirectionCalculator::isNearGoal(const nav_msgs::Path& path, double threshold) {
    if (path.poses.empty()) {
        return false;
    }

    try {
        // 获取机器人当前位姿
        geometry_msgs::TransformStamped transform =
            tfBuffer.lookupTransform(path.header.frame_id, "base_link",
                                   ros::Time(0), ros::Duration(1.0));

        // 计算到终点的距离
        const auto& goal = path.poses.back().pose.position;
        double dx = goal.x - transform.transform.translation.x;
        double dy = goal.y - transform.transform.translation.y;
        double distance = std::sqrt(dx*dx + dy*dy);

        return distance < threshold;
    }
    catch (tf2::TransformException& ex) {
        ROS_WARN("Transform lookup failed: %s", ex.what());
        return false;
    }
}