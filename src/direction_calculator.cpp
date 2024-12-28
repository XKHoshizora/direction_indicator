#include "direction_indicator/direction_calculator.h"

DirectionCalculator::DirectionCalculator() : tfListener(tfBuffer) {
    setlocal(LC_ALL, "");  // 设置为空字符串，避免中文乱码

    ros::NodeHandle nh("~");

    // 从参数服务器获取配置
    double turn_threshold_degrees;
    nh.param("/direction_indicator/turn_threshold_degrees", turn_threshold_degrees, 20.0);
    turnThreshold = turn_threshold_degrees * M_PI / 180.0;

    nh.param("/direction_indicator/linear_velocity_threshold", linear_velocity_threshold_, 0.05);
    nh.param("/direction_indicator/angular_velocity_threshold", angular_velocity_threshold_, 0.1);
    nh.param("/direction_indicator/early_warning_distance", early_warning_distance_, 1.5);

    cmd_vel_sub_ = nh.subscribe("/cmd_vel", 1, &DirectionCalculator::cmdVelCallback, this);
}

void DirectionCalculator::cmdVelCallback(const geometry_msgs::TwistConstPtr& msg) {
    current_vel_ = *msg;
}

bool DirectionCalculator::isRotatingInPlace(const nav_msgs::Path& path) {
    bool is_still = std::abs(current_vel_.linear.x) < linear_velocity_threshold_ &&
                   std::abs(current_vel_.linear.y) < linear_velocity_threshold_;
    bool is_rotating = std::abs(current_vel_.angular.z) > angular_velocity_threshold_;

    return is_still && is_rotating;
}

DirectionCalculator::Direction DirectionCalculator::calculateDirection(
    const nav_msgs::Path& path, double lookAheadDistance) {

    // 检查原地转向
    if (isRotatingInPlace(path)) {
        return current_vel_.angular.z > 0 ? Direction::ROTATE_LEFT : Direction::ROTATE_RIGHT;
    }

    if (path.poses.size() < 3) {
        return Direction::UNKNOWN;
    }

    try {
        // 获取机器人当前位姿
        geometry_msgs::TransformStamped transform = tfBuffer.lookupTransform(
            path.header.frame_id, "base_link", ros::Time(0), ros::Duration(1.0));

        // 计算前瞻点
        geometry_msgs::Point lookAheadPoint;
        if (!findLookAheadPoint(path, lookAheadDistance, lookAheadPoint)) {
            return Direction::UNKNOWN;
        }

        // 计算转向角度
        double angle = atan2(lookAheadPoint.y - transform.transform.translation.y,
                           lookAheadPoint.x - transform.transform.translation.x);
        double yaw = tf2::getYaw(transform.transform.rotation);
        double turnAngle = angle - yaw;

        // 角度归一化
        while (turnAngle > M_PI) turnAngle -= 2 * M_PI;
        while (turnAngle < -M_PI) turnAngle += 2 * M_PI;

        // 判断方向
        if (std::abs(turnAngle) < turnThreshold) {
            return Direction::STRAIGHT;
        }
        return (turnAngle > 0) ? Direction::LEFT : Direction::RIGHT;
    }
    catch (tf2::TransformException& ex) {
        ROS_WARN("Transform lookup failed: %s", ex.what());
        return Direction::UNKNOWN;
    }
}

bool DirectionCalculator::findLookAheadPoint(
    const nav_msgs::Path& path,
    double lookAheadDistance,
    geometry_msgs::Point& point) {

    if (path.poses.empty()) {
        return false;
    }

    try {
        geometry_msgs::TransformStamped transform = tfBuffer.lookupTransform(
            path.header.frame_id, "base_link", ros::Time(0), ros::Duration(1.0));

        geometry_msgs::Point robot_pos;
        robot_pos.x = transform.transform.translation.x;
        robot_pos.y = transform.transform.translation.y;
        robot_pos.z = transform.transform.translation.z;

        // 找到最近点
        size_t closest_point_idx = 0;
        double min_distance = std::numeric_limits<double>::max();

        for (size_t i = 0; i < path.poses.size(); i++) {
            const auto& path_point = path.poses[i].pose.position;
            double distance = std::sqrt(
                std::pow(path_point.x - robot_pos.x, 2) +
                std::pow(path_point.y - robot_pos.y, 2));

            if (distance < min_distance) {
                min_distance = distance;
                closest_point_idx = i;
            }
        }

        // 寻找前瞻点
        double accumulated_distance = 0;
        for (size_t i = closest_point_idx; i < path.poses.size() - 1; i++) {
            const auto& p1 = path.poses[i].pose.position;
            const auto& p2 = path.poses[i+1].pose.position;

            double segment_length = std::sqrt(
                std::pow(p2.x - p1.x, 2) +
                std::pow(p2.y - p1.y, 2));

            if (accumulated_distance + segment_length >= lookAheadDistance) {
                // 线性插值
                double remaining = lookAheadDistance - accumulated_distance;
                double ratio = remaining / segment_length;

                point.x = p1.x + (p2.x - p1.x) * ratio;
                point.y = p1.y + (p2.y - p1.y) * ratio;
                point.z = 0;

                return true;
            }

            accumulated_distance += segment_length;
        }

        // 如果找不到合适的前瞻点，使用路径终点
        if (!path.poses.empty()) {
            point = path.poses.back().pose.position;
            return true;
        }

        return false;
    }
    catch (tf2::TransformException& ex) {
        ROS_WARN("Transform lookup failed in findLookAheadPoint: %s", ex.what());
        return false;
    }
}

bool DirectionCalculator::isNearGoal(const nav_msgs::Path& path, double threshold) {
    if (path.poses.empty()) {
        return false;
    }

    try {
        geometry_msgs::TransformStamped transform = tfBuffer.lookupTransform(
            path.header.frame_id, "base_link", ros::Time(0), ros::Duration(1.0));

        const auto& goal = path.poses.back().pose.position;
        double distance = std::sqrt(
            std::pow(goal.x - transform.transform.translation.x, 2) +
            std::pow(goal.y - transform.transform.translation.y, 2));

        return distance < threshold;
    }
    catch (tf2::TransformException& ex) {
        ROS_WARN("Transform lookup failed in isNearGoal: %s", ex.what());
        return false;
    }
}