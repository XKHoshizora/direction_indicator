#pragma once

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>
#include <mutex>

class DirectionCalculator {
public:
    enum class Direction {
        STRAIGHT,
        LEFT,               // 前进中左转
        RIGHT,              // 前进中右转
        ROTATE_LEFT,        // 原地左转
        ROTATE_RIGHT,       // 原地右转
        STOP,
        UNKNOWN
    };

    DirectionCalculator();
    Direction calculateDirection(const nav_msgs::Path& path, double lookAheadDistance);
    bool isNearGoal(const nav_msgs::Path& path, double threshold = 0.5) const;
    bool isRotatingInPlace(const nav_msgs::Path& path) const;

private:
    bool findLookAheadPoint(const nav_msgs::Path& path,
                           double lookAheadDistance,
                           geometry_msgs::Point& point) const;

    std::vector<geometry_msgs::Point> extractFuturePath(
        const nav_msgs::Path& path, double lookAheadDistance) const;

    double calculateMaxCurvature(const std::vector<geometry_msgs::Point>& path) const;
    Direction determineTurnDirection(const std::vector<geometry_msgs::Point>& path) const;
    void cmdVelCallback(const geometry_msgs::TwistConstPtr& msg);

    // 成员变量
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    ros::Subscriber cmd_vel_sub_;
    geometry_msgs::Twist current_vel_;

    // 缓存transform结果
    mutable std::mutex velocity_mutex_;
    mutable geometry_msgs::TransformStamped cached_transform_;
    mutable ros::Time last_transform_time_;
    static constexpr double transform_cache_duration_ = 0.1; // 100ms

    // 配置参数
    double turnThreshold;
    double linear_velocity_threshold_;
    double angular_velocity_threshold_;
    double early_warning_distance_;
};