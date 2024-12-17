// include/direction_indicator/direction_calculator.h
#pragma once

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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
    bool isNearGoal(const nav_msgs::Path& path, double threshold = 0.5);
    // 检查机器人是否在原地转向
    bool isRotatingInPlace(const nav_msgs::Path& path, double angular_velocity_threshold = 0.1);

private:
    bool findLookAheadPoint(const nav_msgs::Path& path,
                           double lookAheadDistance,
                           geometry_msgs::Point& point);

    void cmdVelCallback(const geometry_msgs::TwistConstPtr& msg); // 速度回调函数

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    double turnThreshold;  // Angle threshold for turn detection (radians)

    ros::Subscriber cmd_vel_sub_;      // 订阅速度指令
    geometry_msgs::Twist current_vel_; // 当前速度
};