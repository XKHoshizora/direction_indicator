#pragma once

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <actionlib_msgs/GoalStatusArray.h>
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

    enum class RobotStatus {
        NAVIGATING,
        ROTATING_TO_GOAL,  // 到达目标点后的最终旋转
        REACHED_GOAL,
        UNKNOWN
    };

    DirectionCalculator();
    Direction calculateDirection(const nav_msgs::Path& path, double lookAheadDistance);
    bool isNearGoal(const nav_msgs::Path& path, double threshold = 0.8);
    bool isRotatingInPlace(const nav_msgs::Path& path);
    RobotStatus getCurrentStatus() const { return current_status_; }

private:
    bool findLookAheadPoint(const nav_msgs::Path& path,
                           double lookAheadDistance,
                           geometry_msgs::Point& point);
    void cmdVelCallback(const geometry_msgs::TwistConstPtr& msg);
    void moveBaseStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber move_base_status_sub_;
    geometry_msgs::Twist current_vel_;
    RobotStatus current_status_;
    actionlib_msgs::GoalStatus::_status_type move_base_status_;

    double turnThreshold;
    double linear_velocity_threshold_;
    double angular_velocity_threshold_;
    double early_warning_distance_;
};