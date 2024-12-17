// src/direction_indicator_node.cpp

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include "direction_indicator/direction_calculator.h"
#include "direction_indicator/state_machine.h"

class DirectionIndicatorNode {
public:
    DirectionIndicatorNode() : nh_("~") {
        // 从参数服务器获取参数
        double look_ahead_distance;
        nh_.param("look_ahead_distance", lookAheadDistance_, 1.5);  // 前瞻距离，默认1.5米

        // 订阅全局路径规划
        pathSub_ = nh_.subscribe("/move_base/NavfnROS/plan", 1,
                               &DirectionIndicatorNode::pathCallback, this);

        // 导航状态监控
        status_sub_ = nh_.subscribe("/move_base/status", 1,
                                  &DirectionIndicatorNode::moveBaseStatusCallback, this);

        // 初始化各个组件
        calculator_ = std::make_unique<DirectionCalculator>();
        stateMachine_ = std::make_unique<StateMachine>(nh_);

        ROS_INFO("Direction Indicator Node initialized with look_ahead_distance: %.2f", lookAheadDistance_);
    }

    ~DirectionIndicatorNode() {
        if (pathSub_) {
            pathSub_.shutdown();
        }
        if (status_sub_) {
            status_sub_.shutdown();
        }
    }

private:
    // 导航状态监控
    bool navigation_active_ = false;
    ros::Subscriber status_sub_;

    void moveBaseStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg) {
        // 检查是否有活动的导航目标
        navigation_active_ = false;
        for (const auto& status : msg->status_list) {
            if (status.status == actionlib_msgs::GoalStatus::ACTIVE) {
                navigation_active_ = true;
                break;
            }
        }
    }

    void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
        // 如果导航未激活，且不是原地转向，直接返回
        if (!navigation_active_ && !calculator_.isRotatingInPlace(*msg)) {
            return;
        }

        // 检查路径是否有效
        if (msg->poses.empty()) {
            ROS_WARN_THROTTLE(1.0, "Received empty path");
            return;
        }

        try {
            DirectionCalculator::Direction direction;

            // 检查是否接近目标点
            if (calculator_.isNearGoal(*msg)) {
                ROS_DEBUG("Near goal, preparing to stop");
                direction = DirectionCalculator::Direction::STOP;
            } else {
                // 计算前进方向
                direction = calculator_.calculateDirection(*msg, lookAheadDistance_);

                if (direction == DirectionCalculator::Direction::UNKNOWN) {
                    ROS_WARN_THROTTLE(1.0, "Unable to determine direction");
                    return;
                }
            }

            // 更新状态机，触发方向提示和语音播报
            stateMachine_->update(direction);

        } catch (const std::exception& e) {
            ROS_ERROR("Error in path callback: %s", e.what());
        }
    }

private:
    ros::NodeHandle nh_;                               // ROS节点句柄
    ros::Subscriber pathSub_;                          // 路径订阅者
    std::unique_ptr<DirectionCalculator> calculator_;  // 方向计算器
    std::unique_ptr<StateMachine> stateMachine_;       // 状态机
    double lookAheadDistance_;                         // 前瞻距离参数
};

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "direction_indicator");
    ROS_INFO("Starting Direction Indicator Node...");

    try {
        // 创建并运行节点
        DirectionIndicatorNode node;

        // 进入ROS主循环
        ros::spin();

        return 0;
    }
    catch (const std::exception& e) {
        ROS_ERROR("Direction Indicator Node crashed: %s", e.what());
        return 1;
    }
}