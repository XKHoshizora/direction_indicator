#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <memory>
#include "direction_indicator/direction_calculator.h"
#include "direction_indicator/state_machine.h"

class DirectionIndicatorNode {
public:
    DirectionIndicatorNode() : nh_("~"), navigation_active_(false) {
        // 从参数服务器获取参数
        nh_.param("/direction_indicator/look_ahead_distance", lookAheadDistance_, 1.5);

        // 初始化订阅者
        pathSub_ = nh_.subscribe("/move_base/GlobalPlanner/plan", 1,
                              &DirectionIndicatorNode::pathCallback, this);
        status_sub_ = nh_.subscribe("/move_base/status", 1,
                                 &DirectionIndicatorNode::moveBaseStatusCallback, this);

        // 初始化组件
        calculator_ = std::make_unique<DirectionCalculator>();
        stateMachine_ = std::make_unique<StateMachine>(nh_);

        ROS_INFO("Direction Indicator Node initialized with look_ahead_distance: %.2f",
                 lookAheadDistance_);
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
    void moveBaseStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg) {
        navigation_active_ = false;
        for (const auto& status : msg->status_list) {
            if (status.status == actionlib_msgs::GoalStatus::ACTIVE) {
                navigation_active_ = true;
                break;
            }
        }
    }

    void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
        // 检查导航状态和路径有效性
        if (!navigation_active_ && !calculator_->isRotatingInPlace(*msg)) {
            return;
        }

        if (msg->poses.empty()) {
            ROS_WARN_THROTTLE(1.0, "Received empty path");
            return;
        }

        try {
            DirectionCalculator::Direction direction;

            // 判断是否接近目标点
            if (calculator_->isNearGoal(*msg)) {
                direction = DirectionCalculator::Direction::STOP;
            } else {
                direction = calculator_->calculateDirection(*msg, lookAheadDistance_);

                if (direction == DirectionCalculator::Direction::UNKNOWN) {
                    ROS_WARN_THROTTLE(1.0, "无法确定方向");
                    return;
                }
            }

            // 更新状态机
            stateMachine_->update(direction);

        } catch (const std::exception& e) {
            ROS_ERROR("Path callback error: %s", e.what());
        }
    }

    ros::NodeHandle nh_;
    ros::Subscriber pathSub_;
    ros::Subscriber status_sub_;
    std::unique_ptr<DirectionCalculator> calculator_;
    std::unique_ptr<StateMachine> stateMachine_;

    double lookAheadDistance_;
    bool navigation_active_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "direction_indicator");
    ROS_INFO("Starting Direction Indicator Node...");

    try {
        DirectionIndicatorNode node;
        ros::spin();
        return 0;
    } catch (const std::exception& e) {
        ROS_ERROR("Direction Indicator Node crashed: %s", e.what());
        return 1;
    }
}