#pragma once

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <memory>
#include "direction_calculator.h"
#include "state_machine.h"

class DirectionIndicatorNode {
public:
    DirectionIndicatorNode();
    ~DirectionIndicatorNode();

private:
    void pathCallback(const nav_msgs::Path::ConstPtr& msg);
    void moveBaseStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg);

    ros::NodeHandle nh;
    ros::Subscriber pathSub_;
    ros::Subscriber status_sub_;
    std::unique_ptr<DirectionCalculator> calculator_;
    std::unique_ptr<StateMachine> stateMachine_;

    double lookAheadDistance_;
    bool navigation_active_;
};