// include/direction_indicator/state_machine.h
#pragma once

#include <ros/ros.h>
#include <string>
#include "direction_calculator.h"
#include <audio_compass/TextToSpeech.h>
#include <std_msgs/String.h>

class StateMachine {
public:
    enum class State {
        STRAIGHT,
        TURN_LEFT_ANTICIPATED,    // 前进过程中的左转
        TURN_RIGHT_ANTICIPATED,   // 前进过程中的右转
        TURNING_LEFT_IN_PLACE,    // 原地左转
        TURNING_RIGHT_IN_PLACE,   // 原地右转
        STOP_ANTICIPATED,
        STOPPED
    };

    StateMachine(ros::NodeHandle& nh);
    void update(DirectionCalculator::Direction direction);
    State getCurrentState() const { return currentState; }

private:
    void transitionTo(State newState);
    void speakDirectionChange(const std::string& text);
    void publishDirection(const std::string& direction);

    State currentState;
    ros::Publisher directionPub;
    ros::ServiceClient ttsClient;
    ros::Time lastStateChange;
    double stateChangeDelay;
    bool voice_enable_;
};