#pragma once

#include <ros/ros.h>
#include <string>
#include <memory>
#include <std_msgs/String.h>
#include <direction_indicator/tts_client.hpp>
#include "direction_calculator.h"

class StateMachine {
public:
    enum class State {
        STRAIGHT,
        TURN_LEFT_ANTICIPATED,
        TURN_RIGHT_ANTICIPATED,
        TURNING_LEFT_IN_PLACE,
        TURNING_RIGHT_IN_PLACE,
        STOP_ANTICIPATED,
        STOPPED
    };

    explicit StateMachine(ros::NodeHandle& nh);
    ~StateMachine() = default;

    void update(DirectionCalculator::Direction direction);
    State getCurrentState() const { return currentState; }

private:
    void transitionTo(State newState);
    void publishDirection(const std::string& direction);
    void handleTTSCallback(bool success, const std::string& message);

    mutable std::mutex state_mutex_;
    State currentState;
    ros::Publisher directionPub;
    std::unique_ptr<audio_compass::TTSClient> tts_client_;
    ros::Time lastStateChange;
    double stateChangeDelay;
    bool voice_enable_;
};