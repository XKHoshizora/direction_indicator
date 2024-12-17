// include/direction_indicator/state_machine.h
#pragma once

#include <ros/ros.h>
#include <string>
#include <thread>
#include <mutex>
#include <memory>
#include <audio_compass/TextToSpeech.h>
#include <std_msgs/String.h>
#include "direction_calculator.h"

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
    ~StateMachine() {
        if (tts_thread_ && tts_thread_->joinable()) {
            tts_thread_->join();
        }
    }

    void update(DirectionCalculator::Direction direction);
    State getCurrentState() const { return currentState; }

private:
    void transitionTo(State newState);
    void speakDirectionChange(const std::string& text);
    void publishDirection(const std::string& direction);
    void async_speak(const std::string& text);
    void speak_thread(const std::string& text);

    State currentState;
    ros::Publisher directionPub;
    ros::ServiceClient ttsClient;
    ros::Time lastStateChange;

    // 线程相关成员
    std::unique_ptr<std::thread> tts_thread_;
    std::mutex tts_mutex_;

    double stateChangeDelay;
    bool voice_enable_;
};