// src/state_machine.cpp
#include "direction_indicator/state_machine.h"

StateMachine::StateMachine(ros::NodeHandle& nh)
    : currentState(State::STRAIGHT) {

    // 获取参数
    nh.param("/direction_indicator/state_change_delay", stateChangeDelay, 2.0);
    nh.param("/direction_indicator/voice_enable", voice_enable_, true);

    lastStateChange = ros::Time::now();

    // 初始化发布者和服务客户端
    directionPub = nh.advertise<std_msgs::String>("/direction_indicator", 1);
    ttsClient = nh.serviceClient<audio_compass::TextToSpeech>("/text_to_speech");
}

void StateMachine::update(DirectionCalculator::Direction direction) {
    // 防止过于频繁的状态改变
    if ((ros::Time::now() - lastStateChange).toSec() < stateChangeDelay) {
        return;
    }

    switch (direction) {
        case DirectionCalculator::Direction::STRAIGHT:
            if (currentState != State::STRAIGHT) {
                transitionTo(State::STRAIGHT);
            }
            break;

        case DirectionCalculator::Direction::LEFT:
            if (currentState != State::TURN_LEFT_ANTICIPATED) {
                transitionTo(State::TURN_LEFT_ANTICIPATED);
            }
            break;

        case DirectionCalculator::Direction::RIGHT:
            if (currentState != State::TURN_RIGHT_ANTICIPATED) {
                transitionTo(State::TURN_RIGHT_ANTICIPATED);
            }
            break;

        case DirectionCalculator::Direction::ROTATE_LEFT:
            if (currentState != State::TURNING_LEFT_IN_PLACE) {
                transitionTo(State::TURNING_LEFT_IN_PLACE);
            }
            break;

        case DirectionCalculator::Direction::ROTATE_RIGHT:
            if (currentState != State::TURNING_RIGHT_IN_PLACE) {
                transitionTo(State::TURNING_RIGHT_IN_PLACE);
            }
            break;

        case DirectionCalculator::Direction::STOP:
            if (currentState != State::STOP_ANTICIPATED) {
                transitionTo(State::STOP_ANTICIPATED);
            }
            break;

        default:
            break;
    }
}

void StateMachine::transitionTo(State newState) {
    std::string speechText;
    std::string direction;

    switch (newState) {
        case State::STRAIGHT:
            direction = "STRAIGHT";
            speechText = "继续行驶";
            break;

        case State::TURN_LEFT_ANTICIPATED:
            direction = "LEFT";
            speechText = "即将左转";
            break;

        case State::TURN_RIGHT_ANTICIPATED:
            direction = "RIGHT";
            speechText = "即将右转";
            break;

        case State::TURNING_LEFT_IN_PLACE:
            direction = "ROTATE_LEFT";
            speechText = "原地左转";
            break;

        case State::TURNING_RIGHT_IN_PLACE:
            direction = "ROTATE_RIGHT";
            speechText = "原地右转";
            break;

        case State::STOP_ANTICIPATED:
            direction = "STOP";
            speechText = "即将停止";
            break;

        case State::STOPPED:
            direction = "STOPPED";
            speechText = "已到达目的地";
            break;
    }

    publishDirection(direction);
    speakDirectionChange(speechText);

    currentState = newState;
    lastStateChange = ros::Time::now();
}

void StateMachine::speakDirectionChange(const std::string& text) {
    if (!voice_enable_) {
        return;
    }

    audio_compass::TextToSpeech srv;
    srv.request.text = text;

    if (!ttsClient.call(srv)) {
        ROS_ERROR("Failed to call TTS service");
    }
}

void StateMachine::publishDirection(const std::string& direction) {
    std_msgs::String msg;
    msg.data = direction;
    directionPub.publish(msg);
}