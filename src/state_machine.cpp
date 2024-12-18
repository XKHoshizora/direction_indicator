#include "direction_indicator/state_machine.h"

StateMachine::StateMachine(ros::NodeHandle& nh)
    : currentState(State::STRAIGHT) {

    nh.param("/direction_indicator/state_change_delay", stateChangeDelay, 2.0);
    nh.param("/direction_indicator/voice_enable", voice_enable_, true);

    ROS_INFO("StateMachine initialized with voice_enable_: %s", voice_enable_ ? "true" : "false");

    lastStateChange = ros::Time::now();
    directionPub = nh.advertise<std_msgs::String>("/direction_indicator", 1);
    tts_client_ = std::make_unique<audio_compass::TTSClient>(nh, "/text_to_speech", 10.0);

    ROS_INFO("TTS client created");
}

void StateMachine::handleTTSCallback(bool success, const std::string& message) {
    if (success) {
        ROS_INFO("语音播放成功");
    } else {
        ROS_ERROR("语音播放失败: %s", message.c_str());
    }
}

void StateMachine::transitionTo(State newState) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (currentState == newState) {
        return;
    }

    std::string direction;
    std::string speechText;

    switch (newState) {
        case State::STRAIGHT:
            direction = "STRAIGHT";
            speechText = "直行";
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
            speechText = "左转";
            break;

        case State::TURNING_RIGHT_IN_PLACE:
            direction = "ROTATE_RIGHT";
            speechText = "右转";
            break;

        case State::STOP_ANTICIPATED:
            direction = "STOP";
            speechText = "准备停止";
            break;

        case State::STOPPED:
            direction = "STOPPED";
            speechText = "已到达目的地";
            break;
    }

    ROS_INFO("=== State Transition ===");
    ROS_INFO("From: %d, To: %d", static_cast<int>(currentState), static_cast<int>(newState));
    ROS_INFO("Direction: %s", direction.c_str());
    ROS_INFO("Speech Text: %s", speechText.c_str());
    ROS_INFO("Voice Enable: %s", voice_enable_ ? "true" : "false");
    ROS_INFO("TTS Client Valid: %s", tts_client_ ? "yes" : "no");

    // 发布方向
    publishDirection(direction);

    // 语音提示
    if (voice_enable_) {
        if (tts_client_) {
            ROS_INFO("Attempting to speak: %s", speechText.c_str());
            tts_client_->speakAsync(
                speechText,
                [this, speechText](bool success, const std::string& message) {
                    ROS_INFO("TTS Callback - Text: '%s', Success: %s, Message: %s",
                            speechText.c_str(),
                            success ? "true" : "false",
                            message.c_str());
                }
            );
        } else {
            ROS_ERROR("TTS client is null!");
        }
    } else {
        ROS_INFO("Voice is disabled, skipping speech");
    }

    // 更新状态
    currentState = newState;
    lastStateChange = ros::Time::now();
}

void StateMachine::publishDirection(const std::string& direction) {
    std_msgs::String msg;
    msg.data = direction;
    directionPub.publish(msg);
}

void StateMachine::update(DirectionCalculator::Direction direction) {
    std::lock_guard<std::mutex> lock(state_mutex_);

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
            // 未知状态不进行转换
            break;
    }
}