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
    if (currentState == newState) {
        return;
    }

    std::string direction;
    std::string speechText;

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
            speechText = "开始左转";
            break;

        case State::TURNING_RIGHT_IN_PLACE:
            direction = "ROTATE_RIGHT";
            speechText = "开始右转";
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

    ROS_INFO("状态转换: %s, 播放语音: %s", direction.c_str(), speechText.c_str());

    // 发布方向
    publishDirection(direction);

    // 异步播放语音
    async_speak(speechText);

    // 更新状态
    currentState = newState;
    lastStateChange = ros::Time::now();
}


    // 异步调用tts服务
void StateMachine::async_speak(const std::string& text) {
    // 如果上一个语音还在播放，等待完成
    if (tts_thread_ && tts_thread_->joinable()) {
        tts_thread_->join();
    }

    // 创建新的语音播放线程
    tts_thread_ = std::make_unique<std::thread>(
        [this, text]() {
            std::lock_guard<std::mutex> lock(tts_mutex_);
            this->speak_thread(text);
        }
    );
}

    // tts服务的回调函数
void StateMachine::speak_thread(const std::string& text) {
    if (!voice_enable_) {
        ROS_DEBUG("Voice prompts are disabled");
        return;
    }

    try {
        audio_compass::TextToSpeech srv;
        srv.request.text = text;
        srv.request.language = "zh";

        ROS_INFO("Trying to speak: %s", text.c_str());
        if (ttsClient.call(srv)) {
            if (srv.response.success) {
                ROS_INFO("语音合成成功: %s", text.c_str());
            } else {
                ROS_ERROR("语音合成失败: %s", srv.response.message.c_str());
            }
        } else {
            ROS_ERROR("TTS服务调用失败");
        }
    } catch (const ros::Exception& e) {
        ROS_ERROR("TTS服务异常: %s", e.what());
    }
}

void StateMachine::speakDirectionChange(const std::string& text) {
    if (!voice_enable_) {
        ROS_DEBUG("Voice prompts are disabled");
        return;
    }

    audio_compass::TextToSpeech srv;
    srv.request.text = text;

    if (!ttsClient.call(srv)) {
        ROS_ERROR("Failed to call TTS service for text: %s", text.c_str());
    } else {
        ROS_INFO("Speaking: %s", text.c_str());
    }
}

void StateMachine::publishDirection(const std::string& direction) {
    std_msgs::String msg;
    msg.data = direction;
    directionPub.publish(msg);
}