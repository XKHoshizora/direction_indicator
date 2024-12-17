#ifndef TTS_CLIENT_HPP
#define TTS_CLIENT_HPP

#include <ros/ros.h>
#include <audio_compass/TextToSpeech.h>
#include <future>
#include <mutex>
#include <queue>
#include <memory>
#include <functional>

namespace audio_compass {

/**
 * @brief TTS客户端封装类
 *
 * 提供同步和异步的文本转语音功能，支持多语言和回调函数。
 * 线程安全，支持并发调用。
 */
class TTSClient {
public:
    using CallbackFunc = std::function<void(bool success, const std::string& message)>;

    /**
     * @brief 构造函数
     * @param nh ROS节点句柄
     * @param service_name TTS服务名称
     * @param timeout_sec 服务连接超时时间(秒)
     */
    explicit TTSClient(ros::NodeHandle nh,
                      const std::string& service_name = "/text_to_speech",
                      double timeout_sec = 10.0);

    /**
     * @brief 析构函数
     * 等待所有异步任务完成并清理资源
     */
    ~TTSClient();

    /**
     * @brief 同步执行语音合成
     * @param text 要转换的文本
     * @param language 语言代码(可选)
     * @return 是否成功
     */
    bool speak(const std::string& text, const std::string& language = "");

    /**
     * @brief 异步执行语音合成
     * @param text 要转换的文本
     * @param callback 完成回调函数(可选)
     * @param language 语言代码(可选)
     * @return std::future<bool> 用于获取结果的future对象
     */
    std::future<bool> speakAsync(const std::string& text,
                                CallbackFunc callback = nullptr,
                                const std::string& language = "");

    /**
     * @brief 等待所有异步任务完成
     * @param timeout_sec 超时时间(秒)，默认为0表示一直等待
     * @return 是否所有任务都完成
     */
    bool waitForAll(double timeout_sec = 0.0);

    /**
     * @brief 检查是否有正在进行的语音合成
     * @return 是否忙碌
     */
    bool isBusy() const;

    /**
     * @brief 获取队列中的任务数量
     * @return 任务数量
     */
    size_t pendingTasks() const;

    /**
     * @brief 取消所有待处理的任务
     */
    void cancelAll();

private:
    // 禁用拷贝和赋值
    TTSClient(const TTSClient&) = delete;
    TTSClient& operator=(const TTSClient&) = delete;

    /**
     * @brief 确保已连接到服务
     * @return 是否连接成功
     */
    bool ensureConnected();

    /**
     * @brief 异步任务结构体
     */
    struct TTSTask {
        std::string text;
        std::string language;
        CallbackFunc callback;
        std::promise<bool> promise;
    };

    ros::NodeHandle nh_;
    ros::ServiceClient client_;
    std::string service_name_;
    double timeout_sec_;

    mutable std::mutex mutex_;
    std::queue<std::shared_ptr<TTSTask>> task_queue_;
    std::vector<std::future<void>> active_tasks_;
    bool is_processing_;
    std::thread process_thread_;

    void processQueue();
    void startProcessing();
    void stopProcessing();
};

} // namespace audio_compass

#endif // TTS_CLIENT_HPP