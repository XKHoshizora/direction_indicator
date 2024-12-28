// tts_client.cpp

#include <direction_indicator/tts_client.hpp>
#include <chrono>
#include <thread>

namespace audio_compass {

TTSClient::TTSClient(ros::NodeHandle nh,
                     const std::string& service_name,
                     double timeout_sec)
    : nh_(nh)
    , service_name_(service_name)
    , timeout_sec_(timeout_sec)
    , is_processing_(false)
{
    setlocal(LC_ALL, "");  // 设置为空字符串，避免中文乱码

    startProcessing();
}

TTSClient::~TTSClient()
{
    stopProcessing();
}

bool TTSClient::ensureConnected()
{
    if (!client_)
    {
        try
        {
            bool service_exists = ros::service::waitForService(service_name_, ros::Duration(timeout_sec_));
            if (!service_exists)
            {
                ROS_ERROR_STREAM("TTS service not available: " << service_name_);
                return false;
            }
            client_ = nh_.serviceClient<audio_compass::TextToSpeech>(service_name_);
        }
        catch (const ros::Exception& e)
        {
            ROS_ERROR_STREAM("Failed to connect to TTS service: " << e.what());
            return false;
        }
    }
    return true;
}

bool TTSClient::speak(const std::string& text, const std::string& language)
{
    if (!ensureConnected())
    {
        return false;
    }

    audio_compass::TextToSpeech srv;
    srv.request.text = text;
    srv.request.language = language;

    try
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (client_.call(srv))
        {
            return srv.response.success;
        }
    }
    catch (const ros::Exception& e)
    {
        ROS_ERROR_STREAM("TTS service call failed: " << e.what());
    }
    return false;
}

std::future<bool> TTSClient::speakAsync(const std::string& text,
                                      CallbackFunc callback,
                                      const std::string& language)
{
    auto task = std::make_shared<TTSTask>();
    task->text = text;
    task->language = language;
    task->callback = callback;

    std::future<bool> future = task->promise.get_future();

    {
        std::lock_guard<std::mutex> lock(mutex_);
        task_queue_.push(task);
    }

    return future;
}

void TTSClient::processQueue()
{
    while (is_processing_)
    {
        std::shared_ptr<TTSTask> task;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (task_queue_.empty())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
            task = task_queue_.front();
            task_queue_.pop();
        }

        bool result = speak(task->text, task->language);

        if (task->callback)
        {
            task->callback(result, result ? "Success" : "Failed");
        }
        task->promise.set_value(result);

        // 清理已完成的任务
        {
            std::lock_guard<std::mutex> lock(mutex_);
            active_tasks_.erase(
                std::remove_if(active_tasks_.begin(), active_tasks_.end(),
                    [](std::future<void>& f) {
                        return f.wait_for(std::chrono::seconds(0)) ==
                               std::future_status::ready;
                    }),
                active_tasks_.end());
        }
    }
}

void TTSClient::startProcessing()
{
    is_processing_ = true;
    process_thread_ = std::thread(&TTSClient::processQueue, this);
}

void TTSClient::stopProcessing()
{
    is_processing_ = false;
    if (process_thread_.joinable())
    {
        process_thread_.join();
    }
}

bool TTSClient::waitForAll(double timeout_sec)
{
    auto start_time = std::chrono::steady_clock::now();
    while (isBusy())
    {
        if (timeout_sec > 0)
        {
            auto current_time = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>
                          (current_time - start_time).count();
            if (elapsed >= timeout_sec)
            {
                return false;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return true;
}

bool TTSClient::isBusy() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return !task_queue_.empty() || !active_tasks_.empty();
}

size_t TTSClient::pendingTasks() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return task_queue_.size();
}

void TTSClient::cancelAll()
{
    std::lock_guard<std::mutex> lock(mutex_);
    std::queue<std::shared_ptr<TTSTask>> empty;
    std::swap(task_queue_, empty);
}

} // namespace audio_compass