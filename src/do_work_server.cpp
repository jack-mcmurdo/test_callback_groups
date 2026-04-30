#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
#include "test_callback_groups/action/do_work.hpp"
#include <thread>
#include <memory>
#include <atomic>
#include <chrono>

using namespace std::chrono_literals;
using DoWork = test_callback_groups::action::DoWork;
using GoalHandleDoWork = rclcpp_action::ServerGoalHandle<DoWork>;

class DoWorkServer : public rclcpp::Node
{
public:
    DoWorkServer() : Node("do_work_server")
    {
        // Initialize atomic flags
        pause_ = false;

        // Create callback groups
        action_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        subscription_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        diagnostics_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        // Create action server with callback group
        action_server_ = rclcpp_action::create_server<DoWork>(
            this,
            "do_work",
            std::bind(&DoWorkServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&DoWorkServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&DoWorkServer::handle_accepted, this, std::placeholders::_1),
            rcl_action_server_get_default_options(),
            action_callback_group_);

        // Create publisher for work output
        work_publisher_ = this->create_publisher<std_msgs::msg::String>("work_output", 10);

        // Create subscription for pause work with callback group
        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = subscription_callback_group_;
        pause_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
            "pause_work", 10,
            std::bind(&DoWorkServer::pause_callback, this, std::placeholders::_1),
            sub_options);

        // Setup diagnostics updater
        diagnostics_updater_ = std::make_unique<diagnostic_updater::Updater>(this);
        diagnostics_updater_->setHardwareID("do_work_server");
        diagnostics_updater_->add("work_status", this, &DoWorkServer::diagnostics_callback);

        // Start diagnostics timer with callback group (1Hz)
        diagnostics_timer_ = this->create_wall_timer(
            1s, std::bind(&DoWorkServer::update_diagnostics, this),
            diagnostics_callback_group_);

        RCLCPP_INFO(this->get_logger(), "DoWork server started");
    }

private:
    // Action server
    rclcpp_action::Server<DoWork>::SharedPtr action_server_;

    // Publishers and subscriptions
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr work_publisher_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pause_subscription_;

    // Thread management
    std::shared_ptr<std::thread> work_thread_;

    // State variables
    std::atomic<bool> pause_;
    bool previous_pause_value_ = false;

    // Diagnostics
    std::unique_ptr<diagnostic_updater::Updater> diagnostics_updater_;
    rclcpp::TimerBase::SharedPtr diagnostics_timer_;

    // Current goal handle for thread access
    std::shared_ptr<GoalHandleDoWork> current_goal_handle_;
    std::mutex goal_handle_mutex_;

    // Callback groups
    rclcpp::CallbackGroup::SharedPtr action_callback_group_;
    rclcpp::CallbackGroup::SharedPtr subscription_callback_group_;
    rclcpp::CallbackGroup::SharedPtr diagnostics_callback_group_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const DoWork::Goal> goal)
    {
        (void)uuid; // Suppress unused parameter warning
        RCLCPP_INFO(this->get_logger(), "Received goal request with duration %d seconds", goal->duration);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleDoWork> goal_handle)
    {
        (void)goal_handle; // Suppress unused parameter warning
        RCLCPP_INFO(this->get_logger(), "Cancel request received");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleDoWork> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Goal accepted");

        // Wait for any previous thread to complete
        if (work_thread_ && work_thread_->joinable()) {
            RCLCPP_INFO(this->get_logger(), "Waiting for previous work thread to complete");
            work_thread_->join();
        }

        // Store goal handle for thread access
        {
            std::lock_guard<std::mutex> lock(goal_handle_mutex_);
            current_goal_handle_ = goal_handle;
        }

        // Start new work thread (undetached)
        work_thread_ = std::make_shared<std::thread>(&DoWorkServer::execute_work, this, goal_handle);
    }

    void pause_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        // Set pause to true only if previous value was false and current message is true
        if (!previous_pause_value_ && msg->data) {
            pause_ = true;
            RCLCPP_INFO(this->get_logger(), "Work paused");
        } else if (!msg->data) {
            pause_ = false;
            RCLCPP_INFO(this->get_logger(), "Work resumed");
        }
        previous_pause_value_ = msg->data;
    }

    void execute_work(std::shared_ptr<GoalHandleDoWork> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Starting work execution");

        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<DoWork::Feedback>();
        auto result = std::make_shared<DoWork::Result>();

        int work_count = 1;
        auto start_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::seconds(goal->duration);
        auto last_pause_log = std::chrono::steady_clock::now() - std::chrono::seconds(4); // Initialize to allow immediate first log

        while (rclcpp::ok()) {
            // Check if goal was cancelled
            if (goal_handle->is_canceling()) {
                RCLCPP_INFO(this->get_logger(), "Goal was cancelled during execution");
                goal_handle->canceled(result);
                return;
            }

            // Check if paused
            if (pause_.load()) {
                auto now = std::chrono::steady_clock::now();
                if (std::chrono::duration_cast<std::chrono::seconds>(now - last_pause_log).count() >= 3) {
                    RCLCPP_INFO(this->get_logger(), "Work execution paused");
                    last_pause_log = now;
                }
                std::this_thread::sleep_for(100ms);
                continue;
            }

            // Calculate elapsed time
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - start_time);

            // Check if duration completed
            if (elapsed >= duration) {
                RCLCPP_INFO(this->get_logger(), "Work completed successfully");
                goal_handle->succeed(result);
                return;
            }

            // Publish work message
            auto work_msg = std_msgs::msg::String();
            work_msg.data = "work " + std::to_string(work_count++);
            work_publisher_->publish(work_msg);
            RCLCPP_INFO(this->get_logger(), "Published: %s", work_msg.data.c_str());

            // Send feedback
            feedback->seconds_elapsed = static_cast<int32_t>(elapsed.count());
            goal_handle->publish_feedback(feedback);

            // Sleep for 1 second
            std::this_thread::sleep_for(1s);
        }
    }

    void diagnostics_callback(diagnostic_updater::DiagnosticStatusWrapper & stat)
    {
        bool thread_active = false;
        bool thread_joinable = false;

        if (work_thread_) {
            thread_joinable = work_thread_->joinable();
            // Consider thread active if it's joinable (running)
            thread_active = thread_joinable;
        }

        bool is_paused = pause_.load();

        if (thread_active && !is_paused) {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Work thread running normally");
        } else if (thread_active && is_paused) {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Work thread paused");
        } else if (!thread_active) {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "No active work thread");
        }

        stat.add("thread_active", thread_active ? "true" : "false");
        stat.add("thread_joinable", thread_joinable ? "true" : "false");
        stat.add("paused", is_paused ? "true" : "false");
        stat.add("node_status", "healthy");
    }

    void update_diagnostics()
    {
        diagnostics_updater_->force_update();
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DoWorkServer>();

    // Use MultiThreadedExecutor to enable true parallelism with callback groups
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}