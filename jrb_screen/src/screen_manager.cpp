#include <iostream>
#include <string>
#include <thread>
#include <memory>
#include <stdexcept>
#include <raylib.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

constexpr int STATE_WAIT_FOR_MATCH = 0;
constexpr int STATE_MATCH = 1;
constexpr int STATE_AFTER_MATCH = 2;

class ScreenManager : public rclcpp::Node
{
public:
    ScreenManager()
        : Node("screen_manager"),
          width_(800),
          height_(480),
          fps_(5),
          state_(STATE_WAIT_FOR_MATCH),
          stop_(false),
          score_("0"),
          text_(""),
          team_(""),
          strategy_(""),
          fullscreen_(false)
    {
        RCLCPP_INFO(this->get_logger(), "init");

        // Parameters
        rcl_interfaces::msg::IntegerRange integer_range_width;
        integer_range_width.from_value = 100;
        integer_range_width.to_value = 1000;
        integer_range_width.step = 10;

        rcl_interfaces::msg::ParameterDescriptor param_descriptor_width;
        param_descriptor_width.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
        param_descriptor_width.integer_range.push_back(integer_range_width);
        param_descriptor_width.read_only = true;

        declare_parameter<int>("width", 800, param_descriptor_width);

        rcl_interfaces::msg::IntegerRange integer_range_height;
        integer_range_height.from_value = 100;
        integer_range_height.to_value = 1000;
        integer_range_height.step = 10;

        rcl_interfaces::msg::ParameterDescriptor param_descriptor_height;
        param_descriptor_height.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
        param_descriptor_height.integer_range.push_back(integer_range_height);
        param_descriptor_height.read_only = true;

        declare_parameter<int>("height", 480, param_descriptor_height);

        rcl_interfaces::msg::IntegerRange integer_range_fps;
        integer_range_fps.from_value = 1;
        integer_range_fps.to_value = 60;
        integer_range_fps.step = 1;

        rcl_interfaces::msg::ParameterDescriptor param_descriptor_fps;
        param_descriptor_fps.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
        param_descriptor_fps.integer_range.push_back(integer_range_fps);
        param_descriptor_fps.read_only = true;

        declare_parameter<int>("fps", 5, param_descriptor_fps);

        width_ = get_parameter("width").as_int();
        height_ = get_parameter("height").as_int();
        fps_ = get_parameter("fps").as_int();
        
        fullscreen_ = !get_parameter("use_sim_time").as_bool();

        // Subscriptions
        auto qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default))
                               .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
                               .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                               .keep_last(1);

        sub_score_ = this->create_subscription<std_msgs::msg::String>(
            "screen/score", 10, std::bind(&ScreenManager::on_score, this, std::placeholders::_1));

        sub_text_ = this->create_subscription<std_msgs::msg::String>(
            "screen/text", 10, std::bind(&ScreenManager::on_text, this, std::placeholders::_1));

        sub_starter_ = this->create_subscription<std_msgs::msg::Bool>(
            "hardware/starter", qos_profile,
            std::bind(&ScreenManager::on_starter, this, std::placeholders::_1));

        sub_team_ = this->create_subscription<std_msgs::msg::String>(
            "hardware/team", qos_profile,
            std::bind(&ScreenManager::on_team, this, std::placeholders::_1));

        sub_strategy_ = this->create_subscription<std_msgs::msg::Bool>(
            "hardware/strategy", qos_profile,
            std::bind(&ScreenManager::on_strategy, this, std::placeholders::_1));

        sub_end_match_ = this->create_subscription<std_msgs::msg::Empty>(
            "strategy/end_match", 1,
            std::bind(&ScreenManager::on_end_match, this, std::placeholders::_1));

        // Internal state
        raylib_thread_ = std::thread(&ScreenManager::loop, this);
    }

    ~ScreenManager()
    {
        if (raylib_thread_.joinable())
        {
            raylib_thread_.join();
        }
    }

    void on_score(const std_msgs::msg::String::SharedPtr msg)
    {
        score_ = msg->data;
    }

    void on_text(const std_msgs::msg::String::SharedPtr msg)
    {
        text_ = msg->data;
    }

    void on_starter(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (state_ == STATE_WAIT_FOR_MATCH && msg->data)
        {
            state_ = STATE_MATCH;
        }
        else if (state_ == STATE_AFTER_MATCH && !msg->data)
        {
            state_ = STATE_WAIT_FOR_MATCH;
        }
    }

    void on_team(const std_msgs::msg::String::SharedPtr msg)
    {
        team_ = msg->data;
    }

    void on_strategy(const std_msgs::msg::Bool::SharedPtr msg)
    {
        strategy_ = std::to_string(msg->data);
    }

    void on_end_match(const std_msgs::msg::Empty::SharedPtr msg)
    {
        (void)msg;
        state_ = STATE_AFTER_MATCH;
    }

    void loop()
    {
        if (fullscreen_) {
            InitWindow(0, 0, "jeroboam_screen");
            ToggleFullscreen();
            width_ = GetScreenWidth(); 
            height_ = GetScreenHeight();
            RCLCPP_ERROR(get_logger(), "%d %d", width_, height_);
        } else {
            InitWindow(width_, height_, "jeroboam_screen");
        } 
        

        SetTargetFPS(fps_);
        std::string data_path = ament_index_cpp::get_package_share_directory("jrb_screen");
        Image image = LoadImage((data_path + "/assets/Jeroboam.png").c_str());
        ImageResize(&image, 200, 200);
        Texture2D logo_texture = LoadTextureFromImage(image);
        UnloadImage(image);

        while (rclcpp::ok() && !WindowShouldClose())
        {
            BeginDrawing();
            ClearBackground(RAYWHITE);
            DrawTexture(logo_texture, 10, 10, WHITE);

            std::string big_text, subtext;
            if (state_ == STATE_WAIT_FOR_MATCH)
            {
                big_text = "Le match va commencer...";
                subtext = "Équipe: " + team_ + " Stratégie: " + strategy_;
            }
            else if (state_ == STATE_MATCH)
            {
                big_text = "SCORE: " + score_;
            }
            else if (state_ == STATE_AFTER_MATCH)
            {
                big_text = "SCORE: " + score_;
                subtext = "Match fini !";
            }

            int big_text_width = MeasureText(big_text.c_str(), 50);
            int subtext_width = MeasureText(subtext.c_str(), 20);
            int text_width = MeasureText(text_.c_str(), 20);

            DrawText(big_text.c_str(), (width_ - big_text_width) / 2, 0.45 * height_, 50, MAROON);
            DrawText(subtext.c_str(), (width_ - subtext_width) / 2, 0.7 * height_, 20, LIGHTGRAY);
            DrawText(text_.c_str(), (width_ - text_width) / 2, 0.8 * height_, 20, LIGHTGRAY);

            EndDrawing();
        }

        CloseWindow();
        rclcpp::shutdown();
    }

private:
    int width_;
    int height_;
    int fps_;
    int state_;
    bool stop_;
    std::string score_;
    std::string text_;
    std::string team_;
    std::string strategy_;
    bool fullscreen_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_score_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_text_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_starter_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_team_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_strategy_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_end_match_;

    std::thread raylib_thread_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ScreenManager>();

    try
    {
        rclcpp::spin(node);
    }
    catch (std::exception &e)
    {
        RCLCPP_ERROR(node->get_logger(), "Error while stopping the node: %s", e.what());
        throw;
    }
    catch (...)
    {
        RCLCPP_ERROR(node->get_logger(), "Unknown error while stopping the node");
        throw;
    }

    rclcpp::shutdown();
    return 0;
}