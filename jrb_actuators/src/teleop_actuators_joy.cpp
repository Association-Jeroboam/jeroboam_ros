#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/bool.hpp"

class TeleopActuatorsJoy : public rclcpp::Node
{
public:
    TeleopActuatorsJoy()
        : Node("teleop_actuators_joy")
    {
        // Subscribers
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&TeleopActuatorsJoy::joy_callback, this, std::placeholders::_1));

        // Publishers
        turbine_speed_pub_ = this->create_publisher<std_msgs::msg::UInt16>("hardware/turbine/speed", 10);
        led_pub_ = this->create_publisher<std_msgs::msg::Bool>("hardware/led", 10);
        roll_height_pub_ = this->create_publisher<std_msgs::msg::Int16>("hardware/roll/height", 10);
        roll_speed_pub_ = this->create_publisher<std_msgs::msg::Int8>("hardware/roll/speed", 10);

        RCLCPP_INFO(get_logger(), "TeleopActuatorsJoy node has been initialized");
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        /**
         * Robotbleu controls
         */

        // X button: turbine speed
        static bool x_button_prev_state = false;
        static uint8_t speed_mode = 0;

        const uint16_t SPEED1 = 500;
        const uint16_t SPEED2 = 1000;

        bool x_button_curr_state = msg->buttons[0];

        if (x_button_curr_state && !x_button_prev_state)
        {
            speed_mode = (speed_mode + 1) % 3;
            uint16_t speed_value = 0;

            switch (speed_mode)
            {
            case 0:
                speed_value = 0;
                break;
            case 1:
                speed_value = SPEED1;
                break;
            case 2:
                speed_value = SPEED2;
                break;
            }

            std_msgs::msg::UInt16 speed_msg;
            speed_msg.data = speed_value;
            turbine_speed_pub_->publish(speed_msg);
        }

        x_button_prev_state = x_button_curr_state;

        // Y button: LED control
        static bool y_button_prev_state = false;
        static bool led_state = false;

        if (msg->buttons[3] && !y_button_prev_state)
        {
            led_state = !led_state;
            std_msgs::msg::Bool led_msg;
            led_msg.data = led_state;
            led_pub_->publish(led_msg);
        }

        y_button_prev_state = msg->buttons[3];

        // Top / bottom arrows: Roll height control
        static const int16_t ROLL_HEIGHT_INCREMENT = 10;
        static int16_t roll_height = 0;

        bool publish_roll_height = false;

        if (msg->axes[7] > 0.5)
        {
            roll_height += ROLL_HEIGHT_INCREMENT;
            publish_roll_height = true;
        }
        else if (msg->axes[7] < -0.5)
        {
            roll_height -= ROLL_HEIGHT_INCREMENT;
            publish_roll_height = true;
        }

        if (publish_roll_height) {
            std_msgs::msg::Int16 roll_height_msg;
            roll_height_msg.data = roll_height;
            roll_height_pub_->publish(roll_height_msg);
        }

        // Left & right big trigger: Roll speed control
        static float prev_axes_2 = 1.0; // Left
        static float prev_axes_5 = 1.0; // right
        static int8_t roll_speed = 0;

        int8_t prev_roll_speed = roll_speed;
        float axes_2 = msg->axes[2];
        float axes_5 = msg->axes[5];

        if ((axes_2 != prev_axes_2) || (axes_5 != prev_axes_5))
        {
            roll_speed = 0;
            if (std::fabs(axes_2 - 1.0) >= 0.1)
            {
                roll_speed = 1;
            }
            if (std::fabs(axes_5 - 1.0) >= 0.1)
            {
                roll_speed = -1;
            }

            if (std::fabs(roll_speed - prev_roll_speed) >= 0.1) {
                std_msgs::msg::Int8 roll_speed_msg;
                roll_speed_msg.data = roll_speed;
                roll_speed_pub_->publish(roll_speed_msg);
            }
        }

        prev_axes_2 = axes_2;
        prev_axes_5 = axes_5;
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr led_pub_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr roll_height_pub_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr roll_speed_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr turbine_speed_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopActuatorsJoy>());
    rclcpp::shutdown();
    return 0;
}