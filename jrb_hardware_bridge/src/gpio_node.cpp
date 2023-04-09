#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "serial/serial.h"

class GpioNode : public rclcpp::Node
{
public:
    GpioNode()
        : Node("gpio_node")
    {
        // Internal state
        initSerial();

        // Subscribers
        serialWrite_sub = this->create_subscription<std_msgs::msg::String>(
            "hardware/arduino/serial_write", 10, std::bind(&GpioNode::serialWriteCallback, this, std::placeholders::_1));

        // Publishers
        auto latchedQos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
        
        serialRead_pub = this->create_publisher<std_msgs::msg::String>("hardware/arduino/serial_read", 10);
        starter_pub = this->create_publisher<std_msgs::msg::Bool>("hardware/starter", 10);
        team_pub = this->create_publisher<std_msgs::msg::String>("hardware/team", latchedQos);
        strategy_pub = this->create_publisher<std_msgs::msg::Bool>("hardware/strategy", latchedQos);

        // Timers
        readTimer = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&GpioNode::readSerialCallback, this));

        // Initial read of gpio values
        sendStringOverSerial("m umuxe");
        sendStringOverSerial("r");
    }

    void sendStringOverSerial(const std::string &stringToSend)
    {
        serialConnection.write(stringToSend+"\n");
    }

private:
    void initSerial()
    {
        serialConnection.setPort("/dev/arduino");
        serialConnection.setBaudrate(115200);
        serial::Timeout tt = serial::Timeout::simpleTimeout(50); // ms
        serialConnection.setTimeout(tt);                         // This should be inline except setTimeout takes a reference and so needs a variable
        serialConnection.open();
    }

    void serialWriteCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        sendStringOverSerial(msg->data);
    }

    void readSerialCallback()
    {
        std::string data = serialConnection.readline();

        if (data == "")
        {
            return;
        }

        // Remove \r\n
        data.erase(data.size() - 2);

        // Publish raw data
        publishSerialRead(data);

        extractValuesFromData(data);
    }

    void extractValuesFromData(std::string data)
    {

        char command = data.front();
        std::string values = data.substr(2);

        switch (command)
        {
        // Digital read value
        case 'd':
        {
            int digitalPin;
            int value;
            sscanf(values.c_str(), "%d %d", &digitalPin, &value);

            if (digitalPin == 21)
            {
                publishStarter(value != 0);
                RCLCPP_INFO_STREAM(get_logger(), "starter: " << value);
            }
            else if (digitalPin == 22)
            {
                std::string teamValue = (value == 1) ? "yellow" : "purple";
                publishTeam(teamValue);
                RCLCPP_INFO_STREAM(get_logger(), "team: " << teamValue);
            }
            else if (digitalPin == 23)
            {
                publishStrategy(value != 0);
                RCLCPP_INFO_STREAM(get_logger(), "strategy: " << value);
            }
            else
            {
                RCLCPP_INFO_STREAM(get_logger(), "digitalPin " << digitalPin << ": " << value);
            }

            break;
        }

        // Log
        case 'l':
        {
            RCLCPP_INFO_STREAM(get_logger(), "ARDUINO_LOG: " << values);
            break;
        }

        default:
        {
            RCLCPP_WARN_STREAM(get_logger(), "Unknown command: " << data);
        }
        }
    }

    void publishSerialRead(std::string serialReadValue) const
    {
        std_msgs::msg::String msg;
        msg.data = serialReadValue;
        serialRead_pub->publish(msg);
    }

    void publishStarter(bool starterValue) const
    {
        std_msgs::msg::Bool msg;
        msg.data = starterValue;
        starter_pub->publish(msg);
    }

    void publishTeam(std::string teamValue) const
    {
        std_msgs::msg::String msg;
        msg.data = teamValue;
        team_pub->publish(msg);
    }

    void publishStrategy(bool strategyValue) const
    {
        std_msgs::msg::Bool msg;
        msg.data = strategyValue;
        strategy_pub->publish(msg);
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr serialWrite_sub;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr serialRead_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr starter_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr team_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr strategy_pub;

    rclcpp::TimerBase::SharedPtr readTimer;

    serial::Serial serialConnection;

    bool starterValue, strategyValue;
    std::string teamValue;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GpioNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
