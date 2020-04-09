#include <string>
#include <fstream>
#include <chrono>
#include <cstdio>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "sensor_msgs/msg/temperature.hpp"
#include "std_msgs/msg/header.hpp"

#include "lidar_thermal_cpp/visibility_control.h"
//#include "lidar_thermal_cpp/lidar_soc_temperature.hpp"

#ifdef LIDAR_THERMAL_CPP_STD_MSG
#include "std_msgs/msg/string.hpp"
#endif

using namespace std::chrono_literals;
using namespace std;
namespace lidar_thermal_cpp
{

  class Lidar_SoCTemperaturePublisher : public rclcpp::Node
  {
    private:
      //lidar_SoCTemperature lidar_temp;
      size_t count;
      double soc_temperature;
      int64_t current_period;
      rclcpp::TimerBase::SharedPtr timer;
      rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr lidar_thermal_publisher;
      rmw_qos_reliability_policy_t reliability_policy;
      rmw_qos_history_policy_t history_policy;
      sensor_msgs::msg::Temperature createTemperatureMessage();
      double read_SoCTemperature();
#ifdef LIDAR_THERMAL_CPP_STD_MSG
      rclcpp::TimerBase::SharedPtr std_timer;
      std::unique_ptr<std_msgs::msg::String> lidar_std_msg;
      rclcpp::Publisher<std_msgs::msg::String>::SharedPtr lidar_std_pub;
#endif

    public:
      void call_back();
      Lidar_SoCTemperaturePublisher();
      LIDAR_THERMAL_CPP_PUBLIC
      explicit Lidar_SoCTemperaturePublisher(const rclcpp::NodeOptions & options) : Node("Lidar_SocTemperature", options), count(0)
      {
        setvbuf(stdout, NULL, _IONBF, BUFSIZ);
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, 1));
        qos.reliability(reliability_policy);
        lidar_thermal_publisher = this->create_publisher<sensor_msgs::msg::Temperature>("Lidar_SoCTemperature", qos);
        auto period = std::chrono::nanoseconds(current_period*1000000);
        this->timer = this->create_wall_timer(1s, std::bind(&Lidar_SoCTemperaturePublisher::call_back, this));

#ifdef LIDAR_THERMAL_CPP_STD_MSG
        auto publish_message =
          [this]() -> void
          {
            lidar_std_msg = std::make_unique<std_msgs::msg::String>();
            lidar_std_msg->data = "Hello World: " + std::to_string(count);
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", lidar_std_msg->data.c_str());
            lidar_std_pub->publish(std::move(lidar_std_msg));
          };
        lidar_std_pub = this->create_publisher<std_msgs::msg::String>("Lidar_SoCTemeperature", qos);
        std_timer = this->create_wall_timer(1s, publish_message);
#endif
      }

  };

  sensor_msgs::msg::Temperature Lidar_SoCTemperaturePublisher::createTemperatureMessage()
  {
    sensor_msgs::msg::Temperature temperatureMessage;

    temperatureMessage.header = std_msgs::msg::Header();
    temperatureMessage.header.frame_id = std::to_string(this->count++);
    temperatureMessage.header.stamp = rclcpp::Clock().now();

    temperatureMessage.temperature = read_SoCTemperature();
    temperatureMessage.variance = 1.0;

    return temperatureMessage;
  }

  double Lidar_SoCTemperaturePublisher::read_SoCTemperature()
  {
#if 1
    double result;
    ifstream fin;
    fin.open("/sys/class/thermal/thermal_zone0/temp");
    if(!fin)
    {
      result = 0.0;
    }
    else
    {
      int tmp;
      fin >> tmp;
      result = double(tmp / 1000);
    }
    fin.close();

    return result;
#endif

  }

  void Lidar_SoCTemperaturePublisher::call_back()
  {
#if 0
     auto msg = std::make_unique<sensor_msgs::msg::Temperature>();
     //msg = std::make_unique<sensor_msgs::msg::Temperature>();
     msg->header = std_msgs::msg::Header();
     msg->header.frame_id = std::to_string(this->count++);
     msg->header.stamp = rclcpp::Clock().now();
     //msg->temperature = this->lidar_temp.get_lidar_SoCTemperature();
     msg->temperature = 99;
     msg->variance = 1.0;
     rclcpp::QoS qos(rclcpp::KeepLast(7));
     RCLCPP_INFO(this->get_logger(), "Publishing : '%d' temperature : '%f'", count, msg->temperature);
     lidar_thermal_publisher->publish(std::move(msg));
#else
     auto msg = createTemperatureMessage();
     RCLCPP_INFO(this->get_logger(), "Publishing : '%d' temperature : '%f'", count, msg.temperature);
     rclcpp::QoS qos(rclcpp::KeepLast(7));
     lidar_thermal_publisher->publish(msg);
#endif

  }
}  // namespace lidar_thermal_cpp
RCLCPP_COMPONENTS_REGISTER_NODE(lidar_thermal_cpp::Lidar_SoCTemperaturePublisher)
