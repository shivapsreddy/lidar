#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "sensor_msgs/msg/temperature.hpp"

#include "lidar_thermal_cpp/visibility_control.h"

#ifdef LIDAR_THERMAL_CPP_STD_MSG
#include "std_msgs/msg/string.hpp"
#endif

#ifdef LOG_MISSING_FRAMES
  #ifndef AVERAGE_FRAMES_COUNT
    #define AVERAGE_FRAMES_COUNT 100
  #endif
static uint32_t last_frame_id;
static uint32_t current_frame_id;
static int32_t missed_frames_count=0;
static int32_t average_period=0;
#endif

namespace lidar_thermal_cpp
{
  class Lidar_SoCTemperatureSubscriber: public rclcpp::Node
  {
    private:
      rmw_qos_reliability_policy_t reliability_policy;
      rmw_qos_history_policy_t history_policy;
      rclcpp::Subscription<sensor_msgs::msg::Temperature>::ConstSharedPtr lidar_SoCTemp_subscriber;
#ifdef LIDAR_THERMAL_CPP_STD_MSG
      rclcpp::Subscription<std_msgs::msg::String>::SharedPtr lidar_std_sub;
#endif

    public:
      void call_back(const sensor_msgs::msg::Temperature::SharedPtr msg);

      LIDAR_THERMAL_CPP_PUBLIC
      explicit Lidar_SoCTemperatureSubscriber(const rclcpp::NodeOptions & options) : Node("Lidar_SoCTemperature", options)
      {
        setvbuf(stdout, NULL, _IONBF, BUFSIZ);
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, 1));
        qos.reliability(reliability_policy);

        /*Create a callback function to receive the message from the publisher*/

        /*Create subscriber*/
        lidar_SoCTemp_subscriber = this->create_subscription<sensor_msgs::msg::Temperature>("Lidar_SoCTemperature", qos,
            std::bind(&Lidar_SoCTemperatureSubscriber::call_back, this, std::placeholders::_1));
#ifdef LIDAR_THERMAL_CPP_STD_MSG
        // Create a callback function for when messages are received.
        // Variations of this function also exist using, for example UniquePtr for zero-copy transport.
        //setvbuf(stdout, NULL, _IONBF, BUFSIZ);
        auto callback =
          [this](const std_msgs::msg::String::SharedPtr msg) -> void
          {
            RCLCPP_INFO(this->get_logger(), "I heard: [%s]", msg->data.c_str());
          };
        // Create a subscription to the topic which can be matched with one or more compatible ROS
        // publishers.
        // Note that not all publishers on the same topic with the same type will be compatible:
        // they must have compatible Quality of Service policies.
        lidar_std_sub = create_subscription<std_msgs::msg::String>("Lidar_SoCTemperature", 10, callback);
#endif
    }

  };

  void Lidar_SoCTemperatureSubscriber::call_back(const sensor_msgs::msg::Temperature::SharedPtr msg)
  {
#ifdef LOG_MISSING_FRAMES

    current_frame_id = std::stoi(msg->header.frame_id);
    RCLCPP_INFO(this->get_logger(), "Received temperature message with sequence number: '%d' SoC temperature is '%f'", current_frame_id, msg->temperature);
    if(last_frame_id == 0) {
      last_frame_id = current_frame_id;
    }
    else {

      missed_frames_count += current_frame_id - last_frame_id - 1;
      average_period += current_frame_id - last_frame_id;

      if(average_period >= AVERAGE_FRAMES_COUNT)
      {
         RCLCPP_INFO(this->get_logger(), "The received frames per the last  %d frames is : %d  current frame : %d ", average_period, (average_period - missed_frames_count), current_frame_id );
         average_period = missed_frames_count = 0;
      }
      last_frame_id = current_frame_id;
    }
#else
    RCLCPP_INFO(this->get_logger(), "Received depth-image with sequence number: '%s' SoC temperature is '%f'", msg->header.frame_id.c_str(), msg->temperature);
#endif


  }
}  // namespace lidar_thermal_cpp
RCLCPP_COMPONENTS_REGISTER_NODE(lidar_thermal_cpp::Lidar_SoCTemperatureSubscriber)
