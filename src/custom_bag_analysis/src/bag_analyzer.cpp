#include <chrono>
#include <functional> 
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_transport/reader_writer_factory.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
//#include "sensor_msgs/msg/camerainfo.hpp"

using namespace std::chrono_literals;

class PlaybackNode : public rclcpp::Node
{
  public: 
    PlaybackNode(const std::string & bag_filename) : Node("playback_node")
    {
      publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("sensor_msgs/msg/Imu", 10);
      timer_ = this->create_wall_timer(100ms, std::bind(&PlaybackNode::timer_callback, this));

      rosbag2_storage::StorageOptions storage_options;
      storage_options.uri = bag_filename;
      reader_ = rosbag2_transport::ReaderWriterFactory::make_reader(storage_options);
      reader_->open(storage_options);
    }
  private:
    void timer_callback()
    {
      while(reader_->has_next()) {
        rosbag2_storage::SerializedBagMessageSharedPtr msg = reader_->next();

        if(msg->topic_name != "/zed/zed_node/imu/data_raw") {
          continue;
        }

        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
        zed::msg::Imu::SharedPtr ros_msg = std::make_shared<zed::msg::Imu>();

        serialization_.deserialize_message(&serialized_msg, ros_msg.get());

        publisher_->publish(*ros_msg);
        std::cout << '(' << ros_msg->x << ", " << ros_msg->y << ")\n";
        break;
      }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<zed::msg::Imu>::SharedPtr publisher_;

    rclcpp::Serialization<zed::msg::Imu> serialization_;
    std::unique_ptr<rosbag2_cpp::Reader> reader_;
}


int main(int argc, char ** argv)
{
  if(argc != 2) {
    std::cerr << argv[0] << std::endl;
    return 1;
  }

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlaybackNode>(argv[1]));
  rclcpp::shutdown();
}