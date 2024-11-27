#include <iostream>
#include <chrono>
#include <memory>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/duration.hpp>

int main(int argc, char *argv[]) {
    // Verify input arguments
    
    std::string temp = argv[1];
    
    if (argc < 2) {
        std::cerr << "Usage: analyze_bag " << temp << std::endl;
        return 1;
    }
    std::string bag_path = argv[1];

    try {
        // Set up storage options
        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = bag_path;
        storage_options.storage_id = "sqlite3";

        // Initialize the rosbag2 reader
        rosbag2_cpp::Reader reader;
        reader.open(storage_options);

        // Variables for analysis
        size_t imu_count = 0;
        rclcpp::Time first_msg_time(0, 0, RCL_SYSTEM_TIME);
        rclcpp::Time last_msg_time(0, 0, RCL_SYSTEM_TIME);
        bool first_message = true;
        
        double total_time_difference = 0.0;
        rclcpp::Time previous_time(0, 0, RCL_SYSTEM_TIME);

        std::cout << "Analyzing bag file: " << bag_path << std::endl;

        // Iterate through messages
        while (reader.has_next()) {
            auto serialized_message = reader.read_next();
            auto topic_name = serialized_message->topic_name;

            // Choose different topics
            if (topic_name == "/zed/zed_node/left_raw_gray/camera_info") {
                // Deserialize the message
                auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
                rclcpp::Serialization<sensor_msgs::msg::Imu> serialization;
                rclcpp::SerializedMessage serialized_data(*serialized_message->serialized_data);
                serialization.deserialize_message(&serialized_data, imu_msg.get());

                // Extract timestamp
                rclcpp::Time current_time(imu_msg->header.stamp.sec, imu_msg->header.stamp.nanosec, RCL_SYSTEM_TIME);

                // Analyze time difference
                if (imu_count > 0) {
                    rclcpp::Duration time_diff = current_time - previous_time;
                    total_time_difference += time_diff.seconds();
                }

                // Store current timestamp for next iteration
                previous_time = current_time;

                // Set first and last message timestamps
                if (first_message) {
                    first_msg_time = current_time;
                    first_message = false;
                }
                last_msg_time = current_time;

                // Increment message count
                imu_count++;
            }
        }

        // Calculate statistics
        rclcpp::Duration duration = last_msg_time - first_msg_time;
        double total_time = duration.seconds();
        double frequency = (imu_count > 1) ? imu_count / total_time : 0.0;
        double avg_time_diff = (imu_count > 1) ? total_time_difference / (imu_count - 1) : 0.0;

        // Output results
        std::cout << "IMU Message Analysis Results:" << std::endl;
        std::cout << "  Total Messages: " << imu_count << std::endl;
        std::cout << "  Duration: " << total_time << " seconds" << std::endl;
        std::cout << "  Frequency: " << frequency << " Hz" << std::endl;
        std::cout << "  Average Time Difference between Messages: " << avg_time_diff << " seconds" << std::endl;

    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}

