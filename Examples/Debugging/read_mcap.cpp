/*
  This program:
    1) Opens a .mcap bag file using rosbag2_cpp with the "mcap" storage plugin.
    2) (Optionally) filters to certain topics if desired.
    3) Iterates over all messages, counts them per topic, and records which
       message type is used on each topic.
    4) Demonstrates how you *might* deserialize known message types in C++.

  Usage:
    $ ros2 run my_bag_reader read_mcap --ros-args -p
  bag_file:=/path/to/file.mcap
*/

#include <algorithm>
#include <filesystem> // C++17 <filesystem> for creating directories, if needed
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rcutils/time.h>

#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/bag_metadata.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/storage_options.hpp>

// If you want to deserialize sensor_msgs::msg::Image, for example:
#include <sensor_msgs/msg/image.hpp>

// cv_bridge + OpenCV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "ORBextractor.h"

int main(int argc, char *argv[]) {
  // Initialize rclcpp if you need node-based functionality
  rclcpp::init(argc, argv);

  // Create a node to use for logging or parameter parsing
  auto node = std::make_shared<rclcpp::Node>("read_mcap_node");

  // Get the .mcap file path from a parameter or fallback
  std::string input_bag = node->declare_parameter<std::string>("bag_file", "");
  if (input_bag.empty()) {
    RCLCPP_ERROR(node->get_logger(),
                 "Please specify a MCAP file path via 'bag_file' parameter.");
    return 1;
  }

  // Create the output directory if it doesn't exist
  const std::string output_dir = "output_images";
  try {
    std::filesystem::create_directories(output_dir);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node->get_logger(), "Failed to create directory '%s': %s",
                 output_dir.c_str(), e.what());
    return 1;
  }

  // Setup rosbag2 storage options
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = input_bag;
  storage_options.storage_id = "mcap"; // to read MCAP files

  // Setup converter options (we read and output in "cdr" format)
  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";

  // Create reader with a SequentialReader
  rosbag2_cpp::Reader reader(
      std::make_unique<rosbag2_cpp::readers::SequentialReader>());
  reader.open(storage_options, converter_options);

  // Optional: Filter to only read certain topics
  /*
  rosbag2_storage::StorageFilter filter;
  filter.topics.push_back("/my_topic");
  reader.set_filter(filter);
  */

  // Retrieve the topic list & types
  auto topics_and_types = reader.get_all_topics_and_types();

  // Build a map: topic -> type
  std::map<std::string, std::string> topic_to_types;
  for (auto &topic_info : topics_and_types) {
    topic_to_types[topic_info.name] = topic_info.type;
  }

  // Maps for stats
  std::map<std::string, size_t> topic_message_counts;
  std::map<std::string, std::vector<std::string>> type_to_topics;

  // Track whether we've already saved frames (0=none, 1=first saved, 2=second
  // saved)
  static std::map<std::string, int> first_image_saved;

  // ──────────────────────────────────────────────────────────────────────────────
  //  New data structures to store first-frame data for matching later
  // ──────────────────────────────────────────────────────────────────────────────
  static std::map<std::string, cv::Mat> first_image;
  static std::map<std::string, std::vector<cv::KeyPoint>> first_keypoints;
  static std::map<std::string, cv::Mat> first_descriptors;

  // If some code above tries to see if "sensor_msgs/msg/Image" is in
  // type_to_topics: (You can do this later or not at all. This snippet just
  // ensures the map keys exist.)
  if (type_to_topics.find("sensor_msgs/msg/Image") != type_to_topics.end()) {
    for (const auto &topic : type_to_topics["sensor_msgs/msg/Image"]) {
      first_image_saved[topic] = 0;
    }
  }

  // Main loop: read messages until there are no more
  while (reader.has_next()) {
    // Read the next message
    auto serialized_msg = reader.read_next();
    const auto &topic_name = serialized_msg->topic_name;
    const auto &time_ns = serialized_msg->time_stamp;

    // Bump the count of messages for this topic
    topic_message_counts[topic_name]++;

    // Lookup the type for this topic
    const auto &type_name = topic_to_types[topic_name];

    // Add to type->topics map if new
    auto it = type_to_topics.find(type_name);
    if (it == type_to_topics.end()) {
      type_to_topics[type_name] = {topic_name};
    } else {
      auto &topic_vec = it->second;
      if (std::find(topic_vec.begin(), topic_vec.end(), topic_name) ==
          topic_vec.end()) {
        topic_vec.push_back(topic_name);
      }
    }

    // ORBextractor(int nfeatures, float scaleFactor, int nlevels,
    //              int iniThFAST, int minThFAST);
    // Create an ORB extractor
    std::unique_ptr<ORB_SLAM3::ORBextractor> orb_extractor =
        std::make_unique<ORB_SLAM3::ORBextractor>(250, 1.2, 12, 12, 5);

    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    std::vector<int> vLapping = {0, 0};

    // Only process sensor_msgs/msg/Image
    if (type_name == "sensor_msgs/msg/Image") {

      // If we've already processed first and second frames for this topic, skip
      if (first_image_saved[topic_name] == 2) {
        continue;
      }

      // Use rclcpp::Serialization to deserialize
      static rclcpp::Serialization<sensor_msgs::msg::Image> serializer;
      sensor_msgs::msg::Image image_msg;
      rclcpp::SerializedMessage rcl_serialized;

      // Copy from the bag's serialized_data to our rcl_serialized buffer
      rcl_serialized.reserve(serialized_msg->serialized_data->buffer_length);
      memcpy(rcl_serialized.get_rcl_serialized_message().buffer,
             serialized_msg->serialized_data->buffer,
             serialized_msg->serialized_data->buffer_length);
      rcl_serialized.get_rcl_serialized_message().buffer_length =
          serialized_msg->serialized_data->buffer_length;

      // Deserialize into our ROS 2 message
      serializer.deserialize_message(&rcl_serialized, &image_msg);

      // Convert to OpenCV image with cv_bridge
      cv_bridge::CvImagePtr cv_ptr;
      try {
        // "bgr8" for typical color images; adjust if necessary
        cv_ptr = cv_bridge::toCvCopy(image_msg, "bgr8");
      } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(node->get_logger(), "cv_bridge exception: %s", e.what());
        continue;
      }

      // ──────────────────────────────────────────────────────────────────────────
      //  Perform ORB extraction on the current frame
      // ──────────────────────────────────────────────────────────────────────────
      int num_of_features = (*orb_extractor)(cv_ptr->image, cv::Mat(),
                                             keypoints, descriptors, vLapping);

      // Create safe filename components
      std::string sanitized_topic = topic_name;
      std::replace(sanitized_topic.begin(), sanitized_topic.end(), '/', '_');

      // ──────────────────────────────────────────────────────────────────────────
      //  CASE 1: If this is the VERY FIRST frame for this topic
      // ──────────────────────────────────────────────────────────────────────────
      if (first_image_saved[topic_name] == 0) {
        // Store the first frame's image, keypoints, descriptors
        first_image[topic_name] = cv_ptr->image.clone();
        first_keypoints[topic_name] = keypoints;
        first_descriptors[topic_name] = descriptors.clone();

        // Save the raw image
        std::string out_filename =
            output_dir + "/0_" + sanitized_topic + ".jpg";
        cv::imwrite(out_filename, cv_ptr->image);

        // For debugging, you can also draw and save the keypoints of the first
        // frame
        cv::Mat output_first;
        cv::drawKeypoints(cv_ptr->image, keypoints, output_first,
                          cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
        std::string features_filename =
            output_dir + "/0_features_" + sanitized_topic + ".jpg";
        cv::imwrite(features_filename, output_first);

        // Mark that we've processed the first frame
        first_image_saved[topic_name] = 1;
      }
      // ──────────────────────────────────────────────────────────────────────────
      //  CASE 2: If this is the SECOND frame for this topic
      // ──────────────────────────────────────────────────────────────────────────
      else if (first_image_saved[topic_name] == 1) {
        // We now have the first frame data stored; let’s do the matching
        cv::BFMatcher bf(cv::NORM_HAMMING, /*crossCheck=*/false);
        std::vector<cv::DMatch> matches;
        bf.match(first_descriptors[topic_name], descriptors, matches);

        // Sort matches by distance so we can draw the "best" ones first
        std::sort(matches.begin(), matches.end(),
                  [](const cv::DMatch &a, const cv::DMatch &b) {
                    return a.distance < b.distance;
                  });

        std::cout << "There is " << matches.size() << " matches for " << sanitized_topic << std::endl;

        // (Optional) Keep only the top N matches for visualization clarity
        const size_t MAX_MATCHES = 5;
        if (matches.size() > MAX_MATCHES) {
          matches.resize(MAX_MATCHES);
        }


        // Draw side-by-side matches
        cv::Mat matched_output;
        cv::drawMatches(first_image[topic_name],     // img1
                        first_keypoints[topic_name], // keypoints1
                        cv_ptr->image,               // img2
                        keypoints,                   // keypoints2
                        matches,                     // matches
                        matched_output,              // output image
                        cv::Scalar::all(-1), // matchColor
                        cv::Scalar::all(-1), // singlePointColor
                        std::vector<char>(), // matchesMask
                        cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

        // Save the side-by-side match result
        std::string match_filename =
            output_dir + "/1_matches_" + sanitized_topic + ".jpg";
        cv::imwrite(match_filename, matched_output);

        // (Optional) Also save the second frame’s raw image and its keypoints
        std::string second_image_filename =
            output_dir + "/1_" + sanitized_topic + ".jpg";
        cv::imwrite(second_image_filename, cv_ptr->image);

        cv::Mat output_second;
        cv::drawKeypoints(cv_ptr->image, keypoints, output_second,
                          cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
        std::string second_features_filename =
            output_dir + "/1_features_" + sanitized_topic + ".jpg";
        cv::imwrite(second_features_filename, output_second);

        // After second frame, mark so we don’t keep doing this
        first_image_saved[topic_name] = 2;
      }
    }
  }

  // Print statistics
  std::cout << "\nTopics and Number of Messages:\n";
  for (const auto &kv : topic_message_counts) {
    std::cout << "  Topic: " << kv.first << ", Count: " << kv.second << "\n";
  }

  std::cout << "\nTopic -> Type:\n";
  for (const auto &kv : topic_to_types) {
    std::cout << "  " << kv.first << " : " << kv.second << "\n";
  }

  std::cout << "\nType -> Topics:\n";
  for (const auto &kv : type_to_topics) {
    std::cout << "  " << kv.first << " -> [ ";
    for (auto &t : kv.second) {
      std::cout << t << " ";
    }
    std::cout << "]\n";
  }

  return 0;
}
