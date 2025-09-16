#include <algorithm>
#include <filesystem> // C++17 <filesystem> for creating directories, if needed
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "ORBextractor.h"

int main(int argc, char *argv[]) {
  // Check if user provided enough arguments
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <image_path>" << std::endl;
    return -1;
  }

  // The first command-line argument after the program name will be our image
  // file path
  std::string image_file = argv[1];

  // Load the image
  cv::Mat img = cv::imread(image_file, cv::IMREAD_COLOR);

  // Check if the image was loaded properly
  if (img.empty()) {
    std::cerr << "Failed to open image: " << image_file << std::endl;
    return -1;
  }

  std::unique_ptr<ORB_SLAM3::ORBextractor> orb_extractor =
      std::make_unique<ORB_SLAM3::ORBextractor>(250, 1.2, 12, 12, 5);
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;
  std::vector<int> vLapping = {0, 0};

  int num_of_features =
      (*orb_extractor)(img, cv::Mat(), keypoints, descriptors, vLapping);

  cv::Mat feature_extracted_img;
  cv::drawKeypoints(img, keypoints, feature_extracted_img, cv::Scalar::all(-1),
                    cv::DrawMatchesFlags::DEFAULT);

  // Example: Display the loaded image
  cv::imshow("Loaded Image", feature_extracted_img);
  cv::waitKey(0);

  return 0;
}
