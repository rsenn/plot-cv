/*
 * Author: Samyak Datta (datta[dot]samyak[at]gmail.com)
 *
 * This code is an implementation of the following paper
 * Juliano L. Moreira, Adriana Braun, Soraia R Musse, "Eyes and Eyebrow Detection for
 * Performance Driven Animation", 23rd SIBGRAPI Conference on Graphics, Patterns and Images, 2010
 *
 */

#include "eyebrow_roi.h"

#include <iostream>
#include <utility>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
// using namespace cv;

string input_image_path;
string face_cascade_path, eye_cascade_path;

cv::Mat_<uchar> CRTransform(const cv::Mat& image);
cv::Mat_<uchar> exponentialTransform(const cv::Mat_<uchar>& image);
pair<double, double> returnImageStats(const cv::Mat_<uchar>& image);
cv::Mat_<uchar> binaryThresholding(const cv::Mat_<uchar>& image, const pair<double, double>& stats);
int returnLargestContourIndex(std::vector<std::vector<cv::Point>> contours);

int
main(int argc, char** argv) {
  if(argc != 4) {
    cout << "Parameters missing!\n";
    return 1;
  }

  input_image_path = argv[1];
  face_cascade_path = argv[2];
  eye_cascade_path = argv[3];

  cv::Mat_<Vec3b> image_BGR = cv::imread(input_image_path);

  // Detect faces and eyebrows in image
  EyebrowROI eyebrow_detector(image_BGR, face_cascade_path, eye_cascade_path);
  eyebrow_detector.detectEyebrows();
  std::vector<cv::Mat> eyebrows_roi = eyebrow_detector.displayROI();

  // cv::Mat_<uchar> image_exp = exponentialTransform(CRTransform(image_BGR));
  cv::Mat_<uchar> image_exp = exponentialTransform(CRTransform(eyebrows_roi[0]));
  cv::Mat_<uchar> image_binary = binaryThresholding(image_exp, returnImageStats(image_exp));

  // A clone image is required because cv::findContours() modifies the input image
  cv::Mat image_binary_clone = image_binary.clone();
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(image_binary_clone, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

  // Initialize blank image (for drawing contours)
  cv::Mat_<uchar> image_contour(image_binary.size());
  for(int i = 0; i < image_contour.rows; ++i) {
    for(int j = 0; j < image_contour.cols; ++j) image_contour.at<uchar>(i, j) = 0;
  }

  // Draw largest contour on the blank image
  cout << "cv::Size of the contour image: " << image_contour.rows << " X " << image_contour.cols << "\n";
  int largest_contour_idx = returnLargestContourIndex(contours);
  for(int i = 0; i < contours[largest_contour_idx].size(); ++i) {
    cv::Point_<int> pt = contours[largest_contour_idx][i];
    image_contour.at<uchar>(pt.y, pt.x) = 255;
  }

  cv::imshow("Binary-Image", image_binary);
  cv::imshow("Contour", image_contour);

  cv::waitKey(0);
  return 0;
}

cv::Mat_<uchar>
CRTransform(const cv::Mat& image) {
  cv::Mat_<Vec3b> _image = image;
  cv::Mat_<uchar> CR_image(image.size());
  for(int i = 0; i < image.rows; ++i) {
    for(int j = 0; j < image.cols; ++j) CR_image.at<uchar>(i, j) = (255 - _image(i, j)[2]);
  }
  return CR_image;
}

cv::Mat_<uchar>
exponentialTransform(const cv::Mat_<uchar>& image) {
  std::vector<int> exponential_transform(256, 0);
  for(int i = 0; i < 256; ++i) exponential_transform[i] = round(cv::exp((i * cv::log(255)) / 255));

  cv::Mat_<uchar> image_exp(image.size());
  for(int i = 0; i < image.rows; ++i) {
    for(int j = 0; j < image.cols; ++j) image_exp.at<uchar>(i, j) = exponential_transform[image.at<uchar>(i, j)];
  }
  return image_exp;
}

pair<double, double>
returnImageStats(const cv::Mat_<uchar>& image) {
  double cv::mean = 0.0, std_dev = 0.0;
  int total_pixels = (image.rows * image.cols);

  int intensity_sum = 0;
  for(int i = 0; i < image.rows; ++i) {
    for(int j = 0; j < image.cols; ++j) intensity_sum += image.at<uchar>(i, j);
  }
  cv::mean = (double)intensity_sum / total_pixels;

  int sum_sq = 0;
  for(int i = 0; i < image.rows; ++i) {
    for(int j = 0; j < image.cols; ++j) sum_sq += ((image.at<uchar>(i, j) - cv::mean) * (image.at<uchar>(i, j) - cv::mean));
  }
  std_dev = sqrt((double)sum_sq / total_pixels);

  return make_pair(cv::mean, std_dev);
}

cv::Mat_<uchar>
binaryThresholding(const cv::Mat_<uchar>& image, const pair<double, double>& stats) {
  cv::Mat_<uchar> image_binary(image.size());

  double Z = 0.9;
  double cv::threshold = stats.first + (Z * stats.second);
  for(int i = 0; i < image.rows; ++i) {
    for(int j = 0; j < image.cols; ++j) {
      if(image.at<uchar>(i, j) >= cv::threshold + numeric_limits<double>::epsilon())
        image_binary.at<uchar>(i, j) = 255;
      else
        image_binary.at<uchar>(i, j) = 0;
    }
  }
  return image_binary;
}

int
returnLargestContourIndex(std::vector<std::vector<cv::Point>> contours) {
  int max_contour_size = 0;
  int max_contour_idx = -1;
  for(int i = 0; i < contours.size(); ++i) {
    if(contours[i].size() > max_contour_size) {
      max_contour_size = contours[i].size();
      max_contour_idx = i;
    }
  }
  return max_contour_idx;
}
