#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <stdio.h>
//using namespace cv;
using namespace std;
int edgeThresh = 1;
int edgeThreshScharr = 1;
cv::Mat image, gray, blurImage, edge1, edge2, cedge;
const char* window_name1 = "Edge map : cv::Canny default (cv::Sobel gradient)";
const char* window_name2 = "Edge map : cv::Canny with custom gradient (cv::Scharr)";
// define a trackbar callback
static void
onTrackbar(int, void*) {
  cv::blur(gray, blurImage, cv::Size(3, 3));
  // Run the edge detector on grayscale
  cv::Canny(blurImage, edge1, edgeThresh, edgeThresh * 3, 3);
  cedge = cv::Scalar::all(0);
  image.copyTo(cedge, edge1);
  cv::imshow(window_name1, cedge);
  cv::Mat dx, dy;
  cv::Scharr(blurImage, dx, CV_16S, 1, 0);
  cv::Scharr(blurImage, dy, CV_16S, 0, 1);
  cv::Canny(dx, dy, edge2, edgeThreshScharr, edgeThreshScharr * 3);
  cedge = cv::Scalar::all(0);
  image.copyTo(cedge, edge2);
  cv::imshow(window_name2, cedge);
}
static void
help() {
  printf("\nThis sample demonstrates cv::Canny edge detection\n"
         "Call:\n"
         "    /.edge [image_name -- Default is fruits.jpg]\n\n");
}
const char* keys = {"{help h||}{@image |fruits.jpg|input image name}"};
int
main(int argc, const char** argv) {
  help();
  cv::CommandLineParser parser(argc, argv, keys);
  string filename = parser.get<string>(0);
  image = cv::imread(filename, cv::IMREAD_COLOR);
  if(image.empty()) {
    printf("Cannot cv::read image file: %s\n", filename.c_str());
    help();
    return -1;
  }
  cedge.create(image.size(), image.type());
  cv::cvtColor(image, gray, COLOR_BGR2GRAY);
  // Create a window
  cv::namedWindow(window_name1, 1);
  cv::namedWindow(window_name2, 1);
  // create a toolbar
  cv::createTrackbar("cv::Canny cv::threshold default", window_name1, &edgeThresh, 100, onTrackbar);
  cv::createTrackbar("cv::Canny cv::threshold cv::Scharr", window_name2, &edgeThreshScharr, 400, onTrackbar);
  // Show the image
  onTrackbar(0, 0);
  // Wait for a key stroke; the same function arranges events processing
  cv::waitKey(0);
  return 0;
}
