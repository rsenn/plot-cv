/*
  g++ canny.cpp -o canny -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_objdetect

  // Info: http://docs.opencv.org/doc/tutorials/imgproc/shapedescriptors/find_contours/find_contours.html
*/

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdlib.h>
#include <stdio.h>

using namespace cv;

/// Global variables

cv::Mat src, src_gray;
cv::Mat dst, detected_edges;
cv::Mat final;
cv::Mat canny_output, alpha;
std::vector<std::vector<Point>> contours;
std::vector<Vec4i> hierarchy;

int edgeThresh = 1;
int lowThreshold = 99;
int const max_lowThreshold = 100, maxRGB = 255;
int ratio = 3;
int kernel_size = 3;
char* window_name = "Contours";

int red = 0, blue = 255, green = 0, minActivate = 100;
int save = 0;
int fill = 4;

void
doCanny() {
  /// Detect edges using canny
  Canny(src_gray, canny_output, lowThreshold, lowThreshold * 2, 3);
  /// Find contours
  findContours(canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

  /// Draw contours
  final = cv::Mat::zeros(canny_output.size(), CV_8UC4);
  for(int i = 0; i < contours.size(); i++) {
    Scalar color = Scalar(blue, green, red);
    drawContours(final, contours, i, color, fill, 8, hierarchy, 2, Point());
  }

  // Set the Alpha channel to get transparent background
  int size = final.cols * final.rows * final.channels();
  uchar* p = final.data;
  for(int i = 0; i < size; i += final.channels()) {
    // printf("\n[%d] = %d", i, *p);
    if(*p >= minActivate || *(p + 1) >= minActivate || *(p + 2) >= minActivate) {
      *p = blue;
      *(p + 1) = green;
      *(p + 2) = red;
      *(p + 3) = 255; // Alpha -> Activated
    } else {
      *p = 255;
      *(p + 1) = 255;
      *(p + 2) = 255;
      *(p + 3) = 0; // Alpha -> Deactivated
    }
    p += final.channels();
  }

  /// Show in a window
  imshow(window_name, final);
  // printf("\nB: %d, G: %d, R: %d, CHANneLS: %d \n", blue, green, red, final.channels());
}

/**
 * @function CannyThreshold
 * @brief Trackbar callback - Canny thresholds input with a ratio 1:3
 */
void
CannyThreshold(int, void*) {
  doCanny();
}

/**
 * @brief Select R callback - Canny thresholds input with a ratio 1:3
 */
void
setColorRGR(int, void*) {
  doCanny();
}

// Save Image
void
saveImage(int, void*) {
  if(save == 1) {
    imwrite("out.png", final);
  }
}

/** @function main */
int
main(int argc, char** argv) {
  /// Load an image
  src = imread(argv[1]);

  if(!src.data) {
    return -1;
  }

  /// Create a matrix of the same type and size as src (for dst)
  dst.create(src.size(), src.type());

  /// Convert the image to grayscale
  cvtColor(src, src_gray, CV_BGR2GRAY);

  /// Create a window
  namedWindow(window_name, CV_WINDOW_AUTOSIZE);

  /// Create a Trackbar for user to enter threshold
  createTrackbar("Min Threshold:", window_name, &lowThreshold, max_lowThreshold, CannyThreshold);

  // RGB Trackbars
  createTrackbar("RED:", window_name, &red, maxRGB, setColorRGR);
  createTrackbar("GREEN:", window_name, &green, maxRGB, setColorRGR);
  createTrackbar("BLUE:", window_name, &blue, maxRGB, setColorRGR);
  createTrackbar("FILL:", window_name, &fill, maxRGB, setColorRGR); // Fill contour

  // Save Button
  createTrackbar("SAVE", window_name, &save, 1, saveImage);

  /// Show the image
  CannyThreshold(0, 0);

  /// Wait until user exit program by pressing a key
  waitKey(0);

  return 0;
}