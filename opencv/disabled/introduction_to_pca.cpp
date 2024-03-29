/**
 * @file introduction_to_pca.cpp
 * @brief This program demonstrates how to use OpenCV PCA to extract the orienation of an object
 * @author OpenCV team
 */

#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
// using namespace cv;

// Function declarations
void drawAxis(cv::Mat&, cv::Point, cv::Point, cv::Scalar, const float);
double getOrientation(const std::vector<cv::Point>&, cv::Mat&);

/**
 * @function drawAxis
 */
void
drawAxis(cv::Mat& img, cv::Point p, cv::Point q, cv::Scalar colour, const float scale = 0.2) {
  //! [visualization1]
  double angle;
  double hypotenuse;
  angle = atan2((double)p.y - q.y, (double)p.x - q.x); // angle in radians
  hypotenuse = sqrt((double)(p.y - q.y) * (p.y - q.y) + (p.x - q.x) * (p.x - q.x));
  //    double degrees = angle * 180 / CV_PI; // convert radians to degrees (0-180 range)
  //    cout << "Degrees: " << abs(degrees - 180) << endl; // angle in 0-360 degrees range

  // Here we lengthen the arrow by a factor of scale
  q.x = (int)(p.x - scale * hypotenuse * cos(angle));
  q.y = (int)(p.y - scale * hypotenuse * sin(angle));
  cv::line(img, p, q, colour, 1, cv::LINE_AA);

  // create the arrow hooks
  p.x = (int)(q.x + 9 * cos(angle + CV_PI / 4));
  p.y = (int)(q.y + 9 * sin(angle + CV_PI / 4));
  cv::line(img, p, q, colour, 1, cv::LINE_AA);

  p.x = (int)(q.x + 9 * cos(angle - CV_PI / 4));
  p.y = (int)(q.y + 9 * sin(angle - CV_PI / 4));
  cv::line(img, p, q, colour, 1, cv::LINE_AA);
  //! [visualization1]
}

/**
 * @function getOrientation
 */
double
getOrientation(const std::vector<cv::Point>& pts, cv::Mat& img) {
  //! [pca]
  // Construct a buffer used by the pca analysis
  int sz = static_cast<int>(pts.size());
  cv::Mat data_pts = cv::Mat(sz, 2, CV_64FC1);
  for(int i = 0; i < data_pts.rows; ++i) {
    data_pts.at<double>(i, 0) = pts[i].x;
    data_pts.at<double>(i, 1) = pts[i].y;
  }

  // Perform PCA analysis
  PCA pca_analysis(data_pts, cv::Mat(), CV_PCA_DATA_AS_ROW);

  // Store the center of the object
  cv::Point cntr =
      cv::Point(static_cast<int>(pca_analysis.cv::mean.at<double>(0, 0)), static_cast<int>(pca_analysis.mean.at<double>(0, 1)));

  // Store the eigenvalues and eigenstd::vectors
  std::vector<cv::Point2d> eigen_vecs(2);
  std::vector<double> eigen_val(2);
  for(int i = 0; i < 2; ++i) {
    eigen_vecs[i] =
        cv::Point2d(pca_analysis.eigenstd::vectors.at<double>(i, 0), pca_analysis.eigenstd::vectors.at<double>(i, 1));

    eigen_val[i] = pca_analysis.eigenvalues.at<double>(0, i);
  }

  //! [pca]
  //! [visualization]
  // Draw the principal components
  cv::circle(img, cntr, 3, cv::Scalar(255, 0, 255), 2);
  cv::Point p1 = cntr + 0.02 * cv::Point(static_cast<int>(eigen_vecs[0].x * eigen_val[0]),
                                         static_cast<int>(eigen_vecs[0].y * eigen_val[0]));
  cv::Point p2 = cntr - 0.02 * cv::Point(static_cast<int>(eigen_vecs[1].x * eigen_val[1]),
                                         static_cast<int>(eigen_vecs[1].y * eigen_val[1]));
  drawAxis(img, cntr, p1, cv::Scalar(0, 255, 0), 1);
  drawAxis(img, cntr, p2, cv::Scalar(255, 255, 0), 5);

  double angle = atan2(eigen_vecs[0].y, eigen_vecs[0].x); // orientation in radians
  //! [visualization]

  return angle;
}

/**
 * @function main
 */
int
main(int, char** argv) {
  //! [pre-process]
  // Load image
  //    cv::Mat src = cv::imread("pca_test1.jpg");
  cv::Mat src = cv::imread(argv[1]);

  // Check if image is loaded successfully
  if(!src.data || src.empty()) {
    cout << "Problem loading image!!!" << endl;
    return EXIT_FAILURE;
  }

  cv::imshow("src", src);

  // Convert image to grayscale
  cv::Mat gray;
  cv::cvtColor(src, gray, COLOR_BGR2GRAY);

  // Convert image to binary
  cv::Mat bw;
  cv::threshold(gray, bw, 50, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
  //! [pre-process]

  //! [contours]
  // Find all the contours in the thresholded image
  std::vector<cv::Vec4i> hierarchy;
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(bw, contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

  for(size_t i = 0; i < contours.size(); ++i) {
    // Calculate the area of each contour
    double area = cv::contourArea(contours[i]);
    // Ignore contours that are too small or too large
    if(area < 1e2 || 1e5 < area)
      continue;

    // Draw each contour only for visualisation purposes
    cv::drawContours(src, contours, static_cast<int>(i), cv::Scalar(0, 0, 255), 2, 8, hierarchy, 0);
    // Find the orientation of each shape
    getOrientation(contours[i], src);
  }
  //! [contours]

  cv::imshow("output", src);

  cv::waitKey(0);
  return 0;
}
