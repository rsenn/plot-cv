#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>
#include <iostream>

using namespace cv;

/**
 * Helper function to find a cosine of angle between vectors
 * from pt0->pt1 and pt0->pt2
 */
static double
angle(Point pt1, Point pt2, Point pt0) {
  double dx1 = pt1.x - pt0.x;
  double dy1 = pt1.y - pt0.y;
  double dx2 = pt2.x - pt0.x;
  double dy2 = pt2.y - pt0.y;

  return (dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
}

/**
 * Helper function to display text in the center of a contour
 */
void
setLabel(Mat& im, const std::string label, std::vector<Point>& contour) {
  double scale = 0.4;
  int baseline = 0;
  int fontface = FONT_HERSHEY_SIMPLEX;
  int thickness = 1;

  Size text = getTextSize(label, fontface, scale, thickness, &baseline);
  Rect r = boundingRect(contour);

  Point pt(r.x + ((r.width - text.width) / 2), r.y + ((r.height + text.height) / 2));
  rectangle(im, pt + Point(0, baseline), pt + Point(text.width, -text.height), CV_RGB(255, 255, 255), cv::FILLED);
  putText(im, label, pt, fontface, scale, CV_RGB(0, 0, 0), thickness, 8);
}

int
main(int argc, char** argv) {
  Mat img_src, img_gray, img_bin, img_dest;

  // Read source image
  img_src = imread(argv[1], 1);
  if(img_src.empty())
    return -1;

  // Convert to grayscale
  cvtColor(img_src, img_gray, cv::COLOR_BGR2GRAY);

  // Convert to binary image using Canny
  Canny(img_gray, img_bin, 0, 50, 5);

  // Find contours
  std::vector<std::vector<Point>> contours;
  cv::findContours(img_bin.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  // The array for storing the approximation curve
  std::vector<cv::Point> approx;

  // We'll put the labels in this destination image
  img_dest = img_src.clone();

  for(int i = 0; i < contours.size(); i++) {
    // Approximate contour with accuracy proportional to the contour perimeter
    approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true) * 0.02, true);

    // Skip small or non-convex objects
    if(std::fabs(contourArea(contours[i])) < 100 || !isContourConvex(approx))
      continue;

    if(approx.size() == 3)
      setLabel(img_dest, "TRI", contours[i]); // Triangles

    else if(approx.size() >= 4 && approx.size() <= 6) {
      // Number of vertices of polygonal curve
      int vtc = approx.size();

      // Get the cosines of all corners
      std::vector<double> cos;
      for(int j = 2; j < vtc + 1; j++) cos.push_back(angle(approx[j % vtc], approx[j - 2], approx[j - 1]));

      // Sort ascending the cosine values
      std::sort(cos.begin(), cos.end());

      // Get the lowest and the highest cosine
      double mincos = cos.front();
      double maxcos = cos.back();

      // Use the degrees obtained above and the number of vertices to determine the shape of the contour
      if(vtc == 4 && mincos >= -0.1 && maxcos <= 0.3)
        setLabel(img_dest, "RECT", contours[i]);
      else if(vtc == 5 && mincos >= -0.34 && maxcos <= -0.27)
        setLabel(img_dest, "PENTA", contours[i]);
      else if(vtc == 6 && mincos >= -0.55 && maxcos <= -0.45)
        setLabel(img_dest, "HEXA", contours[i]);
    } else {
      // Detect and label circles
      double area = contourArea(contours[i]);
      Rect r = boundingRect(contours[i]);
      int radius = r.width / 2;

      if(std::abs(1 - ((double)r.width / r.height)) <= 0.2 && std::abs(1 - (area / (CV_PI * std::pow(radius, 2.0)))) <= 0.2)
        setLabel(img_dest, "CIR", contours[i]);
    }
  }

  imshow("src", img_src);
  imshow("dst", img_dest);
  // imshow( "gray", img_gray );
  // imshow( "binary", img_bin );
  waitKey(0);

  return 0;
}
