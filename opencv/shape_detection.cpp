/*************************************************************
 *
 *                Developed by Farid Huseynov
 *                     ITUNOM UAV Team
 *           	 All rights reserved © 2019
 *
 **************************************************************/

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>
#include <iostream>
/*
 * function to find a cosine of angle between vectors
 * from pt0->pt1 and pt0->pt2
 */
static double
angle(cv::Point pt1, cv::Point pt2, cv::Point pt0) {
  double dx1 = pt1.x - pt0.x;
  double dy1 = pt1.y - pt0.y;
  double dx2 = pt2.x - pt0.x;
  double dy2 = pt2.y - pt0.y;
  return (dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
}

/*
 * function to display text in the center of a contour
 */
void
setLabel(cv::Mat& im, const std::string label, std::vector<cv::Point>& contour) {
  int fontface = cv::FONT_HERSHEY_SIMPLEX;
  double scale = 0.4;
  int thickness = 1;
  int baseline = 0;

  cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);
  cv::Rect r = cv::boundingRect(contour);

  cv::Point pt(r.x + ((r.width - text.width) / 2), r.y + ((r.height + text.height) / 2));
  cv::rectangle(im, pt + cv::Point(0, baseline), pt + cv::Point(text.width, -text.height), CV_RGB(255, 255, 255), cv::FILLED);
  cv::putText(im, label, pt, fontface, scale, CV_RGB(0, 0, 0), thickness, 8);
}

int
main() {
  cv::Mat src = cv::imread("images/shps.jpg");
  // cv::Mat src = cv::imread("F:/ITUNOM 2019/shape_datasets/quarter-circle/4__120.jpg");
  // cv::Mat src = cv::imread("F:/ITUNOM 2019/old shape datasets/dtsts/quarter-circle/Adsýz.png");

  if(src.empty())
    return -1;
  // Apply blurring
  cv::Mat blurred;
  cv::GaussianBlur(src, blurred, cv::Size(3, 3), 0);

  // Convert to grayscale
  cv::Mat gray;
  cv::cvtColor(blurred, gray, cv::COLOR_BGR2GRAY);

  // Use Canny instead of threshold to catch squares with gradient shading
  cv::Mat bw;
  cv::Canny(gray, bw, 80, 120);

  // Find contours
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(bw.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  std::vector<cv::Point> approx;
  cv::Mat dst = src.clone();

  for(int i = 0; i < contours.size(); i++) {
    // Approximate contour with accuracy proportional to the contour perimeter
    cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true) * 0.02, true);

    // Number of vertices of polygonal curve
    int vtc = approx.size();

    // setLabel(dst, std::to_string(approx.size()), contours[i]);

    // 'Convex and with the area more than 100p' objects
    if(std::fabs(cv::contourArea(contours[i])) > 100 && cv::isContourConvex(approx)) {

      if(approx.size() == 3) {
        setLabel(dst, "TRI", contours[i]); // Triangles
      } else if(approx.size() >= 4 && approx.size() <= 12) {

        // Get the cosines of all corners
        std::vector<double> cos;
        for(int j = 2; j < vtc + 1; j++) cos.push_back(angle(approx[j % vtc], approx[j - 2], approx[j - 1]));

        // Sort ascending the cosine values
        std::sort(cos.begin(), cos.end());

        // Get the lowest and the highest cosine
        double mincos = cos.front();
        double maxcos = cos.back();

        // Use the degrees obtained above and the number of vertices to determine the shape of the contour

        // square
        if(vtc == 4 && mincos >= -0.15 && maxcos <= 0.15 && (cv::minAreaRect(contours[i]).size.width / cv::minAreaRect(contours[i]).size.height > 0.8 && cv::minAreaRect(contours[i]).size.width / cv::minAreaRect(contours[i]).size.height < 1.2)) {
          setLabel(dst, "SQR", contours[i]);
        }

        // rectangle
        else if(vtc == 4 && mincos >= -0.15 && maxcos <= 0.15)
          setLabel(dst, "RECT", contours[i]);

        // trapezoid
        else if(vtc == 4 && mincos <= -0.2)
          setLabel(dst, "TRAPEZOID", contours[i]);

        // pentagon
        else if(vtc == 5 && mincos >= -0.5 && maxcos <= -0.05)
          setLabel(dst, "PENTA", contours[i]);

        // semi-circe
        else if(vtc == 5 && mincos >= -0.85 && maxcos <= 0.6)
          setLabel(dst, "SEMI-CIRCLE", contours[i]);

        // quarter-circle
        else if(vtc == 5 && mincos >= -1 && maxcos <= 0.6)
          setLabel(dst, "QUARTER-CIRCLE", contours[i]);

        // hexagon
        else if(vtc == 6 && mincos >= -0.7 && maxcos <= -0.3)
          setLabel(dst, "HEXA", contours[i]);

        // semi-circle
        else if(vtc == 6 && mincos >= -0.85 && maxcos <= 0.6)
          setLabel(dst, "SEMI-CIRCLE", contours[i]);

        // quarter-circle
        else if(vtc == 6 && mincos >= -1 && maxcos <= 0.6)
          setLabel(dst, "QUARTER-CIRCLE", contours[i]);

        // heptagon
        else if(vtc == 7)
          setLabel(dst, "HEPTA", contours[i]);

        // octagon or circle
        else if(vtc == 8) {
          // circle
          double area = cv::contourArea(contours[i]);
          cv::Rect r = cv::boundingRect(contours[i]);
          int radius = r.width / 2;
          if(std::abs(1 - ((double)r.width / r.height)) <= 0.2 && std::abs(1 - (area / (CV_PI * std::pow(radius, 2)))) <= 0.2) {
            setLabel(dst, "CIR", contours[i]);
          }

          // octagon
          else
            setLabel(dst, "OCTA", contours[i]);
        }
      }
    }

    // For the non-convex shapes and circle
    else if(std::fabs(cv::contourArea(contours[i])) > 100) {
      if(vtc == 4)
        setLabel(dst, "QUADRANGULAR", contours[i]);
      if(vtc == 5)
        setLabel(dst, "PENTAGON", contours[i]);
      if(vtc == 6)
        setLabel(dst, "HEXAGON", contours[i]);
      if(vtc == 7)
        setLabel(dst, "HEPTAGON", contours[i]);
      if(vtc == 10)
        setLabel(dst, "STAR", contours[i]);
      else if(vtc == 12)
        setLabel(dst, "CROSS", contours[i]);
      else
        setLabel(dst, "UNKNOWN", contours[i]);
    }
  }
  cv::namedWindow("src", cv::WINDOW_FREERATIO);
  cv::namedWindow("dst", cv::WINDOW_FREERATIO);
  cv::imshow("src", src);
  cv::imshow("dst", dst);
  cv::waitKey(0);
  cv::destroyAllWindows();
  return 0;
}
