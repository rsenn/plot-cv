/********************************************************************************
 *
 *
 *  This program is demonstration for cv::ellipse fitting. Program finds
 *  contours and approximate it by ellipses.
 *
 *  Trackbar specify cv::threshold parametr.
 *
 *  White lines is contours. Red lines is fitting ellipses.
 *
 *
 *  Autor:  Denis Burenkov.
 *
 *
 *
 ********************************************************************************/
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
//using namespace cv;
using namespace std;

// static void help()
// {
//     cout <<
//             "\nThis program is demonstration for cv::ellipse fitting. The program finds\n"
//             "contours and approximate it by ellipses.\n"
//             "Call:\n"
//             "./fitellipse [image_name -- Default stuff.jpg]\n" << endl;
// }

int sliderPos = 70;

cv::Mat image;

void processImage(int, void*);

int
main(int argc, char** argv) {
  const char* filename = argc == 2 ? argv[1] : (char*)"stuff.jpg";
  image = cv::imread(filename, 0);
  if(image.empty()) {
    cout << "Couldn't open image " << filename << "\nUsage: fitellipse <image_name>\n";
    return 0;
  }

  cv::imshow("source", image);
  cv::namedWindow("result", 1);

  // Create toolbars. HighGUI use.
  cv::createTrackbar("cv::threshold", "result", &sliderPos, 255, processImage);
  processImage(0, 0);

  // Wait for a key stroke; the same function arranges events processing
  cv::waitKey();
  return 0;
}

// Define trackbar callback functon. This function find contours,
// draw it and approximate it by ellipses.
void
processImage(int /*h*/, void*) {
  std::vector<std::vector<cv::Point>> contours;
  cv::Mat bimage = image >= sliderPos;

  cv::findContours(bimage, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

  cv::Mat cimage = cv::Mat::zeros(bimage.size(), CV_8UC3);

  for(size_t i = 0; i < contours.size(); i++) {
    size_t count = contours[i].size();
    if(count < 6)
      continue;

    cv::Mat pointsf;
    cv::Mat(contours[i]).convertTo(pointsf, CV_32F);
    cv::RotatedRect box = cv::fitEllipse(pointsf);

    if(MAX(box.size.width, box.size.height) > MIN(box.size.width, box.size.height) * 30)
      continue;
    cv::drawContours(cimage, contours, (int)i, cv::Scalar::all(255), 1, 8);

    cv::ellipse(cimage, box, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    cv::ellipse(cimage, box.center, box.size * 0.5f, box.angle, 0, 360, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
    cv::Point2f vtx[4];
    box.points(vtx);
    for(int j = 0; j < 4; j++) cv::line(cimage, vtx[j], vtx[(j + 1) % 4], cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
  }

  cv::imshow("result", cimage);
}
