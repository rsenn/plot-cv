#include <opencv2/opencv.hpp>

#include <stdio.h>
#include <math.h>
#include <iostream>

//using namespace cv;
using namespace std;

bool
compareContourAreas(std::vector<cv::Point> contour1, std::vector<cv::Point> contour2) {
  double i = fabs(cv::contourArea(cv::Mat(contour1)));
  double j = fabs(cv::contourArea(cv::Mat(contour2)));
  return (i > j);
}

bool
compareXCords(cv::Point p1, cv::Point p2) {
  return (p1.x < p2.x);
}

bool
compareYCords(cv::Point p1, cv::Point p2) {
  return (p1.y < p2.y);
}

bool
compareDistance(pair<cv::Point, cv::Point> p1, pair<Point, cv::Point> p2) {
  return (cv::norm(p1.first - p1.second) < cv::norm(p2.first - p2.second));
}

double
_distance(cv::Point p1, cv::Point p2) {
  return sqrt(((p1.x - p2.x) * (p1.x - p2.x)) + ((p1.y - p2.y) * (p1.y - p2.y)));
}

void
resizeToHeight(cv::Mat src, cv::Mat& dst, int height) {
  cv::Size s = cv::Size(src.cols * (height / double(src.rows)), height);
  cv::resize(src, dst, s, cv::INTER_AREA);
}

void
orderPoints(vector<cv::Point> inpts, vector<Point>& ordered) {
  sort(inpts.begin(), inpts.end(), compareXCords);
  vector<cv::Point> lm(inpts.begin(), inpts.begin() + 2);
  vector<cv::Point> rm(inpts.end() - 2, inpts.end());

  sort(lm.begin(), lm.end(), compareYCords);
  cv::Point tl(lm[0]);
  cv::Point bl(lm[1]);
  vector<pair<cv::Point, cv::Point>> tmp;
  for(size_t i = 0; i < rm.size(); i++) { tmp.push_back(make_pair(tl, rm[i])); }

  sort(tmp.begin(), tmp.end(), compareDistance);
  cv::Point tr(tmp[0].second);
  cv::Point br(tmp[1].second);

  ordered.push_back(tl);
  ordered.push_back(tr);
  ordered.push_back(br);
  ordered.push_back(bl);
}

void
fourPointTransform(cv::Mat src, cv::Mat& dst, vector<cv::Point> pts) {
  vector<cv::Point> ordered_pts;
  orderPoints(pts, ordered_pts);

  double wa = _distance(ordered_pts[2], ordered_pts[3]);
  double wb = _distance(ordered_pts[1], ordered_pts[0]);
  double mw = max(wa, wb);

  double ha = _distance(ordered_pts[1], ordered_pts[2]);
  double hb = _distance(ordered_pts[0], ordered_pts[3]);
  double mh = max(ha, hb);

  cv::Point2f src_[] = {
      cv::Point2f(ordered_pts[0].x, ordered_pts[0].y),
      cv::Point2f(ordered_pts[1].x, ordered_pts[1].y),
      cv::Point2f(ordered_pts[2].x, ordered_pts[2].y),
      cv::Point2f(ordered_pts[3].x, ordered_pts[3].y),
  };
  cv::Point2f dst_[] = {Point2f(0, 0), cv::Point2f(mw - 1, 0), cv::Point2f(mw - 1, mh - 1), Point2f(0, mh - 1)};
  cv::Mat m = cv::getPerspectiveTransform(src_, dst_);
  cv::warpPerspective(src, dst, m, cv::Size(mw, mh));
}

void
preProcess(cv::Mat src, cv::Mat& dst) {
  cv::Mat imageGrayed;
  // cv::Mat imageBlurred1;

  cv::Mat imageOpen, imageClosed, imageBlurred;

  cv::cvtColor(src, imageGrayed, cv::COLOR_BGR2GRAY);

  cv::Mat structuringElmt = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(4, 4));
  cv::morphologyEx(imageGrayed, imageOpen, cv::MORPH_OPEN, structuringElmt);
  cv::morphologyEx(imageOpen, imageClosed, cv::MORPH_CLOSE, structuringElmt);

  cv::GaussianBlur(imageClosed, imageBlurred, cv::Size(7, 7), 0);

  cv::Canny(imageBlurred, dst, 10, 20, 3);
}

string
getOutputFileName(string path, string name) {
  std::string fname, ext;

  size_t sep = path.find_last_of("\\/");
  if(sep != std::string::npos) {
    path = path.substr(sep + 1, path.size() - sep - 1);

    size_t dot = path.find_last_of(".");
    if(dot != std::string::npos) {
      fname = path.substr(0, dot);
      ext = path.substr(dot, path.size() - dot);
    } else {
      fname = path;
      ext = "";
    }
  }

  return fname + "_" + name + ext;
}

int
main(int argc, char** argv) {

  // static const char * const keys = "{ i |image| }";
  // cv::CommandLineParser parser(argc, argv, keys);

  // string image_name(parser.get<cv::String>("image"));

  // if (image_name.empty())
  // {
  //     // parser.printParams();
  //     return -1;
  // }

  // cv::Mat image = cv::imread(image_name);
  // if (image.empty())
  // {
  // 	printf("Cannot cv::read image file: %s\n", image_name.c_str());
  // 	return -1;
  // }

  cv::Mat image = cv::imread("sample.jpeg");
  string image_name = "sample.jpeg";

  if(!image.data) {
    std::cout << "Could not open or find the image" << std::endl;
    return -1;
  }

  double ratio = image.rows / 500.0;
  cv::Mat orig = image.clone();
  resizeToHeight(image, image, 500);

  cv::Mat gray, edged, warped;
  preProcess(image, edged);
#ifndef NDEBUG
  cv::imwrite(getOutputFileName(image_name, "edged.jpeg"), edged);
#endif

  vector<vector<cv::Point>> contours;
  vector<cv::Vec4i> hierarchy;
  vector<vector<cv::Point>> approx;
  cv::findContours(edged, contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

  approx.cv::resize(contours.size());
  size_t i, j;
  for(i = 0; i < contours.size(); i++) {
    double peri = cv::arcLength(contours[i], true);
    cv::approxPolyDP(contours[i], approx[i], 0.02 * peri, true);
  }
  sort(approx.begin(), approx.end(), compareContourAreas);

  for(i = 0; i < approx.size(); i++) {
    cv::drawContours(image, approx, i, cv::Scalar(255, 255, 0), 2);
    if(approx[i].size() == 4) {
      break;
    }
  }

  if(i < approx.size()) {
    cv::drawContours(image, approx, i, cv::Scalar(0, 255, 0), 2);
#ifndef NDEBUG
    cv::imwrite(getOutputFileName(image_name, "outline.jpeg"), image);
#endif
    for(j = 0; j < approx[i].size(); j++) { approx[i][j] *= ratio; }

    fourPointTransform(orig, warped, approx[i]);
#ifndef NDEBUG
    cv::imwrite(getOutputFileName(image_name, "flat.jpeg"), warped);
#endif
    cv::cvtColor(warped, warped, cv::COLOR_BGR2GRAY, 1);
    cv::adaptiveThreshold(warped, warped, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 9, 15);
    cv::GaussianBlur(warped, warped, cv::Size(3, 3), 0);
    cv::imwrite(getOutputFileName(image_name, "scanned.jpeg"), warped);
  }
}
