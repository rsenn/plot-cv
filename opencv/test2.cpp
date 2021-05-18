#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

//using namespace cv;
using namespace std;

int thresh = 100;
cv::Mat source;

void threshold_callback(int, void*);
void setLabel(cv::Mat&, const std::string, std::vector<cv::Point>&);
string intToString(int);

int
main(int argc, char* argv[]) {
  // load image
  source = cv::imread(argv[1]);

  if(!source.data) {
    cerr << "Problem loading image!!!" << endl;
    return EXIT_FAILURE;
  }

  cv::cvtColor(source, source, cv::COLOR_BGR2GRAY);

  cv::namedWindow("Source", cv::WINDOW_AUTOSIZE);
  cv::imshow("Source", source);

  cv::createTrackbar(" Threshold:", "Source", &thresh, 255, threshold_callback);

  threshold_callback(0, 0);

  cv::waitKey(0);
  return (0);
}

/** @function threshold_callback */
void
threshold_callback(int, void*) {

  cv::Mat morph;
  cv::Mat source2 = source.clone();

  // morphological closing with a column filter : retain only large vertical edges
  cv::Mat morphKernelV = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 10));
  // cv::Mat morphKernelV = cv::getStructuringElement( cv::MORPH_RECT, cv::Size( 2*10 + 1, 2*10+1 ), cv::Point( 10, 10 ) );
  cv::morphologyEx(source2, morph, cv::MORPH_CLOSE, morphKernelV);

  cv::Mat bwV;
  // binarize: will contain only large vertical edges
  cv::threshold(morph, bwV, 100, 255.0, cv::THRESH_BINARY | cv::THRESH_OTSU);

  // morphological closing with a row filter : retain only large horizontal edges
  cv::Mat morphKernelH = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(10, 1));
  // cv::Mat morphKernelH = cv::getStructuringElement( cv::MORPH_RECT, cv::Size( 2*10 + 1, 2*10+1 ), cv::Point( 10, 10 ) );
  cv::morphologyEx(source2, morph, cv::MORPH_CLOSE, morphKernelH);

  cv::Mat bwH;
  // binarize: will contain only large horizontal edges
  cv::threshold(morph, bwH, 100, 255.0, cv::THRESH_BINARY | cv::THRESH_OTSU);

  // combine the vertical and horizontal edges
  cv::Mat bw = bwV & bwH;
  cv::threshold(bw, bw, 75, 255.0, cv::THRESH_BINARY_INV);
  cv::imshow("fooo", bw);

  // just for illustration
  cv::Mat rgb;
  cv::cvtColor(source, rgb, cv::COLOR_GRAY2BGR);

  // find contours
  vector<vector<cv::Point>> contours;
  vector<cv::Vec4i> hierarchy;
  cv::findContours(bw, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE, cv::Point(0, 0));
  // filter contours by area to obtain boxes
  double area_min = 315;
  double area_max = 375;

  double area = 0;
  int compt = 0;
  for(int idx = 0; idx >= 0; idx = hierarchy[idx][0]) {
    area = cv::contourArea(contours[idx]);
    cv::Rect rect = cv::boundingRect(contours[idx]);
    // if (rect.width > 17 && rect.height > 17 && area > area_min && area < area_max)
    if(rect.width > 17 && rect.height > 17) {
      compt++;
      cv::drawContours(rgb, contours, idx, cv::Scalar(0, 0, 255), 2, 8, hierarchy);
      string checkbox_label = intToString(rect.y);
      setLabel(rgb, checkbox_label, contours[idx]);
      // take bounding cv::rectangle. better to use filled countour as a mask
      // to extract the cv::rectangle because then you won't get any stray elements
      // cout << " rect: (" << rect.x << ", " << rect.y << ") " << rect.width << " x " << rect.height <<
      // endl;
      cv::Mat imRect(source, rect);
    }
  }
  // cout << compt << endl;

  /// Show in a window
  cv::namedWindow("Contours", cv::WINDOW_AUTOSIZE);
  cv::imshow("Contours", rgb);
}

/**
 * Helper function to display text in the center of a contour
 */
void
setLabel(cv::Mat& im, const std::string label, std::vector<cv::Point>& contour) {
  double scale = 0.4;
  int baseline = 0;
  int fontface = cv::FONT_HERSHEY_SIMPLEX;
  int thickness = 1;

  cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);
  cv::Rect r = cv::boundingRect(contour);

  cv::Point pt(r.x + ((r.width - text.width) / 2), r.y + ((r.height + text.height) / 2));
  cv::rectangle(im, pt + cv::Point(0, baseline), pt + cv::Point(text.width, -text.height), CV_RGB(255, 255, 255), cv::FILLED);
  cv::putText(im, label, pt, fontface, scale, CV_RGB(0, 0, 0), thickness, 8);
}

string
intToString(int number) {
  string result;
  ostringstream convert;
  convert << number;
  result = convert.str();

  return result;
}
