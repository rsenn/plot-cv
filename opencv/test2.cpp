#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int thresh = 100;
Mat source;

void threshold_callback(int, void*);
void setLabel(Mat&, const std::string, std::vector<Point>&);
string intToString(int);

int
main(int argc, char* argv[]) {
  // load image
  source = imread(argv[1]);

  if(!source.data) {
    cerr << "Problem loading image!!!" << endl;
    return EXIT_FAILURE;
  }

  cvtColor(source, source, cv::COLOR_BGR2GRAY);

  namedWindow("Source", cv::WINDOW_AUTOSIZE);
  imshow("Source", source);

  createTrackbar(" Threshold:", "Source", &thresh, 255, threshold_callback);

  threshold_callback(0, 0);

  waitKey(0);
  return (0);
}

/** @function threshold_callback */
void
threshold_callback(int, void*) {

  Mat morph;
  Mat source2 = source.clone();

  // morphological closing with a column filter : retain only large vertical edges
  Mat morphKernelV = getStructuringElement(MORPH_RECT, Size(1, 10));
  // Mat morphKernelV = getStructuringElement( MORPH_RECT, Size( 2*10 + 1, 2*10+1 ), Point( 10, 10 ) );
  morphologyEx(source2, morph, MORPH_CLOSE, morphKernelV);

  Mat bwV;
  // binarize: will contain only large vertical edges
  threshold(morph, bwV, 100, 255.0, cv::THRESH_BINARY | cv::THRESH_OTSU);

  // morphological closing with a row filter : retain only large horizontal edges
  Mat morphKernelH = getStructuringElement(MORPH_RECT, Size(10, 1));
  // Mat morphKernelH = getStructuringElement( MORPH_RECT, Size( 2*10 + 1, 2*10+1 ), Point( 10, 10 ) );
  morphologyEx(source2, morph, MORPH_CLOSE, morphKernelH);

  Mat bwH;
  // binarize: will contain only large horizontal edges
  threshold(morph, bwH, 100, 255.0, cv::THRESH_BINARY | cv::THRESH_OTSU);

  // combine the vertical and horizontal edges
  Mat bw = bwV & bwH;
  threshold(bw, bw, 75, 255.0, cv::THRESH_BINARY_INV);
  imshow("fooo", bw);

  // just for illustration
  Mat rgb;
  cvtColor(source, rgb, cv::COLOR_GRAY2BGR);

  // find contours
  vector<vector<Point>> contours;
  vector<Vec4i> hierarchy;
  findContours(bw, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE, Point(0, 0));
  // filter contours by area to obtain boxes
  double area_min = 315;
  double area_max = 375;

  double area = 0;
  int compt = 0;
  for(int idx = 0; idx >= 0; idx = hierarchy[idx][0]) {
    area = contourArea(contours[idx]);
    Rect rect = boundingRect(contours[idx]);
    // if (rect.width > 17 && rect.height > 17 && area > area_min && area < area_max)
    if(rect.width > 17 && rect.height > 17) {
      compt++;
      drawContours(rgb, contours, idx, Scalar(0, 0, 255), 2, 8, hierarchy);
      string checkbox_label = intToString(rect.y);
      setLabel(rgb, checkbox_label, contours[idx]);
      // take bounding rectangle. better to use filled countour as a mask
      // to extract the rectangle because then you won't get any stray elements
      // cout << " rect: (" << rect.x << ", " << rect.y << ") " << rect.width << " x " << rect.height <<
      // endl;
      Mat imRect(source, rect);
    }
  }
  // cout << compt << endl;

  /// Show in a window
  namedWindow("Contours", cv::WINDOW_AUTOSIZE);
  imshow("Contours", rgb);
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
  rectangle(
      im, pt + Point(0, baseline), pt + Point(text.width, -text.height), CV_RGB(255, 255, 255), cv::FILLED);
  putText(im, label, pt, fontface, scale, CV_RGB(0, 0, 0), thickness, 8);
}

string
intToString(int number) {
  string result;
  ostringstream convert;
  convert << number;
  result = convert.str();

  return result;
}
