#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include "../src/polygon.hpp"
#include "../src/data.hpp"

// using namespace cv;
using namespace std;

const float STEP = 5;
const SegmentationMode mode = ALL_MODE;
const string WIN_NAME = "images";

cv::Mat float2byte(const cv::Mat& If);

void
onMouse1(int event, int x, int y, int foo, void* p) {
  if(event != cv::EVENT_LBUTTONDOWN)
    return;
  cv::Point m1(x, y);
  Data* D = (Data*)p;
  D->polygon.add_point(m1);
  cout << "Adding point: (" << x << ", " << y << ")" << endl;
  D->polygon.draw_polygon(D->image.clone());
}

int
main() {
  // cv::Mat A=cv::imread("../segmentation.png");
  // cv::Mat A = cv::imread("test-images/pieuvre.jpg");
  // cv::Mat A = cv::imread("test-images/eiffel.jpg");
  cv::Mat A = cv::imread("test-images/starfish.jpg");
  cv::namedWindow(WIN_NAME);
  cv::imshow(WIN_NAME, A);
  Data* D = new Data(A);
  bool modifyPolygon = true;
  cv::imshow("gGradient", float2byte(D->gGradient));
  cv::imshow("gx", float2byte(D->gx));
  cv::imshow("gy", float2byte(D->gy));
  cout << "Gradient computing done, please define polygon." << endl;
  cv::imshow(WIN_NAME, float2byte(D->image));
  cv::setMouseCallback(WIN_NAME, onMouse1, D);
  cv::waitKey();
  cout << "Polygon definition is over" << endl;
  cv::setMouseCallback(WIN_NAME, NULL, NULL);
  cv::Mat Imcloned;
  // force ballon:
  while(true) {
    cout << "Gradient descent ...";
    D->draw_next_step(STEP, D->image.clone(), mode);
    cv::waitKey();
    D->find_contour(STEP, mode);
    // cout << "found contour" << endl;
    D->polygon.draw_polygon(D->image.clone());
    cv::waitKey();
    cout << "Done." << endl;
  }
  return 0;
}

cv::Mat
float2byte(const cv::Mat& If) {
  double minVal, maxVal;
  cv::minMaxLoc(If, &minVal, &maxVal);
  cv::Mat Ib;
  If.convertTo(Ib, CV_8U, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));
  return Ib;
}
