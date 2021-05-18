#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

//using namespace cv;
using namespace std;

int
main(int argc, char** argv) {
  cv::Mat src, dst, color_dst;
  if(argc != 2 || !(src = cv::imread(argv[1], 0)).data)
    return -1;

  cv::Canny(src, dst, 50, 200, 3);
  cv::cvtColor(dst, color_dst, cv::COLOR_GRAY2BGR);

  vector<cv::Vec4i> lines;
  cv::HoughLinesP(dst, lines, 1, CV_PI / 180, 80, 30, 10);
  for(size_t i = 0; i < lines.size(); i++) {
    cv::line(color_dst, cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0, 0, 255), 3, 8);
  }
  cv::namedWindow("Source", 1);
  cv::imshow("Source", src);

  cv::namedWindow("Detected Lines", 1);
  cv::imshow("Detected Lines", color_dst);

  cv::waitKey(0);
  return 0;
}
