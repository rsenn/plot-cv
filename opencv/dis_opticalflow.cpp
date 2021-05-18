
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video.hpp>

using namespace std;
//using namespace cv;

static void
help() {
  printf("Usage: dis_optflow <video_file>\n");
}

int
main(int argc, char** argv) {
  cv::VideoCapture cap;

  if(argc < 2) {
    help();
    exit(1);
  }

  cap.open(argv[1]);
  if(!cap.isOpened()) {
    printf("ERROR: Cannot open file %s\n", argv[1]);
    return -1;
  }

  cv::Mat prevgray, gray, rgb, frame;
  cv::Mat flow, flow_uv[2];
  cv::Mat mag, ang;
  cv::Mat hsv_split[3], hsv;
  char ret;

  cv::namedWindow("flow", 1);
  cv::namedWindow("orig", 1);

  cv::Ptr<cv::DenseOpticalFlow> algorithm = cv::DISOpticalFlow::create(DISOpticalFlow::PRESET_MEDIUM);

  while(true) {
    cap >> frame;
    if(frame.empty())
      break;

    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    if(!prevgray.empty()) {
      algorithm->calc(prevgray, gray, flow);
      cv::split(flow, flow_uv);
      cv::multiply(flow_uv[1], -1, flow_uv[1]);
      cv::cartToPolar(flow_uv[0], flow_uv[1], mag, ang, true);
      cv::normalize(mag, mag, 0, 1, cv::NORM_MINMAX);
      hsv_split[0] = ang;
      hsv_split[1] = mag;
      hsv_split[2] = cv::Mat::ones(ang.size(), ang.type());
      cv::merge(hsv_split, 3, hsv);
      cv::cvtColor(hsv, rgb, cv::COLOR_HSV2BGR);
      cv::imshow("flow", rgb);
      cv::imshow("orig", frame);
    }

    if((ret = (char)cv::waitKey(20)) > 0)
      break;
    std::cv::swap(prevgray, gray);
  }

  return 0;
}
