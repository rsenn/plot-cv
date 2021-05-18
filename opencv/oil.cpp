#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/cv::xphoto.hpp>
#include <opencv2/cv::xphoto/oilpainting.hpp>
#include <iostream>

//using namespace cv;
using namespace std;

static void TrackSlider(int, void*);
static void addSlider(cv::String sliderName,
                      cv::String windowName,
                      int minSlider,
                      int maxSlider,
                      int valDefault,
                      int* valSlider,
                      void (*f)(int, void*),
                      void* r);
vector<int> colorSpace = {COLOR_BGR2GRAY, COLOR_BGR2HSV, COLOR_BGR2YUV, COLOR_BGR2XYZ};

struct OilImage {
  cv::String winName = "Oil painting";
  int size;
  int dynRatio;
  int colorSpace;
  cv::Mat img;
};

const cv::String keys = "{Help h usage ? help  |     | Print this message   }"
                    "{v                    | 0   | video index }"
                    "{a                    | 700   | API index }"
                    "{s                    | 10   | neighbouring size }"
                    "{d                    | 1   | dynamic ratio }"
                    "{c                    | 0   | color space }"
                    "{@arg1                |     | file path}";

int
main(int argc, char* argv[]) {
  cv::CommandLineParser parser(argc, argv, keys);

  if(parser.has("help")) {
    parser.printMessage();
    return 0;
  }
  cv::String filename = parser.get<String>(0);
  OilImage p;
  p.dynRatio = parser.get<int>("d");
  p.size = parser.get<int>("s");
  p.colorSpace = parser.get<int>("c");
  if(p.colorSpace < 0 || p.colorSpace >= static_cast<int>(colorSpace.size())) {
    std::cout << "Color space must be >= 0 and <" << colorSpace.size() << "\n";
    return EXIT_FAILURE;
  }
  if(!filename.empty()) {
    p.img = cv::imread(filename);
    if(p.img.empty()) {
      std::cout << "Check file path!\n";
      return EXIT_FAILURE;
    }
    cv::Mat dst;
    cv::xphoto::oilPainting(p.img, dst, p.size, p.dynRatio, colorSpace[p.colorSpace]);
    cv::imshow("oil painting effect", dst);
    cv::waitKey();
    return 0;
  }
  cv::VideoCapture v(parser.get<int>("v") + parser.get<int>("a"));
  v >> p.img;
  p.winName = "Oil Painting";
  cv::namedWindow(p.winName);
  addSlider("DynRatio", p.winName, 1, 127, p.dynRatio, &p.dynRatio, TrackSlider, &p);
  addSlider("cv::Size", p.winName, 1, 100, p.size, &p.size, TrackSlider, &p);
  addSlider("ColorSpace", p.winName, 0, static_cast<int>(colorSpace.size() - 1), p.colorSpace, &p.colorSpace, TrackSlider, &p);
  while(cv::waitKey(20) != 27) {
    v >> p.img;
    cv::imshow("Original", p.img);
    TrackSlider(0, &p);
    cv::waitKey(10);
  }
  return 0;
}

void
addSlider(cv::String sliderName,
          cv::String windowName,
          int minSlider,
          int maxSlider,
          int valDefault,
          int* valSlider,
          void (*f)(int, void*),
          void* r) {
  cv::createTrackbar(sliderName, windowName, valSlider, 1, f, r);
  cv::setTrackbarMin(sliderName, windowName, minSlider);
  cv::setTrackbarMax(sliderName, windowName, maxSlider);
  cv::setTrackbarPos(sliderName, windowName, valDefault);
}

void
TrackSlider(int, void* r) {
  OilImage* p = (OilImage*)r;
  cv::Mat dst;
  p->img = p->img / p->dynRatio;
  p->img = p->img * p->dynRatio;
  cv::xphoto::oilPainting(p->img, dst, p->size, p->dynRatio, colorSpace[p->colorSpace]);
  if(!dst.empty()) {
    cv::imshow(p->winName, dst);
  }
}
