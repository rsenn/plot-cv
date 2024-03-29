#include <fstream>
#include <sstream>
#include <iostream>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

// using namespace cv;
using namespace cv::dnn;
using namespace std;

struct Net_config {
  float confThreshold; // Confidence threshold
  float nmsThreshold;  // Non-maximum suppression threshold
  int inpWidth;        // Width of network's input image
  int inpHeight;       // Height of network's input image
  string classesFile;
  string modelConfiguration;
  string modelWeights;
  string netname;
};

class YOLO {
public:
  YOLO(Net_config config);
  void detect(cv::Mat& frame);

private:
  float confThreshold;
  float nmsThreshold;
  int inpWidth;
  int inpHeight;
  char netname[20];
  vector<string> classes;
  Net net;
  void postprocess(cv::Mat& frame, const vector<cv::Mat>& outs);
  void drawPred(int classId, float conf, int left, int top, int right, int bottom, cv::Mat& frame);
};

Net_config yolo_nets[4] = {{0.5, 0.4, 416, 416, "coco.names", "yolov3.cfg", "yolov3.weights", "yolov3"},
                           {0.5, 0.4, 608, 608, "coco.names", "yolov4.cfg", "yolov4.weights", "yolov4"},
                           {0.5, 0.4, 320, 320, "coco.names", "yolo-fastest-xl.cfg", "yolo-fastest-xl.weights", "yolo-fastest"},
                           {0.5, 0.4, 320, 320, "coco.names", "csdarknet53s-panet-spp.cfg", "yolobile.weights", "yolobile"}};
