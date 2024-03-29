#include <fstream>
#include <sstream>

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "common.hpp"

std::string keys = "{ help  h     | | Print help message. }"
                   "{ @alias      | | An alias name of model to extract preprocessing parameters from models.yml "
                   "file. }"
                   "{ zoo         | models.yml | An optional path to file with preprocessing parameters }"
                   "{ device      |  0 | camera device number. }"
                   "{ input i     | | Path to input image or video file. Skip this argument to capture frames "
                   "from a camera. }"
                   "{ framework f | | Optional name of an origin framework of the model. Detect it automatically "
                   "if it does not set. }"
                   "{ classes     | | Optional path to a text file with names of classes. }"
                   "{ colors      | | Optional path to a text file with colors for an every class. "
                   "An every color is represented with three values from 0 to 255 in BGR channels order. }"
                   "{ backend     | 0 | Choose one of computation backends: "
                   "0: automatically (by default), "
                   "1: Halide language (http://halide-lang.org/), "
                   "2: Intel's Deep Learning Inference Engine (https://software.intel.com/openvino-toolkit), "
                   "3: OpenCV implementation }"
                   "{ target      | 0 | Choose one of target computation devices: "
                   "0: CPU target (by default), "
                   "1: OpenCL, "
                   "2: OpenCL fp16 (half-float precision), "
                   "3: VPU }";

// using namespace cv;
using namespace cv::dnn;

std::vector<std::string> classes;
std::vector<Vec3b> colors;

void showLegend();

void colorizeSegmentation(const cv::Mat& score, cv::Mat& segm);

int
main(int argc, char** argv) {
  cv::CommandLineParser parser(argc, argv, keys);

  const std::string modelName = parser.get<cv::String>("@alias");
  const std::string zooFile = parser.get<cv::String>("zoo");

  keys += genPreprocArguments(modelName, zooFile);

  parser = cv::CommandLineParser(argc, argv, keys);
  parser.about("Use this script to run semantic segmentation deep learning networks using OpenCV.");
  if(argc == 1 || parser.has("help")) {
    parser.printMessage();
    return 0;
  }

  float scale = parser.get<float>("scale");
  cv::Scalar cv::mean = parser.get<Scalar>("mean");
  bool swapRB = parser.get<bool>("rgb");
  int inpWidth = parser.get<int>("width");
  int inpHeight = parser.get<int>("height");
  cv::String model = findFile(parser.get<String>("model"));
  cv::String config = findFile(parser.get<String>("config"));
  cv::String framework = parser.get<String>("framework");
  int backendId = parser.get<int>("backend");
  int targetId = parser.get<int>("target");

  // Open file with classes names.
  if(parser.has("classes")) {
    std::string file = parser.get<cv::String>("classes");
    std::ifstream ifs(file.c_str());
    if(!ifs.is_open())
      CV_Error(cv::Error::StsError, "File " + file + " not found");
    std::string cv::line;
    while(std::getline(ifs, cv::line)) { classes.push_back(line); }
  }

  // Open file with colors.
  if(parser.has("colors")) {
    std::string file = parser.get<cv::String>("colors");
    std::ifstream ifs(file.c_str());
    if(!ifs.is_open())
      CV_Error(cv::Error::StsError, "File " + file + " not found");
    std::string cv::line;
    while(std::getline(ifs, cv::line)) {
      std::istringstream colorStr(cv::line.c_str());

      Vec3b color;
      for(int i = 0; i < 3 && !colorStr.eof(); ++i) colorStr >> color[i];
      colors.push_back(color);
    }
  }

  if(!parser.check()) {
    parser.printErrors();
    return 1;
  }

  CV_Assert(!model.empty());
  //! [Read and initialize network]
  Net net = readNet(model, config, framework);
  net.setPreferableBackend(backendId);
  net.setPreferableTarget(targetId);
  //! [Read and initialize network]

  // Create a window
  static const std::string kWinName = "Deep learning semantic segmentation in OpenCV";
  cv::namedWindow(kWinName, WINDOW_NORMAL);

  //! [Open a video file or an image file or a camera stream]
  cv::VideoCapture cap;
  if(parser.has("input"))
    cap.open(parser.get<cv::String>("input"));
  else
    cap.open(parser.get<int>("device"));
  //! [Open a video file or an image file or a camera stream]

  // Process frames.
  cv::Mat frame, blob;
  while(cv::waitKey(1) < 0) {
    cap >> frame;
    if(frame.empty()) {
      cv::waitKey();
      break;
    }

    //! [Create a 4D blob from a frame]
    blobFromImage(frame, blob, scale, cv::Size(inpWidth, inpHeight), cv::mean, swapRB, false);
    //! [Create a 4D blob from a frame]

    //! [Set input blob]
    net.setInput(blob);
    //! [Set input blob]
    //! [Make forward pass]
    cv::Mat score = net.forward();
    //! [Make forward pass]

    cv::Mat segm;
    colorizeSegmentation(score, segm);

    cv::resize(segm, segm, frame.size(), 0, 0, INTER_NEAREST);
    addWeighted(frame, 0.1, segm, 0.9, 0.0, frame);

    // Put efficiency information.
    std::vector<double> layersTimes;
    double freq = cv::getTickFrequency() / 1000;
    double t = net.getPerfProfile(layersTimes) / freq;
    std::string label = cv::format("Inference time: %.2f ms", t);
    cv::putText(frame, label, cv::Point(0, 15), FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));

    cv::imshow(kWinName, frame);
    if(!classes.empty())
      showLegend();
  }
  return 0;
}

void
colorizeSegmentation(const cv::Mat& score, cv::Mat& segm) {
  const int rows = score.size[2];
  const int cols = score.size[3];
  const int chns = score.size[1];

  if(colors.empty()) {
    // Generate colors.
    colors.push_back(Vec3b());
    for(int i = 1; i < chns; ++i) {
      Vec3b color;
      for(int j = 0; j < 3; ++j) color[j] = (colors[i - 1][j] + rand() % 256) / 2;
      colors.push_back(color);
    }
  } else if(chns != (int)colors.size()) {
    CV_Error(cv::Error::StsError,
             cv::format("Number of output classes does not match "
                        "number of colors (%d != %zu)",
                        chns,
                        colors.size()));
  }

  cv::Mat maxCl = cv::Mat::zeros(rows, cols, CV_8UC1);
  cv::Mat maxVal(rows, cols, CV_32FC1, score.data);
  for(int ch = 1; ch < chns; ch++) {
    for(int row = 0; row < rows; row++) {
      const float* ptrScore = score.ptr<float>(0, ch, row);
      uint8_t* ptrMaxCl = maxCl.ptr<uint8_t>(row);
      float* ptrMaxVal = maxVal.ptr<float>(row);
      for(int col = 0; col < cols; col++) {
        if(ptrScore[col] > ptrMaxVal[col]) {
          ptrMaxVal[col] = ptrScore[col];
          ptrMaxCl[col] = (uchar)ch;
        }
      }
    }
  }

  segm.create(rows, cols, CV_8UC3);
  for(int row = 0; row < rows; row++) {
    const uchar* ptrMaxCl = maxCl.ptr<uchar>(row);
    Vec3b* ptrSegm = segm.ptr<Vec3b>(row);
    for(int col = 0; col < cols; col++) { ptrSegm[col] = colors[ptrMaxCl[col]]; }
  }
}

void
showLegend() {
  static const int kBlockHeight = 30;
  static cv::Mat legend;
  if(legend.empty()) {
    const int numClasses = (int)classes.size();
    if((int)colors.size() != numClasses) {
      CV_Error(cv::Error::StsError,
               cv::format("Number of output classes does not match "
                          "number of labels (%zu != %zu)",
                          colors.size(),
                          classes.size()));
    }
    legend.create(kBlockHeight * numClasses, 200, CV_8UC3);
    for(int i = 0; i < numClasses; i++) {
      cv::Mat block = legend.rowRange(i * kBlockHeight, (i + 1) * kBlockHeight);
      block.setTo(colors[i]);
      cv::putText(block, classes[i], cv::Point(0, kBlockHeight / 2), FONT_HERSHEY_SIMPLEX, 0.5, Vec3b(255, 255, 255));
    }
    cv::namedWindow("Legend", WINDOW_NORMAL);
    cv::imshow("Legend", legend);
  }
}
