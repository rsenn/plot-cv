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
                   "{ input i     | | Path to input image or video file. Skip this argument to capture frames "
                   "from a camera.}"
                   "{ framework f | | Optional name of an origin framework of the model. Detect it automatically "
                   "if it does not set. }"
                   "{ classes     | | Optional path to a text file with names of classes. }"
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

int
main(int argc, char** argv) {
  cv::CommandLineParser parser(argc, argv, keys);

  const std::string modelName = parser.get<cv::String>("@alias");
  const std::string zooFile = parser.get<cv::String>("zoo");

  keys += genPreprocArguments(modelName, zooFile);

  parser = cv::CommandLineParser(argc, argv, keys);
  parser.about("Use this script to run classification deep learning networks using OpenCV.");
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
  static const std::string kWinName = "Deep learning image classification in OpenCV";
  cv::namedWindow(kWinName, WINDOW_NORMAL);

  //! [Open a video file or an image file or a camera stream]
  cv::VideoCapture cap;
  if(parser.has("input"))
    cap.open(parser.get<cv::String>("input"));
  else
    cap.open(0);
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
    cv::Mat prob = net.forward();
    //! [Make forward pass]

    //! [Get a class with a highest score]
    cv::Point classIdPoint;
    double confidence;
    cv::minMaxLoc(prob.reshape(1, 1), 0, &confidence, 0, &classIdPoint);
    int classId = classIdPoint.x;
    //! [Get a class with a highest score]

    // Put efficiency information.
    std::vector<double> layersTimes;
    double freq = cv::getTickFrequency() / 1000;
    double t = net.getPerfProfile(layersTimes) / freq;
    std::string label = cv::format("Inference time: %.2f ms", t);
    cv::putText(frame, label, cv::Point(0, 15), FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));

    // Print predicted class.
    label = cv::format("%s: %.4f",
                       (classes.empty() ? cv::format("Class #%d", classId).c_str() : classes[classId].c_str()),
                       confidence);
    cv::putText(frame, label, cv::Point(0, 40), FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));

    cv::imshow(kWinName, frame);
  }
  return 0;
}
