#include <fstream>
#include <sstream>

#include <opencv2/cv::dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#ifdef CV_CXX11
#include <mutex>
#include <thread>
#include <queue>
#endif

//#include "common.hpp"

std::string keys = "{ help  h     | | Print help message. }"
                   "{ @alias      | | An alias name of model to extract preprocessing parameters from models.yml "
                   "file. }"
                   "{ zoo         | models.yml | An optional path to file with preprocessing parameters }"
                   "{ device      |  0 | camera device number. }"
                   "{ input i     | | Path to input image or video file. Skip this argument to capture frames "
                   "from a camera. }"
                   "{ framework f | | Optional name of an origin framework of the model. Detect it automatically "
                   "if it does not set. }"
                   "{ classes     | | Optional path to a text file with names of classes to label detected "
                   "objects. }"
                   "{ thr         | .5 | Confidence cv::threshold. }"
                   "{ nms         | .4 | Non-maximum suppression cv::threshold. }"
                   "{ backend     |  0 | Choose one of computation backends: "
                   "0: automatically (by default), "
                   "1: Halide language (http://halide-lang.org/), "
                   "2: Intel's Deep Learning Inference Engine (https://software.intel.com/openvino-toolkit), "
                   "3: OpenCV implementation }"
                   "{ target      | 0 | Choose one of target computation devices: "
                   "0: CPU target (by default), "
                   "1: OpenCL, "
                   "2: OpenCL fp16 (half-float precision), "
                   "3: VPU }"
                   "{ async       | 0 | Number of asynchronous forwards at the same time. "
                   "Choose 0 for synchronous mode }";

//using namespace cv;
using namespace cv::dnn;

float confThreshold, nmsThreshold;
std::vector<std::string> classes;

inline void preprocess(const cv::Mat& frame, Net& net, cv::Size inpSize, float scale, const cv::Scalar& cv::mean, bool swapRB);

void postprocess(cv::Mat& frame, const std::vector<Mat>& out, Net& net);

void drawPred(int classId, float conf, int left, int top, int right, int bottom, cv::Mat& frame);

void callback(int pos, void* userdata);

#ifdef CV_CXX11
template<typename T> class QueueFPS : public std::queue<T> {
public:
  QueueFPS() : counter(0) {}

  void
  push(const T& entry) {
    std::lock_guard<std::mutex> lock(mutex);

    std::queue<T>::push(entry);
    counter += 1;
    if(counter == 1) {
      // Start counting from a second frame (warmup).
      tm.reset();
      tm.start();
    }
  }

  T
  get() {
    std::lock_guard<std::mutex> lock(mutex);
    T entry = this->front();
    this->pop();
    return entry;
  }

  float
  getFPS() {
    tm.stop();
    double fps = counter / tm.getTimeSec();
    tm.start();
    return static_cast<float>(fps);
  }

  void
  clear() {
    std::lock_guard<std::mutex> lock(mutex);
    while(!this->empty()) this->pop();
  }

  unsigned int counter;

private:
  TickMeter tm;
  std::mutex mutex;
};
#endif // CV_CXX11

int
main(int argc, char** argv) {
  cv::CommandLineParser parser(argc, argv, keys);

  const std::string modelName = parser.get<cv::String>("@alias");
  const std::string zooFile = parser.get<cv::String>("zoo");

  keys += genPreprocArguments(modelName, zooFile);

  parser = cv::CommandLineParser(argc, argv, keys);
  parser.about("Use this script to run object detection deep learning networks using OpenCV.");
  if(argc == 1 || parser.has("help")) {
    parser.printMessage();
    return 0;
  }

  confThreshold = parser.get<float>("thr");
  nmsThreshold = parser.get<float>("nms");
  float scale = parser.get<float>("scale");
  cv::Scalar cv::mean = parser.get<Scalar>("mean");
  bool swapRB = parser.get<bool>("rgb");
  int inpWidth = parser.get<int>("width");
  int inpHeight = parser.get<int>("height");
  size_t async = parser.get<int>("async");
  CV_Assert(parser.has("model"));
  std::string modelPath = findFile(parser.get<cv::String>("model"));
  std::string configPath = findFile(parser.get<cv::String>("config"));

  // Open file with classes names.
  if(parser.has("classes")) {
    std::string file = parser.get<cv::String>("classes");
    std::ifstream ifs(file.c_str());
    if(!ifs.is_open())
      CV_Error(cv::Error::StsError, "File " + file + " not found");
    std::string cv::line;
    while(std::getline(ifs, cv::line)) { classes.push_back(line); }
  }

  // Load a model.
  Net net = readNet(modelPath, configPath, parser.get<cv::String>("framework"));
  net.setPreferableBackend(parser.get<int>("backend"));
  net.setPreferableTarget(parser.get<int>("target"));
  std::vector<cv::String> outNames = net.getUnconnectedOutLayersNames();

  // Create a window
  static const std::string kWinName = "Deep learning object detection in OpenCV";
  cv::namedWindow(kWinName, WINDOW_NORMAL);
  int initialConf = (int)(confThreshold * 100);
  cv::createTrackbar("Confidence cv::threshold, %", kWinName, &initialConf, 99, callback);

  // Open a video file or an image file or a camera stream.
  cv::VideoCapture cap;
  if(parser.has("input"))
    cap.open(parser.get<cv::String>("input"));
  else
    cap.open(parser.get<int>("device"));

#ifdef CV_CXX11
  bool process = true;

  // Frames capturing thread
  QueueFPS<cv::Mat> framesQueue;
  std::thread framesThread([&]() {
    cv::Mat frame;
    while(process) {
      cap >> frame;
      if(!frame.empty())
        framesQueue.push(frame.clone());
      else
        break;
    }
  });

  // Frames processing thread
  QueueFPS<cv::Mat> processedFramesQueue;
  QueueFPS<std::vector<cv::Mat>> predictionsQueue;
  std::thread processingThread([&]() {
    std::queue<cv::AsyncArray> futureOutputs;
    cv::Mat blob;
    while(process) {
      // Get a next frame
      cv::Mat frame;
      {
        if(!framesQueue.empty()) {
          frame = framesQueue.get();
          if(async) {
            if(futureOutputs.size() == async)
              frame = cv::Mat();
          } else
            framesQueue.clear(); // Skip the rest of frames
        }
      }

      // Process the frame
      if(!frame.empty()) {
        preprocess(frame, net, cv::Size(inpWidth, inpHeight), scale, cv::mean, swapRB);
        processedFramesQueue.push(frame);

        if(async) {
          futureOutputs.push(net.forwardAsync());
        } else {
          std::vector<cv::Mat> outs;
          net.forward(outs, outNames);
          predictionsQueue.push(outs);
        }
      }

      while(!futureOutputs.empty() && futureOutputs.front().wait_for(std::chrono::seconds(0))) {
        cv::AsyncArray async_out = futureOutputs.front();
        futureOutputs.pop();
        cv::Mat out;
        async_out.get(out);
        predictionsQueue.push({out});
      }
    }
  });

  // Postprocessing and rendering loop
  while(cv::waitKey(1) < 0) {
    if(predictionsQueue.empty())
      continue;

    std::vector<cv::Mat> outs = predictionsQueue.get();
    cv::Mat frame = processedFramesQueue.get();

    postprocess(frame, outs, net);

    if(predictionsQueue.counter > 1) {
      std::string label = cv::format("Camera: %.2f FPS", framesQueue.getFPS());
      cv::putText(frame, label, cv::Point(0, 15), FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));

      label = cv::format("Network: %.2f FPS", predictionsQueue.getFPS());
      cv::putText(frame, label, cv::Point(0, 30), FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));

      label = cv::format("Skipped frames: %d", framesQueue.counter - predictionsQueue.counter);
      cv::putText(frame, label, cv::Point(0, 45), FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));
    }
    cv::imshow(kWinName, frame);
  }

  process = false;
  framesThread.join();
  processingThread.join();

#else  // CV_CXX11
  if(async)
    CV_Error(cv::Error::StsNotImplemented, "Asynchronous forward is supported only with Inference Engine backend.");

  // Process frames.
  cv::Mat frame, blob;
  while(cv::waitKey(1) < 0) {
    cap >> frame;
    if(frame.empty()) {
      cv::waitKey();
      break;
    }

    preprocess(frame, net, cv::Size(inpWidth, inpHeight), scale, cv::mean, swapRB);

    std::vector<cv::Mat> outs;
    net.forward(outs, outNames);

    postprocess(frame, outs, net);

    // Put efficiency information.
    std::vector<double> layersTimes;
    double freq = cv::getTickFrequency() / 1000;
    double t = net.getPerfProfile(layersTimes) / freq;
    std::string label = cv::format("Inference time: %.2f ms", t);
    cv::putText(frame, label, cv::Point(0, 15), FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));

    cv::imshow(kWinName, frame);
  }
#endif // CV_CXX11
  return 0;
}

inline void
preprocess(const cv::Mat& frame, Net& net, cv::Size inpSize, float scale, const cv::Scalar& cv::mean, bool swapRB) {
  static cv::Mat blob;
  // Create a 4D blob from a frame.
  if(inpSize.width <= 0)
    inpSize.width = frame.cols;
  if(inpSize.height <= 0)
    inpSize.height = frame.rows;
  blobFromImage(frame, blob, 1.0, inpSize, cv::Scalar(), swapRB, false, CV_8U);

  // Run a model.
  net.setInput(blob, "", scale, cv::mean);
  if(net.getLayer(0)->outputNameToIndex("im_info") != -1) { // Faster-RCNN or R-FCN
    cv::resize(frame, frame, inpSize);
    cv::Mat imInfo = (Mat_<float>(1, 3) << inpSize.height, inpSize.width, 1.6f);
    net.setInput(imInfo, "im_info");
  }
}

void
postprocess(cv::Mat& frame, const std::vector<Mat>& outs, Net& net) {
  static std::vector<int> outLayers = net.getUnconnectedOutLayers();
  static std::string outLayerType = net.getLayer(outLayers[0])->type;

  std::vector<int> classIds;
  std::vector<float> confidences;
  std::vector<cv::Rect> boxes;
  if(outLayerType == "DetectionOutput") {
    // Network produces output blob with a shape 1x1xNx7 where N is a number of
    // detections and an every detection is a vector of values
    // [batchId, classId, confidence, left, top, right, bottom]
    CV_Assert(outs.size() > 0);
    for(size_t k = 0; k < outs.size(); k++) {
      float* data = (float*)outs[k].data;
      for(size_t i = 0; i < outs[k].total(); i += 7) {
        float confidence = data[i + 2];
        if(confidence > confThreshold) {
          int left = (int)data[i + 3];
          int top = (int)data[i + 4];
          int right = (int)data[i + 5];
          int bottom = (int)data[i + 6];
          int width = right - left + 1;
          int height = bottom - top + 1;
          if(width * height <= 1) {
            left = (int)(data[i + 3] * frame.cols);
            top = (int)(data[i + 4] * frame.rows);
            right = (int)(data[i + 5] * frame.cols);
            bottom = (int)(data[i + 6] * frame.rows);
            width = right - left + 1;
            height = bottom - top + 1;
          }
          classIds.push_back((int)(data[i + 1]) - 1); // Skip 0th background class id.
          boxes.push_back(cv::Rect(left, top, width, height));
          confidences.push_back(confidence);
        }
      }
    }
  } else if(outLayerType == "Region") {
    for(size_t i = 0; i < outs.size(); ++i) {
      // Network produces output blob with a shape NxC where N is a number of
      // detected objects and C is a number of classes + 4 where the first 4
      // numbers are [center_x, center_y, width, height]
      float* data = (float*)outs[i].data;
      for(int j = 0; j < outs[i].rows; ++j, data += outs[i].cols) {
        cv::Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
        cv::Point classIdPoint;
        double confidence;
        cv::minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
        if(confidence > confThreshold) {
          int centerX = (int)(data[0] * frame.cols);
          int centerY = (int)(data[1] * frame.rows);
          int width = (int)(data[2] * frame.cols);
          int height = (int)(data[3] * frame.rows);
          int left = centerX - width / 2;
          int top = centerY - height / 2;

          classIds.push_back(classIdPoint.x);
          confidences.push_back((float)confidence);
          boxes.push_back(cv::Rect(left, top, width, height));
        }
      }
    }
  } else
    CV_Error(cv::Error::StsNotImplemented, "Unknown output layer type: " + outLayerType);

  std::vector<int> indices;
  NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);
  for(size_t i = 0; i < indices.size(); ++i) {
    int idx = indices[i];
    cv::Rect box = boxes[idx];
    drawPred(classIds[idx], confidences[idx], box.x, box.y, box.x + box.width, box.y + box.height, frame);
  }
}

void
drawPred(int classId, float conf, int left, int top, int right, int bottom, cv::Mat& frame) {
  cv::rectangle(frame, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(0, 255, 0));

  std::string label = cv::format("%.2f", conf);
  if(!classes.empty()) {
    CV_Assert(classId < (int)classes.size());
    label = classes[classId] + ": " + label;
  }

  int baseLine;
  cv::Size labelSize = cv::getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

  top = max(top, labelSize.height);
  cv::rectangle(
      frame, cv::Point(left, top - labelSize.height), cv::Point(left + labelSize.width, top + baseLine), cv::Scalar::all(255), FILLED);
  cv::putText(frame, label, cv::Point(left, top), FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar());
}

void
callback(int pos, void*) {
  confThreshold = pos * 0.01f;
}
