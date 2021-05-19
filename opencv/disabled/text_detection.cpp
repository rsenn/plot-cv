/*
    Text detection model: https://github.com/argman/EAST
    Download link: https://www.dropbox.com/s/r2ingd0l3zt8hxs/frozen_east_text_detection.tar.gz?dl=1

    CRNN Text recognition model taken from here: https://github.com/meijieru/crnn.pytorch
    How to convert from pb to onnx:
    Using classes from here: https://github.com/meijieru/crnn.pytorch/blob/master/models/crnn.py

    More converted onnx text recognition models can be downloaded directly here:
    Download link: https://drive.google.com/drive/folders/1cTbQ3nuZG-EKWak6emD_s8_hHXWz7lAr?usp=sharing
    And these models taken from here:https://github.com/clovaai/deep-text-recognition-benchmark

    import torch
    from models.crnn import CRNN

    model = CRNN(32, 1, 37, 256)
    model.load_state_dict(torch.load('crnn.pth'))
    dummy_input = torch.randn(1, 1, 32, 100)
    torch.onnx.export(model, dummy_input, "crnn.onnx", verbose=True)
*/

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/dnn.hpp>

// using namespace cv;
// using namespace cv::dnn;

const char* keys =
    "{ help  h     | | Print help message. }"
    "{ input i     | | Path to input image or video file. Skip this argument to capture frames from a camera.}"
    "{ model m     | | Path to a binary .pb file contains trained detector network.}"
    "{ ocr         | | Path to a binary .pb or .onnx file contains trained recognition network.}"
    "{ width       | 320 | Preprocess input image by resizing to a specific width. It should be multiple by 32. }"
    "{ height      | 320 | Preprocess input image by resizing to a specific height. It should be multiple by 32. }"
    "{ thr         | 0.5 | Confidence cv::threshold. }"
    "{ nms         | 0.4 | Non-maximum suppression cv::threshold. }";

void decodeBoundingBoxes(const cv::Mat& scores,
                         const cv::Mat& geometry,
                         float scoreThresh,
                         std::vector<cv::RotatedRect>& detections,
                         std::vector<float>& confidences);

void fourPointsTransform(const cv::Mat& frame, cv::Point2f vertices[4], cv::Mat& result);

void decodeText(const cv::Mat& scores, std::string& text);

int
main(int argc, char** argv) {
  // Parse command cv::line arguments.
  cv::CommandLineParser parser(argc, argv, keys);
  parser.about("Use this script to run TensorFlow implementation (https://github.com/argman/EAST) of "
               "EAST: An Efficient and Accurate Scene Text Detector (https://arxiv.org/abs/1704.03155v2)");
  if(argc == 1 || parser.has("help")) {
    parser.printMessage();
    return 0;
  }

  float confThreshold = parser.get<float>("thr");
  float nmsThreshold = parser.get<float>("nms");
  int inpWidth = parser.get<int>("width");
  int inpHeight = parser.get<int>("height");
  cv::String modelDecoder = parser.get<String>("model");
  cv::String modelRecognition = parser.get<String>("ocr");

  if(!parser.check()) {
    parser.printErrors();
    return 1;
  }

  CV_Assert(!modelDecoder.empty());

  // Load networks.
  Net detector = readNet(modelDecoder);
  Net recognizer;

  if(!modelRecognition.empty())
    recognizer = readNet(modelRecognition);

  // Open a video file or an image file or a camera stream.
  cv::VideoCapture cap;
  bool openSuccess = parser.has("input") ? cap.open(parser.get<cv::String>("input")) : cap.open(0);
  CV_Assert(openSuccess);

  static const std::string kWinName = "EAST: An Efficient and Accurate Scene Text Detector";
  cv::namedWindow(kWinName, WINDOW_NORMAL);

  std::vector<cv::Mat> outs;
  std::vector<cv::String> outNames(2);
  outNames[0] = "feature_fusion/Conv_7/Sigmoid";
  outNames[1] = "feature_fusion/concat_3";

  cv::Mat frame, blob;
  TickMeter tickMeter;
  while(cv::waitKey(1) < 0) {
    cap >> frame;
    if(frame.empty()) {
      cv::waitKey();
      break;
    }

    blobFromImage(frame, blob, 1.0, cv::Size(inpWidth, inpHeight), cv::Scalar(123.68, 116.78, 103.94), true, false);
    detector.setInput(blob);
    tickMeter.start();
    detector.forward(outs, outNames);
    tickMeter.stop();

    cv::Mat scores = outs[0];
    cv::Mat geometry = outs[1];

    // Decode predicted bounding boxes.
    std::vector<cv::RotatedRect> boxes;
    std::vector<float> confidences;
    decodeBoundingBoxes(scores, geometry, confThreshold, boxes, confidences);

    // Apply non-maximum suppression procedure.
    std::vector<int> indices;
    NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);

    cv::Point2f ratio((float)frame.cols / inpWidth, (float)frame.rows / inpHeight);

    // Render text.
    for(size_t i = 0; i < indices.size(); ++i) {
      cv::RotatedRect& box = boxes[indices[i]];

      cv::Point2f vertices[4];
      box.points(vertices);

      for(int j = 0; j < 4; ++j) {
        vertices[j].x *= ratio.x;
        vertices[j].y *= ratio.y;
      }

      if(!modelRecognition.empty()) {
        cv::Mat cropped;
        fourPointsTransform(frame, vertices, cropped);

        cv::cvtColor(cropped, cropped, cv::COLOR_BGR2GRAY);

        cv::Mat blobCrop = blobFromImage(cropped, 1.0 / 127.5, cv::Size(), cv::Scalar::all(127.5));
        recognizer.setInput(blobCrop);

        tickMeter.start();
        cv::Mat result = recognizer.forward();
        tickMeter.stop();

        std::string wordRecognized = "";
        decodeText(result, wordRecognized);
        cv::putText(frame, wordRecognized, vertices[1], FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(0, 0, 255));
      }

      for(int j = 0; j < 4; ++j) cv::line(frame, vertices[j], vertices[(j + 1) % 4], cv::Scalar(0, 255, 0), 1);
    }

    // Put efficiency information.
    std::string label = cv::format("Inference time: %.2f ms", tickMeter.getTimeMilli());
    cv::putText(frame, label, cv::Point(0, 15), FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));

    cv::imshow(kWinName, frame);

    tickMeter.reset();
  }
  return 0;
}

void
decodeBoundingBoxes(const cv::Mat& scores,
                    const cv::Mat& geometry,
                    float scoreThresh,
                    std::vector<cv::RotatedRect>& detections,
                    std::vector<float>& confidences) {
  detections.clear();
  CV_Assert(scores.dims == 4);
  CV_Assert(geometry.dims == 4);
  CV_Assert(scores.size[0] == 1);
  CV_Assert(geometry.size[0] == 1);
  CV_Assert(scores.size[1] == 1);
  CV_Assert(geometry.size[1] == 5);
  CV_Assert(scores.size[2] == geometry.size[2]);
  CV_Assert(scores.size[3] == geometry.size[3]);

  const int height = scores.size[2];
  const int width = scores.size[3];
  for(int y = 0; y < height; ++y) {
    const float* scoresData = scores.ptr<float>(0, 0, y);
    const float* x0_data = geometry.ptr<float>(0, 0, y);
    const float* x1_data = geometry.ptr<float>(0, 1, y);
    const float* x2_data = geometry.ptr<float>(0, 2, y);
    const float* x3_data = geometry.ptr<float>(0, 3, y);
    const float* anglesData = geometry.ptr<float>(0, 4, y);
    for(int x = 0; x < width; ++x) {
      float score = scoresData[x];
      if(score < scoreThresh)
        continue;

      // Decode a prediction.
      // Multiple by 4 because feature maps are 4 time less than input image.
      float offsetX = x * 4.0f, offsetY = y * 4.0f;
      float angle = anglesData[x];
      float cosA = std::cos(angle);
      float sinA = std::sin(angle);
      float h = x0_data[x] + x2_data[x];
      float w = x1_data[x] + x3_data[x];

      cv::Point2f offset(offsetX + cosA * x1_data[x] + sinA * x2_data[x], offsetY - sinA * x1_data[x] + cosA * x2_data[x]);
      cv::Point2f p1 = cv::Point2f(-sinA * h, -cosA * h) + offset;
      cv::Point2f p3 = cv::Point2f(-cosA * w, sinA * w) + offset;
      cv::RotatedRect r(0.5f * (p1 + p3), Size2f(w, h), -angle * 180.0f / (float)CV_PI);
      detections.push_back(r);
      confidences.push_back(score);
    }
  }
}

void
fourPointsTransform(const cv::Mat& frame, cv::Point2f vertices[4], cv::Mat& result) {
  const cv::Size outputSize = cv::Size(100, 32);

  cv::Point2f targetVertices[4] = {
      cv::Point(0, outputSize.height - 1),
      cv::Point(0, 0),
      cv::Point(outputSize.width - 1, 0),
      cv::Point(outputSize.width - 1, outputSize.height - 1),
  };
  cv::Mat rotationMatrix = cv::getPerspectiveTransform(vertices, targetVertices);

  cv::warpPerspective(frame, result, rotationMatrix, outputSize);
}

void
decodeText(const cv::Mat& scores, std::string& text) {
  static const std::string alphabet = "0123456789abcdefghijklmnopqrstuvwxyz";
  cv::Mat scoresMat = scores.reshape(1, scores.size[0]);

  std::vector<char> elements;
  elements.reserve(scores.size[0]);

  for(int rowIndex = 0; rowIndex < scoresMat.rows; ++rowIndex) {
    cv::Point p;
    cv::minMaxLoc(scoresMat.row(rowIndex), 0, 0, 0, &p);
    if(p.x > 0 && static_cast<size_t>(p.x) <= alphabet.size()) {
      elements.push_back(alphabet[p.x - 1]);
    } else {
      elements.push_back('-');
    }
  }

  if(elements.size() > 0 && elements[0] != '-')
    text += elements[0];

  for(size_t elementIndex = 1; elementIndex < elements.size(); ++elementIndex) {
    if(elementIndex > 0 && elements[elementIndex] != '-' && elements[elementIndex - 1] != elements[elementIndex]) {
      text += elements[elementIndex];
    }
  }
}