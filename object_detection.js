import { WINDOW_NORMAL, FILLED, FONT_HERSHEY_SIMPLEX, drawRect, resize, minMaxLoc, Scalar, CV_8U, CV_32FC1, Size, CommandLineParser, createTrackbar, dnn, FileNode, FileStorage, getTextSize, getTickFrequency, Mat, namedWindow, VideoCapture, waitKey, imshow, putText, } from 'opencv';
import { readFileSync } from 'fs';

const Error = { StsNotImplemented: 1 };

Map.prototype.getOrInsertComputed = function(key, callbackFunction) {
  if(!this.has(key)) this.set(key, callbackFunction(key));

  return this.get(key);
};

function CV_Error(code, ...rest) {
  console.error(...rest);
  std.exit(code);
}

function CV_Assert(cond) {
  if(!cond) {
    console.error('Assertion failed');
    std.exit(1);
  }
}

function findFile(file) {
  return file;
}

let keys =
  `{ help  h     | | Print help message. }` +
  `{ @alias      | | An alias name of model to extract preprocessing parameters from models.yml file. }` +
  `{ zoo         | models.yml | An optional path to file with preprocessing parameters }` +
  `{ device      |  0 | camera device number. }` +
  `{ input i     | | Path to input image or video file. Skip this argument to capture frames from a camera. }` +
  `{ framework f | | Optional name of an origin framework of the model. Detect it automatically if it does not set. }` +
  `{ classes     | | Optional path to a text file with names of classes to label detected objects. }` +
  `{ thr         | .5 | Confidence threshold. }` +
  `{ nms         | .4 | Non-maximum suppression threshold. }` +
  `{ backend     |  0 | Choose one of computation backends: ` +
  `0: automatically (by default), ` +
  `1: Halide language (http://halide-lang.org/), ` +
  `2: Intel's Deep Learning Inference Engine (https://software.intel.com/openvino-toolkit), ` +
  `3: OpenCV implementation, ` +
  `4: VKCOM, ` +
  `5: CUDA }` +
  `{ target      | 0 | Choose one of target computation devices: ` +
  `0: CPU target (by default), ` +
  `1: OpenCL, ` +
  `2: OpenCL fp16 (half-float precision), ` +
  `3: VPU, ` +
  `4: Vulkan, ` +
  `6: CUDA, ` +
  `7: CUDA fp16 (half-float preprocess) }` +
  `{ async       | 0 | Number of asynchronous forwards at the same time. ` +
  `Choose 0 for synchronous mode }` +
  `{ scale     | 1 | Preprocess input image by multiplying on a scale factor. }` +
  `{ mean      | 0 | Preprocess input image by subtracting mean values. Mean values should be in BGR order and delimited by spaces.", modelName, zooFile) }` +
  `{ rgb       |   | Indicate that model works with RGB input images instead BGR ones. }` +
  `{ width     |   | Preprocess input image by resizing to a specific width. }` +
  `{ height    |   | Preprocess input image by resizing to a specific height. }` +
  `{ model   m |   | Path to a binary file of model contains trained weights.
                     It could be a file with extensions .caffemodel (Caffe), 
                     .pb (TensorFlow), .t7 or .net (Torch), .weights (Darknet), .bin (OpenVINO). }` +
  `{ config  c |  | Path to a text file of model contains network configuration.
                     It could be a file with extensions .prototxt (Caffe), .pbtxt (TensorFlow), .cfg (Darknet), .xml (OpenVINO). }`;
let confThreshold, nmsThreshold;
let classes = [];

/*inline void preprocess(const Mat& frame, Net& net, Size inpSize, float scale, const Scalar& mean, bool swapRB);

void postprocess(Mat& frame, const std::vector<Mat>& out, Net& net, int backend);

void drawPred(int classId, float conf, int left, int top, int right, int bottom, Mat& frame);

void callback(int pos, void* userdata);*/

/*#ifdef USE_THREADS
template<typename T> class QueueFPS : public std::queue<T> {
public:
  QueueFPS()
      : counter(0) {}

  void push(const T& entry) {
    std::lock_guard<std::mutex> lock(mutex);

    std::queue<T>::push(entry);
    counter += 1;
    if(counter == 1) {
      // Start counting from a second frame (warmup).
      tm.reset();
      tm.start();
    }
  }

  T get() {
    std::lock_guard<std::mutex> lock(mutex);
    T entry = this->front();
    this->pop();
    return entry;
  }

  float getFPS() {
    tm.stop();
    double fps = counter / tm.getTimeSec();
    tm.start();
    return static_cast<float>(fps);
  }

  void clear() {
    std::lock_guard<std::mutex> lock(mutex);
    while(!this->empty())
      this->pop();
  }

  unsigned int counter;

private:
  TickMeter tm;
  std::mutex mutex;
};
#endif // USE_THREADS*/

function main() {
  const parser = new CommandLineParser(scriptArgs, keys);

  const modelName = parser.get('@alias');
  const zooFile = parser.get('zoo');

  /* keys += genPreprocArguments(modelName, zooFile);

  parser = new CommandLineParser(argc, argv, keys);*/
  parser.about('Use this script to run object detection deep learning networks using OpenCV.');

  if(scriptArgs.length == 1 || parser.has('help')) {
    parser.printMessage();
    return 0;
  }

  confThreshold = +parser.get('thr');
  nmsThreshold = +parser.get('nms');
  let scale = +parser.get('scale');
  let mean = parser.get('mean');
  let swapRB = Boolean(parser.get('rgb'));
  let inpWidth = +parser.get('width');
  let inpHeight = +parser.get('height');
  let asyncNumReq = +parser.get('async');
  CV_Assert(parser.has('model'));
  let modelPath = findFile(parser.get('model'));
  let configPath = findFile(parser.get('config'));

  // Open file with classes names.
  if(parser.has('classes')) {
    let file = parser.get('classes');

    classes = readFileSync(file, 'utf-8').trimEnd().split('\n');
  }

  // Load a model.
  let net = dnn.readNet(modelPath, configPath, parser.get('framework'));
  let backend = +parser.get('backend');
  net.setPreferableBackend(backend);
  net.setPreferableTarget(+parser.get('target'));
  let outNames = net.getUnconnectedOutLayersNames();

  // Create a window
  const kWinName = 'Deep learning object detection in OpenCV';
  namedWindow(kWinName, WINDOW_NORMAL);
  let initialConf = Math.floor(confThreshold * 100);

  createTrackbar('Confidence threshold, %', kWinName, v => (initialConf = v), 99, callback);

  // Open a video file or an image file or a camera stream.
  const cap = new VideoCapture();

  if(parser.has('input')) cap.open(parser.get('input'));
  else cap.open(+parser.get('device'));

  /*#ifdef USE_THREADS
  let process = true;

  // Frames capturing thread
  QueueFPS<Mat> framesQueue;
  std::thread framesThread([&]() {
    Mat frame;
    while(process) {
      cap >> frame;
      if(!frame.empty())
        framesQueue.push(frame.clone());
      else
        break;
    }
  });

  // Frames processing thread
  QueueFPS<Mat> processedFramesQueue;
  QueueFPS<std::vector<Mat>> predictionsQueue;
  std::thread processingThread([&]() {
    std::queue<AsyncArray> futureOutputs;
    Mat blob;
    while(process) {
      // Get a next frame
      Mat frame;
      {
        if(!framesQueue.empty()) {
          frame = framesQueue.get();
          if(asyncNumReq) {
            if(futureOutputs.length == asyncNumReq)
              frame = Mat();
          } else
            framesQueue.clear(); // Skip the rest of frames
        }
      }

      // Process the frame
      if(!frame.empty()) {
        preprocess(frame, net, Size(inpWidth, inpHeight), scale, mean, swapRB);
        processedFramesQueue.push(frame);

        if(asyncNumReq) {
          futureOutputs.push(net.forwardAsync());
        } else {
          std::vector<Mat> outs;
          net.forward(outs, outNames);
          predictionsQueue.push(outs);
        }
      }

      while(!futureOutputs.empty() && futureOutputs.front().wait_for(std::chrono::seconds(0))) {
        AsyncArray async_out = futureOutputs.front();
        futureOutputs.pop();
        Mat out;
        async_out.get(out);
        predictionsQueue.push({out});
      }
    }
  });

  // Postprocessing and rendering loop
  while(waitKey(1) < 0) {
    if(predictionsQueue.empty())
      continue;

    std::vector<Mat> outs = predictionsQueue.get();
    Mat frame = processedFramesQueue.get();

    postprocess(frame, outs, net, backend);

    if(predictionsQueue.counter > 1) {
      std::string label = format("Camera: %.2f FPS", framesQueue.getFPS());
      putText(frame, label, Point(0, 15), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0));

      label = format("Network: %.2f FPS", predictionsQueue.getFPS());
      putText(frame, label, Point(0, 30), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0));

      label = format("Skipped frames: %d", framesQueue.counter - predictionsQueue.counter);
      putText(frame, label, Point(0, 45), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0));
    }
    imshow(kWinName, frame);
  }

  process = false;
  framesThread.join();
  processingThread.join();

#else  // USE_THREADS*/
  if(asyncNumReq) CV_Error(Error.StsNotImplemented, 'Asynchronous forward is supported only with Inference Engine backend.');

  // Process frames.
  let frame = new Mat(),
    blob = new Mat();

  while(waitKey(1) < 0) {
    cap.read(frame);
    if(frame.empty) {
      waitKey();
      break;
    }

    preprocess(frame, net, new Size(inpWidth, inpHeight), scale, mean, swapRB);

    let outs = [];
    net.forward(outs, outNames);

    postprocess(frame, outs, net, backend);

    // Put efficiency information.
    let layersTimes = [];
    let freq = getTickFrequency() / 1000;
    let t = net.getPerfProfile(layersTimes) / freq;
    let label = `Inference time: ${t} ms`;
    putText(frame, label, new Point(0, 15), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0));

    imshow(kWinName, frame);
  }
  /*#endif // USE_THREADS*/
  return 0;
}

let blob = new Mat();

function preprocess(frame, net, inpSize, scale, mean, swapRB) {
  // Create a 4D blob from a frame.
  if(inpSize.width <= 0) inpSize.width = frame.cols;
  if(inpSize.height <= 0) inpSize.height = frame.rows;
  dnn.blobFromImage(frame, blob, 1.0, inpSize, Scalar(), swapRB, false, CV_8U);

  // Run a model.
  net.setInput(blob, '', scale, mean);

  if(net.getLayer(0).outputNameToIndex('im_info') != -1) {
    // Faster-RCNN or R-FCN
    resize(frame, frame, inpSize);
    let imInfo = new Mat(1, 3, CV_32FC1, new Float32Array([inpSize.height, inpSize.width, 1.6]));

    net.setInput(imInfo, 'im_info');
  }
}

function postprocess(/*Mat&*/ frame, outs, /*Net& */ net, backend) {
  let outLayers = net.getUnconnectedOutLayers();
  let outLayerType = net.getLayer(outLayers[0]).type;

  let classIds = [];
  let confidences = [];
  let boxes = [];

  if(outLayerType == 'DetectionOutput') {
    // Network produces output blob with a shape 1x1xNx7 where N is a number of
    // detections and an every detection is a vector of values
    // [batchId, classId, confidence, left, top, right, bottom]
    CV_Assert(outs.length > 0);
    for(let k = 0; k < outs.length; k++) {
      let data = outs[k].array;

      for(let i = 0; i < outs[k].total(); i += 7) {
        let confidence = data[i + 2];
        if(confidence > confThreshold) {
          let left = Math.floor(data[i + 3]);
          let top = Math.floor(data[i + 4]);
          let right = Math.floor(data[i + 5]);
          let bottom = Math.floor(data[i + 6]);
          let width = right - left + 1;
          let height = bottom - top + 1;
          if(width <= 2 || height <= 2) {
            left = Math.floor(data[i + 3] * frame.cols);
            top = Math.floor(data[i + 4] * frame.rows);
            right = Math.floor(data[i + 5] * frame.cols);
            bottom = Math.floor(data[i + 6] * frame.rows);
            width = right - left + 1;
            height = bottom - top + 1;
          }
          classIds.push(Math.floor(data[i + 1]) - 1); // Skip 0th background class id.
          boxes.push(new Rect(left, top, width, height));
          confidences.push(confidence);
        }
      }
    }
  } else if(outLayerType == 'Region') {
    for(let i = 0; i < outs.length; ++i) {
      // Network produces output blob with a shape NxC where N is a number of
      // detected objects and C is a number of classes + 4 where the first 4
      // numbers are [center_x, center_y, width, height]
      let data = outs[i].array;
      for(let j = 0; j < outs[i].rows; ++j, data += outs[i].cols) {
        let scores = outs[i].row(j).colRange(5, outs[i].cols);
        let classIdPoint;
        let confidence;
        minMaxLoc(
          scores,
          0,
          v => (confidence = v),
          0,
          v => (classIdPoint = v),
        );
        if(confidence > confThreshold) {
          let centerX = Math.floor(data[0] * frame.cols);
          let centerY = Math.floor(data[1] * frame.rows);
          let width = Math.floor(data[2] * frame.cols);
          let height = Math.floor(data[3] * frame.rows);
          let left = centerX - width / 2;
          let top = centerY - height / 2;

          classIds.push(classIdPoint.x);
          confidences.push(Math.floor(confidence));
          boxes.push(new Rect(left, top, width, height));
        }
      }
    }
  } else CV_Error(Error.StsNotImplemented, 'Unknown output layer type: ' + outLayerType);

  // NMS is used inside Region layer only on DNN_BACKEND_OPENCV for another backends we need NMS in sample
  // or NMS is required if number of outputs > 1
  if(outLayers.length > 1 || (outLayerType == 'Region' && backend != dnn.DNN_BACKEND_OPENCV)) {
    /*std::map<int, std::vector<size_t>>*/ let class2indices = new Map();

    for(let i = 0; i < classIds.length; i++) if(confidences[i] >= confThreshold) class2indices.getOrInsertComputed(classIds[i], k => []).push(i);
    let nmsBoxes = [];
    let nmsConfidences = [];
    let nmsClassIds = [];

    for(let [first, second] of class2indices) {
      let localBoxes;
      let localConfidences;
      let classIndices = second;

      for(let i = 0; i < classIndices.length; i++) {
        localBoxes.push(boxes[classIndices[i]]);
        localConfidences.push(confidences[classIndices[i]]);
      }
      let nmsIndices = [];
      dnn.NMSBoxes(localBoxes, localConfidences, confThreshold, nmsThreshold, nmsIndices);
      for(let i = 0; i < nmsIndices.length; i++) {
        let idx = nmsIndices[i];
        nmsBoxes.push(localBoxes[idx]);
        nmsConfidences.push(localConfidences[idx]);
        nmsClassIds.push(first);
      }
    }
    boxes = nmsBoxes;
    classIds = nmsClassIds;
    confidences = nmsConfidences;
  }

  for(let idx = 0; idx < boxes.length; ++idx) {
    let box = boxes[idx];
    drawPred(classIds[idx], confidences[idx], box.x, box.y, box.x + box.width, box.y + box.height, frame);
  }
}

function drawPred(classId, conf, left, top, right, bottom, /*Mat&*/ frame) {
  drawRect(frame, new Point(left, top), new Point(right, bottom), Scalar(0, 255, 0));

  let label = Math.round(conf * 100) / 100;

  if(!classes.length == 0) {
    CV_Assert(classId < classes.length);
    label = classes[classId] + ': ' + label;
  }

  let baseLine;
  let labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, v => (baseLine = v));

  top = Math.max(top, labelSize.height);
  drawRect(frame, new Point(left, top - labelSize.height), new Point(left + labelSize.width, top + baseLine), Scalar(255, 255, 255, 255), FILLED);
  putText(frame, label, new Point(left, top), FONT_HERSHEY_SIMPLEX, 0.5, Scalar());
}

function callback(pos) {
  confThreshold = pos * 0.01;
}

main();
