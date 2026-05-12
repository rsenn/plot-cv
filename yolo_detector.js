/**
 * @file yolo_detector.cpp
 * @brief Yolo Object Detection Sample
 * @author OpenCV team
 */
import { drawRect, CV_32F, CommandLineParser, transposeND, dnn, FileNode, FileStorage, getTextSize, imread, Mat, namedWindow, Scalar, Size, VideoCapture, waitKey, WINDOW_NORMAL, imshow, Point, minMaxLoc, Rect, cvFloor, FONT_HERSHEY_SIMPLEX, FILLED, putText, } from 'opencv';
import { readFileSync } from 'fs';

let classes = [];

function findFile(file) {
  return file;
}

function CV_Error(code, ...rest) {
  console.error(...rest);
  std.exit(code);
}

function CV_CheckEQ(a, b, msg) {
  if(a != b) {
    console.error(msg);
    std.exit(1);
  }
}

const keys =
  `{ help  h     |   | Print help message. }` +
  `{ device      | 0 | camera device number. }` +
  `{ model       | onnx/models/yolox_s_inf_decoder.onnx | Default model. }` +
  `{ yolo        | yolox | yolo model version. }` +
  `{ input i     | | Path to input image or video file. Skip this argument to capture frames from a camera. }` +
  `{ classes     | | Optional path to a text file with names of classes to label detected objects. }` +
  `{ nc          | 80 | Number of classes. Default is 80 (coming from COCO dataset). }` +
  `{ thr         | .5 | Confidence threshold. }` +
  `{ nms         | .4 | Non-maximum suppression threshold. }` +
  `{ mean        | 0.0 0.0 0.0 | Normalization constant. }` +
  `{ scale       | 1.0 1.0 1.0 | Preprocess input image by multiplying on a scale factor. }` +
  `{ width       | 640 | Preprocess input image by resizing to a specific width. }` +
  `{ height      | 640 | Preprocess input image by resizing to a specific height. }` +
  `{ rgb         | 1 | Indicate that model works with RGB input images instead BGR ones. }` +
  `{ padvalue    | 114.0 | padding value. }` +
  `{ paddingmode | 2 | Choose one of computation backends: ` +
  `0: resize to required input size without extra processing, ` +
  `1: Image will be cropped after resize, ` +
  `2: Resize image to the desired size while preserving the aspect ratio of original image }` +
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
  `Choose 0 for synchronous mode }`;

function getClasses(classesFile) {
  try {
    let t = readFileSync(classesFile, 'utf-8');
  } catch(e) {
    CV_Error(Error.StsError, 'File ' + classesFile + ' not found');
  }

  classes = t.trimEnd().split('\n');
}

function drawPrediction(classId, conf, left, top, right, bottom, frame) {
  drawRect(frame, new Point(left, top), new Point(right, bottom), Scalar(0, 255, 0));

  let label = conf + ''; //format('%.2f', conf);

  if(!classes.empty) {
    //CV_Assert(classId < classes.length);
    label = classes[classId] + ': ' + label;
  }

  let baseLine;
  let labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, v => (baseLine = v));

  top = Math.max(top, labelSize.height);

  drawRect(frame, new Point(left, top - labelSize.height), new Point(left + labelSize.width, top + baseLine), Scalar(255, 255, 255, 255), FILLED);

  console.log('putting label: ' + label);
  putText(frame, label, new Point(left, top), FONT_HERSHEY_SIMPLEX, 0.5, Scalar());
}

function yoloPostProcessing(
  outs,
  keep_classIds,
  /*std.vector<float>&*/ keep_confidences,
  /*std.vector<Rect>&*/ keep_boxes,
  /*float*/ conf_threshold,
  /*float*/ iou_threshold,
  /* const std.string& */ model_name,
  /*const int*/ nc = 80,
) {
  const classIds = [];
  const confidences = [];
  const boxes = [];
  console.log('model_name', model_name);

  if(model_name == 'yolov8' || model_name == 'yolov10' || model_name == 'yolov9') {
    transposeND(outs[0], [0, 2, 1], outs[0]);
  }

  if(model_name == 'yolonas') {
    let concat_out = new Mat();
    outs[0] = outs[0].reshape(1, outs[0].size[1]);
    outs[1] = outs[1].reshape(1, outs[1].size[1]);
    hconcat(outs[1], outs[0], concat_out);
    outs[0] = concat_out;
    outs.pop();
    outs[0] = outs[0].reshape(0, [1, outs[0].size[0], outs[0].size[1]]);
  }
  console.log('outs', outs);
  CV_CheckEQ(outs[0].dims, 3, 'Invalid output shape. The shape should be [1, #anchors, nc+5 or nc+4]');
  //CV_CheckEQ(outs[0].size[2] == nc + 5 || outs[0].size[2] == nc + 4, true, 'Invalid output shape: ' + outs[0].size);

  for(let preds of outs) {
    preds = preds.reshape(1, preds.size[1]);

    for(let i = 0; i < preds.rows; ++i) {
      let obj_conf = model_name == 'yolov8' || model_name == 'yolonas' || model_name == 'yolov9' || model_name == 'yolov10' ? 1.0 : preds.at(i, 4);

      if(obj_conf < conf_threshold) continue;

      let scores = preds.row(i).colRange(model_name == 'yolov8' || model_name == 'yolonas' || model_name == 'yolov9' || model_name == 'yolov10' ? 4 : 5, preds.cols);
      let conf;
      let maxLoc = new Point();

      minMaxLoc(
        scores,
        0,
        v => (conf = v),
        0,
        v => (maxLoc = v),
      );

      conf = model_name == 'yolov8' || model_name == 'yolonas' || model_name == 'yolov9' || model_name == 'yolov10' ? conf : conf * obj_conf;
      if(conf < conf_threshold) continue;

      let det = preds.at(i);

      /*console.log('preds.size', preds.size);
      console.log('preds.channels', preds.channels);
      console.log('preds.type', preds.type);
      console.log('det', det);*/

      const cx = preds.at(i + 0);
      const cy = preds.at(i + 1);
      const w = preds.at(i + 2);
      const h = preds.at(i + 3);

      console.log('det', { cx, cy, w, h });

      if(model_name == 'yolonas' || model_name == 'yolov10') boxes.push(new Rect(cx, cy, w, h));
      else boxes.push(new Rect(cx - 0.5 * w, cy - 0.5 * h, cx + 0.5 * w, cy + 0.5 * h));

      classIds.push(maxLoc.x);
      confidences.push(conf);
    }
  }

  const keep_idx = [];
  console.log('boxes', boxes);
  dnn.NMSBoxes(boxes, confidences, conf_threshold, iou_threshold, keep_idx);

  console.log('keep_idx', keep_idx);

  for(let i of keep_idx) {
    keep_classIds.push(classIds[i]);
    keep_confidences.push(confidences[i]);
    keep_boxes.push(boxes[i]);
  }
}

/**
 * @function main
 * @brief Main function
 */
function main() {
  const parser = new CommandLineParser(scriptArgs, keys);
  parser.about('Use this script to run object detection deep learning networks using OpenCV.');

  if(parser.has('help')) {
    parser.printMessage();
    return 0;
  }

  //CV_Assert(parser.has('model'));
  //CV_Assert(parser.has('yolo'));
  let weightPath = findFile(parser.get('model'));
  let yolo_model = parser.get('yolo');
  let nc = parser.get('nc');

  let confThreshold = parser.get('thr');
  let nmsThreshold = parser.get('nms');
  let paddingValue = parser.get('padvalue');
  let swapRB = parser.get('rgb');
  let inpWidth = parser.get('width');
  let inpHeight = parser.get('height');
  let scale = Scalar(parser.get('scale'));
  let mean = Scalar(parser.get('mean'));
  let paddingMode = parser.get('paddingmode');

  if(
    yolo_model != 'yolov5' &&
    yolo_model != 'yolov6' &&
    yolo_model != 'yolov7' &&
    yolo_model != 'yolov8' &&
    yolo_model != 'yolov10' &&
    yolo_model != 'yolov9' &&
    yolo_model != 'yolox' &&
    yolo_model != 'yolonas'
  )
    CV_Error(Error.StsError, 'Invalid yolo model: ' + yolo_model);

  if(parser.has('classes')) getClasses(findFile(parser.get('classes')));

  let net = dnn.readNet(weightPath);
  let backend = parser.get('backend');
  net.setPreferableBackend(backend);
  net.setPreferableTarget(parser.get('target'));

  let cap = new VideoCapture();
  let img = new Mat();
  let isImage = false;
  let isCamera = false;

  if(parser.has('input')) {
    let input = parser.get('input');

    if(input.endsWith('.jpg') || input.endsWith('.png')) {
      img = imread(findFile(input));

      if(img.empty) CV_Error(Error.StsError, 'Cannot read image file: ' + input);

      isImage = true;
    } else {
      cap.open(input);

      if(!cap.isOpened()) CV_Error(Error.StsError, 'Cannot open video ' + input);

      isCamera = true;
    }
  } else {
    let cameraIndex = parser.get('device');
    cap.open(cameraIndex);

    if(!cap.isOpened()) CV_Error(Error.StsError, 'Cannot open camera #' + cameraIndex);

    isCamera = true;
  }

  let size = new Size(inpWidth, inpHeight);
  let imgParams = new dnn.Image2BlobParams(scale, size, mean, swapRB, CV_32F, dnn.DNN_LAYOUT_NCHW, paddingMode, paddingValue);

  let paramNet = new dnn.Image2BlobParams();
  paramNet.scalefactor = scale;
  paramNet.size = size;
  paramNet.mean = mean;
  paramNet.swapRB = swapRB;
  paramNet.paddingmode = paddingMode;

  let outs = [];
  let keep_classIds = [];
  let keep_confidences = [];
  let keep_boxes = [];
  let boxes = [];
  let inp = new Mat();

  while(waitKey(1) < 0) {
    if(isCamera) cap.read(img);

    if(img.empty) {
      console.log('Empty frame');
      waitKey();
      break;
    }

    inp = dnn.blobFromImageWithParams(img, imgParams);

    //console.log('inp',inp);

    const outLayers = net.getUnconnectedOutLayersNames();

    //console.log('outLayers',outLayers);

    net.setInput(inp);
    net.forward(outs, outLayers);

    console.log('outs', outs);

    yoloPostProcessing(outs, keep_classIds, keep_confidences, keep_boxes, confThreshold, nmsThreshold, yolo_model, nc);

    for(let box of keep_boxes) boxes.push(new Rect(cvFloor(box.x), cvFloor(box.y), cvFloor(box.width - box.x), cvFloor(box.height - box.y)));

    paramNet.blobRectsToImageRects(boxes, boxes, img.size);

    console.log('keep_boxes', keep_boxes);

    //console.log('boxes',boxes);

    for(let idx = 0; idx < boxes.length; ++idx) {
      const box = boxes[idx];
      drawPrediction(keep_classIds[idx], keep_confidences[idx], box.x, box.y, box.width + box.x, box.height + box.y, img);
    }

    const kWinName = 'Yolo Object Detector';
    namedWindow(kWinName, WINDOW_NORMAL);
    imshow(kWinName, img);

    outs.splice(0, outs.length);
    keep_classIds.splice(0, keep_classIds.length);
    keep_confidences.splice(0, keep_confidences.length);
    keep_boxes.splice(0, keep_boxes.length);
    boxes.splice(0, boxes.length);

    if(isImage) {
      waitKey();
      break;
    }
  }
}

main();
