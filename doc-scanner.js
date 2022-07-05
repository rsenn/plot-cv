import { Contour, CHAIN_APPROX_SIMPLE, COLOR_BGR2GRAY, CV_32FC2, Canny, FILLED, FONT_HERSHEY_PLAIN, GaussianBlur, MORPH_RECT, Mat, Point, RETR_EXTERNAL, Rect, Size, VideoCapture, approxPolyDP, arcLength, contourArea, cvtColor, dilate, drawContours, findContours, getPerspectiveTransform, getStructuringElement, imread, imshow, putText, resize, waitKey, warpPerspective } from 'opencv';

import { Console } from 'console';

let imgOriginal,
  imgGray = new Mat(),
  imgCanny = new Mat(),
  imgBlur = new Mat(),
  imgDilate = new Mat();
let initialPoints, finalPoints;
let w = 420,
  h = 596;

function preProcessing(img) {
  cvtColor(img, imgGray, COLOR_BGR2GRAY);

  GaussianBlur(imgGray, imgBlur, new Size(3, 3), 3, 0);
  Canny(imgBlur, imgCanny, 25, 75);

  let kernel = getStructuringElement(MORPH_RECT, new Size(3, 3));

  dilate(imgCanny, imgDilate, kernel);
  return imgDilate;
}

function getContours(imgMask) {
  let contours = [];
  let hierarchy = [];

  findContours(imgMask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
  let contourPoly = new Array(contours.length);
  let boundRect = new Array(contours.length);
  let biggest = [];
  let maxArea = -10;

  console.log('contours', contours);

  for(let i = 0; i < contours.length; i++) {
    let c = Contour.from(contours[i]);
    // console.log('c', c);
    let area = contourArea(c);

    console.log('area', area);

    if(area >= 1000) {
      console.log('ENTERED LOOP 1');
      let peri = arcLength(c, true);
      console.log('peri', peri);
      if(peri === Infinity) peri = arcLength(c, true);
      if(peri === Infinity) console.log('c', c);
      if(peri === Infinity) peri = 50;
      contourPoly[i] = new Mat(c.length, 1, CV_32FC2);
      approxPolyDP(c, contourPoly[i], 0.02 * peri, true);

      if(area > maxArea && contourPoly[i].length == 4) {
        console.log('ENTERED LOOP 2');
        //drawContours(imgOriginal, contourPoly, i, Scalar(255, 0, 255), 3);
        maxArea = area;
        biggest = [contourPoly[i][0], contourPoly[i][1], contourPoly[i][2], contourPoly[i][3]];
      }

      //cout << contourPoly[i] << endl;
    }
  }
  return biggest;
}

function drawPoints(points, color) {
  for(let i = 0; i < points.length; i++) {
    circle(imgOriginal, points[i], 5, color, FILLED);
    putText(imgOriginal, String(i), points[i], FONT_HERSHEY_PLAIN, 3, color, 5);
  }
}

function reOrder(points) {
  let newPoints = [];
  let sumPoints = [],
    subPoints = [];

  for(let i = 0; i < 4; i++) {
    sumPoints.push(points[i].x + points[i].y);
    subPoints.push(points[i].x - points[i].y);
  }

  newPoints.push(points[min_element(sumPoints.begin(), sumPoints.end()) - sumPoints.begin()]); // 0
  newPoints.push(points[max_element(subPoints.begin(), subPoints.end()) - subPoints.begin()]); //1
  newPoints.push(points[min_element(subPoints.begin(), subPoints.end()) - subPoints.begin()]); //2
  newPoints.push(points[max_element(sumPoints.begin(), sumPoints.end()) - sumPoints.begin()]); //3

  return newPoints;
}

function getWarp(img, points, w, h) {
  let src = [points[0], points[1], points[2], points[3]];
  let dest = [new Point(0, 0), new Point(w, 0), new Point(0, h), new Point(w, h)];
  let inter = getPerspectiveTransform(src, dest);
  let imgWarp = new Mat();
  warpPerspective(img, imgWarp, inter, Size(w, h));
  return imgWarp;
}

function main(...args) {
  globalThis.console = new Console(std.err, {
    inspectOptions: { compact: false, customInspect: true, maxArrayLength: 20, maxStringLength: 100, numberBase: 10 }
  });

  console.log('args', args);

  /*if(args.length == 0)
    args=[0];
*/
  for(let arg of args) {
    //let cap = new VideoCapture(0);
    let imgCrop, imgWarp, imgThreshold;

    let path = arg;
    imgOriginal = imread(path);

    let cropFactor = 10;
    let roi = new Rect(cropFactor, cropFactor, w - 2 * cropFactor, h - 2 * cropFactor);

    console.log('imgOriginal', imgOriginal);

    imgThreshold = preProcessing(imgOriginal);

    initialPoints = getContours(imgThreshold);
    console.log('initialPoints', initialPoints);

    finalPoints = reOrder(initialPoints);

    imgWarp = getWarp(imgOriginal, finalPoints, w, h);

    imgCrop = imgWarp(roi);

    imshow('Image', imgOriginal);
    imshow('ImageThreshold', imgThreshold);
    imshow('ImageWarp', imgWarp);
    imshow('ImageCropped', imgCrop);

    waitKey(0);
  }
}

main(...scriptArgs.slice(1));
