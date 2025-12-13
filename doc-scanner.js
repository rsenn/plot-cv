import { HSLA } from './lib/color/hsla.js';
import { Console } from 'console';
import { approxPolyDP, arcLength, Canny, CHAIN_APPROX_SIMPLE, COLOR_BGR2GRAY, COLOR_GRAY2BGR, Contour, contourArea, cvtColor, dilate, drawCircle, drawContours, FILLED, findContours, FONT_HERSHEY_PLAIN, GaussianBlur, getPerspectiveTransform, getStructuringElement, imread, imshow, imwrite, Mat, MORPH_RECT, Point, putText, Rect, RETR_EXTERNAL, Size, waitKey, warpPerspective } from 'opencv';

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

  let kernel = getStructuringElement(MORPH_RECT, new Size(1, 1));
  // return imgCanny;

  dilate(imgCanny, imgDilate, kernel);
  return imgDilate;
}

function i2color(i) {
  return parseInt('0x' + new HSLA(i, 100, 50).toRGBA().toString().slice(1));
  return i * 2378494; //((i&0b11111) << 3) |((((i>>> 5)&0b11111) << 3) << 8) |((((i>>> 10)&0b11111) << 3) << 16);
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
  for(let i = 0; i < contours.length; i++) {}

  cvtColor(imgDilate, imgDilate, COLOR_GRAY2BGR);
  let j = 0;
  for(let i = 0; i < contours.length; i++) {
    let c = Contour.from(contours[i]);
    //console.log('c', c.toString());
    let area = contourArea(c);

    //console.log('area', area);

    if(area >= 1000) {
      drawContours(imgDilate, [c], 0, i2color(j), 3);
      j++;
      console.log('ENTERED LOOP 1');
      let peri = arcLength(c, true);
      console.log('peri', peri);
      /*if(peri === Infinity) peri = arcLength(c, true);
      if(peri === Infinity) console.log('c', c);
      */ if(peri === Infinity) peri = 50;
      //contourPoly[i] = new Mat(c.length, 1, CV_32FC2);
      contourPoly[i] = new Contour();
      approxPolyDP(c, contourPoly[i], peri / 50, true);
      //  console.log('contourPoly[i]', contourPoly[i]);

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

function getBiggest(imgMask) {
  let contours = [];
  let hierarchy = [];

  findContours(imgMask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
  let contourPoly = new Array(contours.length);
  let boundRect = new Array(contours.length);
  let maxArea = -10;

  console.log('contours', contours);
  console.log('hierarchy', hierarchy);
  for(let i = 0; i < contours.length; i++) {}

  cvtColor(imgDilate, imgDilate, COLOR_GRAY2BGR);

  let areas = [];
  let j = 0;
  for(let i = 0; i < contours.length; i++) {
    let c = Contour.from(contours[i]);
    //console.log('c', c.toString());
    let area = c.area; //contourArea(contours[i]);
    let peri = arcLength(c, true);
    //    if(peri === Infinity) peri = c.arcLength(true);
    if(peri === Infinity) peri = c.arcLength(true);

    if(peri !== Infinity) {
      let contour = new Contour();
      console.log('c', c);
      approxPolyDP(c, contour, 0.02 * peri, true);
      console.log('contour', contour);

      drawContours(imgDilate, [c], 0, i2color(i), 1);

      if(contour.length == 4) areas.push([area, peri, contour, i]);
    }
  }

  areas.sort((a, b) => b[0] - a[0]);
  //areas.sort((a, b) => b[1] - a[1]);

  console.log('areas', areas);

  let [area, peri, contour, index] = areas[0];
  console.log('area', area);
  console.log('peri', peri);
  console.log('length', contour.length);
  console.log('contour', contour);

  drawContours(imgDilate, [contour], 0, i2color(0), 3);

  return contours[index];
}

function drawPoints(points, color) {
  for(let i = 0; i < points.length; i++) {
    drawCircle(imgOriginal, points[i], 5, color, FILLED);
    putText(imgOriginal, String(i), points[i], FONT_HERSHEY_PLAIN, 3, color, 5);
  }
}

function min_element(arr) {
  let index = 0;
  let minVal = arr[0];
  for(let i = 1; i < arr.length; i++) {
    if(arr[i] <= minVal) {
      minVal = arr[i];
      index = i;
    }
  }
  return index;
}

function max_element(arr) {
  let index = 0;
  let maxVal = arr[0];
  for(let i = 1; i < arr.length; i++) {
    if(arr[i] >= maxVal) {
      maxVal = arr[i];
      index = i;
    }
  }
  return index;
}

function reOrder(points) {
  let newPoints = [];
  let sumPoints = [],
    subPoints = [];

  console.log('points', points);
  for(let i = 0; i < 4; i++) {
    sumPoints.push(Math.round(points[i].x + points[i].y));
    subPoints.push(Math.round(points[i].x - points[i].y));
  }

  console.log('sumPoints', sumPoints);
  console.log('subPoints', subPoints);

  let indices = [min_element(sumPoints), max_element(subPoints), min_element(subPoints), max_element(sumPoints)];

  console.log('indices', indices);

  newPoints.push(points[min_element(sumPoints)]); // 0
  newPoints.push(points[max_element(subPoints)]); //1
  newPoints.push(points[min_element(subPoints)]); //2
  newPoints.push(points[max_element(sumPoints)]); //3

  return newPoints;
}

function getWarp(img, points, w, h) {
  let src = [points[0], points[1], points[2], points[3]];
  let dest = [new Point(0, 0), new Point(w, 0), new Point(0, h), new Point(w, h)];
  console.log('src', src);
  console.log('dest', dest);
  let inter = getPerspectiveTransform(src, dest);
  console.log('inter', inter);

  let imgWarp = new Mat();
  warpPerspective(img, imgWarp, inter, new Size(w, h));
  return imgWarp;
}

function main(...args) {
  globalThis.console = new Console(std.err, {
    inspectOptions: { compact: false, customInspect: true, maxArrayLength: 20, maxStringLength: 100, numberBase: 10 }
  });

  console.log('args', args);
  let key;

  for(let arg of args) {
    //let cap = new VideoCapture(0);
    let imgCrop, imgWarp, imgThreshold;

    let path = arg;
    imgOriginal = imread(path);

    let cropFactor = 10;
    let roi = new Rect(cropFactor, cropFactor, w - 2 * cropFactor, h - 2 * cropFactor);

    console.log('imgOriginal', imgOriginal);

    imgThreshold = preProcessing(imgOriginal);

    initialPoints = getBiggest(imgThreshold);
    console.log('initialPoints', initialPoints);

    finalPoints = reOrder(initialPoints);
    console.log('finalPoints', finalPoints);
    imgWarp = getWarp(imgOriginal, finalPoints, w, h);

    imgCrop = imgWarp(roi);

    imshow('Image', imgOriginal);

    for(let i = 0; i < 4; i++) {
      drawCircle(imgThreshold, initialPoints[i], 8, i2color(i * 10 + 120, 2), 2);
    }

    imshow('ImageThreshold', imgThreshold);
    imwrite('ImageDilate.png', imgDilate);
    imwrite('ImageThreshold.png', imgThreshold);
    imshow('ImageWarp', imgWarp);
    imwrite('ImageWarp.png', imgWarp);
    imshow('imgCrop', imgCrop);
    imwrite('imgCrop.png', imgCrop);

    for(;;) {
      key = waitKey(10);
      if(key != -1) console.log('key', key);
      if(key == 81 || key == 27) break;
    }
  }
}

main(...scriptArgs.slice(1));