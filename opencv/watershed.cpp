#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgproc/types_c.h>

#include <cstdio>
#include <iostream>

using namespace cv;
using namespace std;

static void
help() {
  cout << "\nThis program demonstrates the famous watershed segmentation algorithm in OpenCV: "
          "watershed()\n"
          "Usage:\n"
          "./watershed [image_name -- default is fruits.jpg]\n"
       << endl;

  cout << "Hot keys: \n"
          "\tESC - quit the program\n"
          "\tr - restore the original image\n"
          "\tw or SPACE - run watershed segmentation algorithm\n"
          "\t\t(before running it, *roughly* mark the areas to segment on the image)\n"
          "\t  (before that, roughly outline several markers on the image)\n";
}
cv::Mat markerMask, img;
cv::Point prevPt(-1, -1);

static void
onMouse(int event, int x, int y, int flags, void*) {
  if(x < 0 || x >= img.cols || y < 0 || y >= img.rows)
    return;
  if(event == cv::EVENT_LBUTTONUP || !(flags & cv::EVENT_FLAG_LBUTTON))
    prevPt = cv::Point(-1, -1);
  else if(event == cv::EVENT_LBUTTONDOWN)
    prevPt = cv::Point(x, y);
  else if(event == cv::EVENT_MOUSEMOVE && (flags & cv::EVENT_FLAG_LBUTTON)) {
    cv::Point pt(x, y);
    if(prevPt.x < 0)
      prevPt = pt;
    line(markerMask, prevPt, pt, Scalar::all(255), 5, 8, 0);
    line(img, prevPt, pt, Scalar::all(255), 5, 8, 0);
    prevPt = pt;
    imshow("image", img);
  }
}

int
main(int argc, char** argv) {
  char* filename = argc >= 2 ? argv[1] : (char*)"fruits.jpg";
  cv::Mat img0 = imread(filename, 1), imgGray;

  if(img0.empty()) {
    cout << "Couldn'g open image " << filename << ". Usage: watershed <image_name>\n";
    return 0;
  }
  help();
  namedWindow("image", 1);

  img0.copyTo(img);
  cvtColor(img, markerMask, COLOR_BGR2GRAY);
  cvtColor(markerMask, imgGray, COLOR_GRAY2BGR);
  markerMask = Scalar::all(0);
  imshow("image", img);
  setMouseCallback("image", onMouse, 0);

  for(;;) {
    int c = waitKey(0);

    if((char)c == 27)
      break;

    if((char)c == 'r') {
      markerMask = Scalar::all(0);
      img0.copyTo(img);
      imshow("image", img);
    }

    if((char)c == 'w' || (char)c == ' ') {
      int i, j, compCount = 0;
      std::vector<std::vector<cv::Point>> contours;
      std::vector<Vec4i> hierarchy;

      findContours(markerMask, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);

      if(contours.empty())
        continue;
      cv::Mat markers(markerMask.size(), CV_32S);
      markers = Scalar::all(0);
      int idx = 0;
      for(; idx >= 0; idx = hierarchy[idx][0], compCount++)
        drawContours(markers, contours, idx, Scalar::all(compCount + 1), -1, 8, hierarchy, INT_MAX);

      if(compCount == 0)
        continue;

      std::vector<Vec3b> colorTab;
      for(i = 0; i < compCount; i++) {
        int b = theRNG().uniform(0, 255);
        int g = theRNG().uniform(0, 255);
        int r = theRNG().uniform(0, 255);

        colorTab.push_back(Vec3b((uchar)b, (uchar)g, (uchar)r));
      }

      double t = (double)getTickCount();
      watershed(img0, markers);
      t = (double)getTickCount() - t;
      printf("execution time = %gms\n", t * 1000. / getTickFrequency());

      cv::Mat wshed(markers.size(), CV_8UC3);

      // paint the watershed image
      for(i = 0; i < markers.rows; i++)
        for(j = 0; j < markers.cols; j++) {
          int index = markers.at<int>(i, j);
          if(index == -1)
            wshed.at<Vec3b>(i, j) = Vec3b(255, 255, 255);
          else if(index <= 0 || index > compCount)
            wshed.at<Vec3b>(i, j) = Vec3b(0, 0, 0);
          else
            wshed.at<Vec3b>(i, j) = colorTab[index - 1];
        }

      wshed = wshed * 0.5 + imgGray * 0.5;
      imshow("watershed transform", wshed);
    }
  }

  return 0;
}
