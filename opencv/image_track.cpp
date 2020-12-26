//
//  image_track.cpp
//  main
//
//  Created by 陳家麒 on 2017/7/24.
//
//

#include <image_track.hpp>
using namespace std;
using namespace cv;
void static onMouse(int event, int x, int y, int, void*) {
  static cv::Point origin;

  if(selectObject) {
    /* check mouse select area which The coordinates of the upper left corner and the length and
     * width of the area */
    selection.x = MIN(x, origin.x);
    selection.y = MIN(y, origin.y);
    selection.width = std::abs(x - origin.x);
    selection.height = std::abs(y - origin.y);
    // test
    cout << "x=" << selection.x << "\ty=" << selection.y << "\t width=" << selection.width << "\t height"
         << selection.height << endl;
    cout << selection << endl;
    // & is cv::Rect loaded
    // That the two regions take the intersection, the main purpose is to deal with when the mouse
    // in the selected area to remove the screen
    /*Rect argument (x,y,width,height)*/
    selection &= cv::Rect(0, 0, image.cols, image.rows);
    // cout << "image.cols = " << image.cols << "\timage.rows" <<  image.rows << endl;
    cout << "after selection = " << selection << endl;
  }
  switch(event) {
    case cv::EVENT_LBUTTONDOWN:
      origin = cv::Point(x, y);
      selection = cv::Rect(x, y, 0, 0);
      selectObject = true;

      break;
    case cv::EVENT_LBUTTONUP:
      selectObject = false;
      if(selection.width > 0 && selection.height > 0)
        trackObject = -1;
      break;
  }
}
image_track::image_track() {

  // hsize = 16;                   // 计算直方图所必备的内容
  // hranges[0] = 0;
  // hranges[1] = 180;// 计算直方图所必备的内容
  // phranges = hranges;  // 计算直方图所必备的内容
  // std::cout << phranges << std::endl;
}

/*using mouse select track object*/
void
image_track::track_start(cv::VideoCapture video) {
  try {
    // video.set(cv::CAP_PROP_POS_FRAMES,300);
    cv::namedWindow("CamShift at Rozen");
    // cv::namedWindow("testing on HSV");
    cv::namedWindow("Hue");
    cv::namedWindow("backproj");
    /*register mouse event callback function,third param is user provid callback)*/
    cv::setMouseCallback("CamShift at Rozen", onMouse, 0);
    while(true) {
      // video >> frame;
      video.grab();
      video.retrieve(frame);

      if(frame.empty())
        break;
      frame.copyTo(image);

      // exchange to HSV space
      cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
      if(trackObject) {

        cv::inRange(hsv, Scalar(0, 0, 0), Scalar(256, 256, 256), mask);

        imshow("testing on HSV", mask);
        // separate the hue channel
        int ch[] = {0, 0};
        hue.create(hsv.size(), hsv.depth());
        cv::mixChannels(&hsv, 1, &hue, 1, ch, 1);
        imshow("Hue", hue);

        if(trackObject < 0) {
          cv::Mat roi(hue, selection), maskroi(mask, selection);
          // calculates the histogram of the area where the ROI located
          calcHist(&roi, 1, 0, maskroi, hist, 1, &hsize, &phranges);
          normalize(hist, hist, 0, 255, CV_MINMAX);

          trackWindow = selection;
          cout << "trackWindows->" << trackWindow << endl;
          trackObject = 1;
        }

        calcBackProject(&hue, 1, 0, hist, backproj, &phranges);

        backproj &= mask;
        imshow("backproj", backproj);
        cv::RotatedRect trackBox =
            CamShift(backproj, trackWindow, cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1));
        if(trackWindow.area() <= 1) {
          int cols = backproj.cols, rows = backproj.rows, r = (MIN(cols, rows) + 5) / 6;
          trackWindow = cv::Rect(trackWindow.x - r, trackWindow.y - r, trackWindow.x + r, trackWindow.y + r) &
                        cv::Rect(0, 0, cols, rows);
        }
        ellipse(image, trackBox, cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
      }
      if(selectObject && selection.width > 0 && selection.height > 0) {
        cv::Mat roi(image, selection);
        bitwise_not(roi, roi);
      }

      imshow("CamShift at Rozen", image);
      char c = (char)cv::waitKey(1000 / 15.0);
      if(c == 27)
        break;
    }
  } catch(std::exception& e) {
    std::cout << &e << std::endl;
    cv::destroyAllWindows();
    video.release();
  }
}
