// distance Measurement.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "cv.h"
#include "highgui.h"
#include "opencv2/imgproc/imgproc_c.h"
#include <fstream>
#include "math.h"

int
_tmain(int argc, _TCHAR* argv[]) {
  using namespace std;
//  using namespace cv;

  cv::Mat img, img_gray, channel[3];
  cv::VideoCapture cam(1);
  double distance = 0;

  // FILE *data;
  // data = fopen("data320.csv","a");

  cam.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
  cam.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
  cam.set(cv::CAP_PROP_CONVERT_RGB, 1);
  cv::namedWindow("Frame", cv::WINDOW_AUTOSIZE);
  while(cv::waitKey(10) != 'a') {
    cam >> img;
    cv::cvtColor(img, img_gray, cv::COLOR_RGB2GRAY);
    cv::split(img, channel);
    cv::subtract(channel[2], img_gray, img_gray);
    // cv::convertScaleAbs(img, img);
    cv::threshold(img_gray, img_gray, 90, 255, THRESH_BINARY);

    cv::erode(img_gray, img_gray, cv::Mat(), cv::Point(-1, -1), 4);
    cv::dilate(img_gray, img_gray, cv::Mat(), cv::Point(-1, -1), 4);

    vector<vector<cv::Size>> contors;
    vector<cv::Vec4i> heirarcy;
    cv::findContours(img_gray, contors, heirarcy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    vector<cv::Rect> boundRect(contors.size());
    vector<vector<cv::Point>> contor_poly(contors.size());

    for(int i = 0; i < contors.size(); i++) {
      cv::approxPolyDP(cv::Mat(contors[i]), contor_poly[i], 3, true);
      boundRect[i] = cv::boundingRect(cv::Mat(contor_poly[i]));
    }
    int max_index = 0, max_area = 0;
    for(int i = 0; i < boundRect.size(); i++) {
      int a = boundRect[i].area();
      cv::rectangle(img, boundRect[i].tl(), boundRect[i].br(), cv::Scalar(255, 255, 0), 2, 8, 0);
      if(a > max_area) {
        max_area = a;
        max_index = i;
      }
    }
    int confidence = 0;
    for(int i = 0; i < boundRect.size(); i++) {
      if((boundRect[i].x < boundRect[max_index].x + boundRect[max_index].width &&
          boundRect[i].x > boundRect[max_index].x - int(0.1 * boundRect[max_index].width)) &&
         (boundRect[i].y > boundRect[max_index].y))
        confidence += 45;
    }
    if(boundRect.size() > 0) {
      if(confidence > 99)
        confidence = 0;
      // try{
      // cv::Mat sub_image = cv::Mat(img, cv::Rect(max(boundRect[max_index].x-30, 0),
      // max(boundRect[max_index].y-30, 0), min(int(boundRect[max_index].width*1.75), img.cols -
      // boundRect[max_index].x+30), min(boundRect[max_index].height*3, img.rows -
      // boundRect[max_index].y+30))); cv::imshow("Frame", sub_image);
      //}catch(int e){
      //	cout<<"cv::Error occured"<<endl;
      //}
      cv::rectangle(img, boundRect[max_index].tl(), boundRect[max_index].br(), cv::Scalar(0, 255, 0), 2, 8, 0);

      // fprintf(data,"%d , %d , %d\n", boundRect[max_index].width, boundRect[max_index].height,
      // boundRect[max_index].area());
      distance = 8414.7 * cv::pow(boundRect[max_index].area(), -0.468);
      cout << distance << " cm."
           << " Confidence: " << confidence << endl;
      cv::imshow("Frame", img);

    }

    else
      cv::imshow("Frame", img);
  }

  // fflush(data);
  // fclose(data);
  cam.release();
  return 0;
}
