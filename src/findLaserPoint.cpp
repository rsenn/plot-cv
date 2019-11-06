#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv/cvaux.h>
#include <opencv/cxcore.h>
#include <opencv2/highgui.hpp>
#include <fstream>
#include <string>
#include <sstream>
using namespace std;
using namespace cv;

int thresholdValue = 155;
int thresholdValueHSV = 100;
int mouseY = 100, mouseX = 100;
cv::Point center(0, 0);
cv::Point lasercv::Point(-1, -1);
void
trackbar(int input, void* u) {
  thresholdValue = input;
}
void
trackbar2(int input, void* u) {
  thresholdValueHSV = input;
}
void
mouseCall1(int eventIndex, int x, int y, int flag, void* param) {
  if(eventIndex == CV_EVENT_LBUTTONDOWN) {
    mouseY = y;
    mouseX = x;
  }
}
void
brushSideWhite(cv::Mat& img) {
  int starti = 0, endi = 0;
  int startj = 0, endj = 0;

  starti = 0, endi = img.rows / 5;
  startj = 0, endj = img.cols;
  for(int i = starti; i < endi; i++)
    for(int j = startj; j < endj; j++) img.at<uchar>(i, j) = 255;

  starti = 3 * img.rows / 5, endi = img.rows;
  startj = 0, endj = img.cols;
  for(int i = starti; i < endi; i++)
    for(int j = startj; j < endj; j++) img.at<uchar>(i, j) = 255;

  starti = img.rows / 5, endi = 4 * img.rows / 5;
  startj = 0, endj = img.cols / 5;
  for(int i = starti; i < endi; i++)
    for(int j = startj; j < endj; j++) img.at<uchar>(i, j) = 255;

  starti = img.rows / 5, endi = 4 * img.rows / 5;
  startj = 4 * img.cols / 5, endj = img.cols;
  for(int i = starti; i < endi; i++)
    for(int j = startj; j < endj; j++) img.at<uchar>(i, j) = 255;
}
cv::Point
findBrightPoint(cv::Mat& img) {
  // imshow("tt",img);
  std::vector<std::vector<cv::Point>> contours;
  std::vector<Vec4i> hierarchy;
  cv::Mat imgProcCopy = img.clone();
  findContours(imgProcCopy, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
  if(contours.size() == 0)
    return cv::Point(-1, -1);
  int maxArea = 0, maxAreaIndex = -1;
  for(int i = 0; i < contours.size(); i++) {
    if(contourArea(contours[i]) > maxArea) {
      maxArea = contourArea(contours[i]);
      maxAreaIndex = i;
    }
  }
  if(maxAreaIndex == -1)
    return cv::Point(-1, -1);

  int x = 0, y = 0;
  for(int i = 0; i < contours[maxAreaIndex].size(); i++) {
    x += contours[maxAreaIndex][i].x;
    y += contours[maxAreaIndex][i].y;
  }
  x /= contours[maxAreaIndex].size();
  y /= contours[maxAreaIndex].size();
  cv::Point temp(x, y);
  return temp;
}
void
invertColor(cv::Mat& img) {
  for(int i = 0; i < img.rows; i++)
    for(int j = 0; j < img.cols; j++) img.at<uchar>(i, j) = 255 - img.at<uchar>(i, j);
}
int
main() {
  cv::Mat imgRaw;
  cv::Mat imgProc;
  cv::Mat imgHSV;
  cv::Mat imgTemp;
  cv::Mat imgToMapProc;
  cv::Mat imgToMap(528, 459, CV_8UC3);
  VideoCapture cap(1);
  cvNamedWindow("demoRaw", 1);
  // cvNamedWindow("demoProc",0);
  cvNamedWindow("demoMap", 1);
  createTrackbar("Thre", "demoProc", &thresholdValue, 255, trackbar);
  // createTrackbar("Thre","demoHSV",&thresholdValueHSV,255,trackbar2);
  setMouseCallback("demoRaw", mouseCall1);

  while(waitKey(30) != 27) {
    cap >> imgRaw;
    cv::Mat imgRawCopy = imgRaw.clone();
    cvtColor(imgRaw, imgProc, CV_BGR2GRAY);
    cvtColor(imgRaw, imgHSV, CV_BGR2HSV_FULL);
    threshold(imgProc, imgProc, thresholdValue, 255, 0);
    std::vector<std::vector<cv::Point>> contours;
    std::vector<std::vector<cv::Point>> wall;
    std::vector<Vec4i> hierarchy;
    cv::Mat imgProcCopy = imgProc.clone();
    findContours(imgProcCopy, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
    int left = 1000, right = 0, up = 1000, down = 0;
    int maxArea = 0, maxAreaIndex = -1;
    for(int i = 0; i < contours.size(); i++) {
      int areaSize = contourArea(contours[i]);
      if(areaSize > maxArea) {
        maxArea = areaSize;
        maxAreaIndex = i;
      }
    }

    cv::Point2f leftUp, rightUp, leftDown, rightDown;
    left = 1000, right = 0, up = 1000, down = 0;
    int tempLeftU = 1000, tempRightU = 0, tempLeftUIndex = -1, tempRightUIndex = -1;
    int tempLeftD = 1000, tempRightD = 0, tempLeftDIndex = -1, tempRightDIndex = -1;
    if(maxAreaIndex != -1) {
      for(int j = 0; j < contours[maxAreaIndex].size(); j++) {
        imgRaw.at<Vec3b>(contours[maxAreaIndex][j].y, contours[maxAreaIndex][j].x) = Vec3b(0, 0, 255);
        if(contours[maxAreaIndex][j].x < left)
          left = contours[maxAreaIndex][j].x;
        if(contours[maxAreaIndex][j].x > right)
          right = contours[maxAreaIndex][j].x;
        if(contours[maxAreaIndex][j].y < up)
          up = contours[maxAreaIndex][j].y;
        if(contours[maxAreaIndex][j].y > down)
          down = contours[maxAreaIndex][j].y;
      }
      for(int j = 0; j < contours[maxAreaIndex].size(); j++) {
        if(abs(contours[maxAreaIndex][j].y - up) < 5) { // Deal with the upside points
          if(contours[maxAreaIndex][j].x < tempLeftU) {
            tempLeftU = contours[maxAreaIndex][j].x;
            tempLeftUIndex = j;
          }
          if(contours[maxAreaIndex][j].x > tempRightU) {
            tempRightU = contours[maxAreaIndex][j].x;
            tempRightUIndex = j;
          }
        }

        if(abs(contours[maxAreaIndex][j].y - down) < 10) { // Deal with the down side points
          if(contours[maxAreaIndex][j].x < tempLeftD) {
            tempLeftD = contours[maxAreaIndex][j].x;
            tempLeftDIndex = j;
          }
          if(contours[maxAreaIndex][j].x > tempRightD) {
            tempRightD = contours[maxAreaIndex][j].x;
            tempRightDIndex = j;
          }
        }
      }
      leftUp = contours[maxAreaIndex][tempLeftUIndex];
      rightUp = contours[maxAreaIndex][tempRightUIndex];
      leftDown = contours[maxAreaIndex][tempLeftDIndex];
      rightDown = contours[maxAreaIndex][tempRightDIndex];
      circle(imgRaw, leftUp, 2, Scalar(0, 255, 0));
      circle(imgRaw, rightUp, 2, Scalar(0, 255, 0));
      circle(imgRaw, leftDown, 2, Scalar(0, 255, 0));
      circle(imgRaw, rightDown, 2, Scalar(0, 255, 0));

      line(imgRaw, leftUp, rightUp, Scalar(0, 255, 0));
      line(imgRaw, leftUp, leftDown, Scalar(0, 255, 0));
      line(imgRaw, leftDown, rightDown, Scalar(0, 255, 0));
      line(imgRaw, rightDown, rightUp, Scalar(0, 255, 0));
      // line(imgRaw,cv::Point(0,mouseY),cv::Point(imgRaw.cols-1,mouseY),Scalar(0,0,255),1);
      // line(imgRaw,cv::Point(mouseX,0),cv::Point(mouseX,imgRaw.rows),Scalar(0,0,255),1);

      std::vector<cv::Point2f> corners_trans(4);
      std::vector<cv::Point2f> corners(4);
      corners[0] = leftUp;
      corners[1] = rightUp;
      corners[2] = rightDown;
      corners[3] = leftDown;

      Vec4i uu;
      uu[0] = 0;

      corners_trans[0] = cv::Point2f(0, 0);
      corners_trans[1] = cv::Point2f(imgToMap.cols - 1, 0);
      corners_trans[2] = cv::Point2f(imgToMap.cols - 1, imgToMap.rows - 1);
      corners_trans[3] = cv::Point2f(0, imgToMap.rows - 1);

      cv::Mat transform = getPerspectiveTransform(corners, corners_trans);
      warpPerspective(imgRawCopy, imgToMap, transform, imgToMap.size());

      cv::Mat imgPoint;

      cvtColor(imgToMap, imgToMapProc, CV_BGR2GRAY);
      threshold(imgToMapProc, imgPoint, 254, 255, 0);
      threshold(imgToMapProc, imgToMapProc, thresholdValue, 255, 0);

      cv::Point brightcv::Point = findBrightPoint(imgPoint);

      brushSideWhite(imgToMapProc); //将边缘部分的噪点删除掉

      std::vector<Vec4i> lines;
      invertColor(imgToMapProc);
      HoughLinesP(imgToMapProc, lines, 1, CV_PI / 180, 30, 30, 10);

      cv::Mat imgToMapProcCopy(imgToMapProc.size(), imgToMapProc.type(), Scalar(0));

      int sumX = 0, sumY = 0, sumCountX = 0, sumCountY = 0;
      // cout<<endl;
      for(int i = 0; i < lines.size(); i++) {
        line(imgToMapProcCopy, cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2], lines[i][3]), Scalar(255));
        // cout<<"deltX:"<<abs(lines[i][0]-lines[i][2])<<" deltY:"<<abs(lines[i][1]-lines[i][3])<<endl;
        if(abs(lines[i][0] - lines[i][2]) < 20) {
          sumX += (lines[i][0] + lines[i][2]) / 2;
          sumCountX++;
        }
        if(abs(lines[i][1] - lines[i][3]) < 20) {
          sumY += (lines[i][1] + lines[i][3]) / 2;
          sumCountY++;
        }
      }
      // cout<<"sumX:"<<sumX<<" sumY:"<<sumY<<" countX:"<<sumCountX<<" countY:"<<sumCountY<<endl;
      if(sumCountX != 0) {
        sumX = sumX / sumCountX;
        // cout<<"avrX:"<<sumX;
      }
      if(sumCountY != 0) {
        sumY = sumY / sumCountY;
        // cout<<" avrY:"<<sumY<<endl;
      }

      if(sumCountX != 0 && sumCountY != 0) {
        center.x = sumX;
        center.y = sumY;
        circle(imgToMapProcCopy, center, 7, Scalar(255));
      }
      /*
      if(lines.size()==2)
      {
          line(imgToMapProcCopy,cv::Point(lines[0][0],lines[0][1]),cv::Point(lines[0][2],lines[0][3]),Scalar(255));
          line(imgToMapProcCopy,cv::Point(lines[1][0],lines[1][1]),cv::Point(lines[1][2],lines[1][3]),Scalar(255));
      }
      */
      if(brightcv::Point.x != -1) {
        circle(imgToMapProcCopy, brightcv::Point, 3, Scalar(255));
        lasercv::Point.x = (brightcv::Point.x - center.x) / 6;
        lasercv::Point.y = (center.y - brightcv::Point.y) / 6;
        istringstream info;
        String coordInfo = "(";
        String tempStr;
        info.clear();
        info << (int)lasercv::Point.x;
        info >> tempStr;
        coordInfo += tempStr;
        coordInfo += ",";
        info.clear();
        info << (int)lasercv::Point.y;
        info >> tempStr;
        coordInfo += tempStr;
        coordInfo += ")";
        putText(imgToMapProcCopy, coordInfo, brightcv::Point - cv::Point(18, 10), 0, 0.3, Scalar(255));
      }

      imshow("demoMap", imgToMapProcCopy);
    }
    imshow("demoRaw", imgRaw);
    // imshow("demoProc",imgProc);
  }
}

/*
imgTemp=cv::Mat(imgHSV.size(),CV_8UC1,Scalar(0));
        int delt=10;
        for(int i=0;i<imgHSV.rows;i++)
        {
            for(int j=0;j<imgHSV.cols;j++)
            {
                if((imgHSV.at<Vec3b>(i,j).val[0]>thresholdValueHSV-delt)&&(imgHSV.at<Vec3b>(i,j).val[0]<thresholdValueHSV+delt))
                {
                    imgTemp.at<uchar>(i,j)=255;
                }
                else
                {
                    imgTemp.at<uchar>(i,j)=0;
                }
            }
        }
        blur(imgTemp,imgTemp,Size(4,4));
*/