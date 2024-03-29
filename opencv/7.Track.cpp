#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <opencv2/core/utility.hpp>
#include <opencv2/tracking.hpp>

#include <time.h>

// using namespace cv;
using namespace std;

cv::VideoCapture cap(0);
cv::Mat frame;
cv::Mat frame_gray;
cv::Mat frame_result;
void
Camera_capture() {
  cap >> frame;
  if(frame.empty()) {
    printf("ERROR: khong the bat hinh\r\n");
    exit(-1); // Thoat khoi chuong trinh
  }
}

// Cac kieu chuyen doi nguong
// 0: binary, 1: binary dao (inverted), 2: cat ngon (truncate),
// 3: To zero, 4: To Zero dao (inverted), 5: Color
int threshold_value = 40;
int max_value = 255;
int threshold_type = 5;
void
Threshold_Convert(int, void*) {
  if(threshold_type < 5) {
    // Chuyen ve thang xam
    cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);
    // Chuyen ve nguong
    cv::threshold(frame_gray, frame_result, threshold_value, max_value, threshold_type);
  } else {
    frame.copyTo(frame_result); // Giu nguyen mau sac
  }
}

int
main() {
  if(!cap.isOpened()) {
    printf("ERROR: khong the mo camera\r\n");
    return -1;
  }
  /*---- Bat mot so khung hinh dau tien de doi camera on dinh --*/
  int count = 0;
  cv::namedWindow("Camera", cv::WINDOW_NORMAL);
  cv::resizeWindow("Camera", 300, 300);
  // Tao Trackbar de chon ham lay nguong
  cv::createTrackbar("Kieu: \n 0: Binary \n 1: Binary dao \n 2: Cat ngon \n 3: To Zero \n 4: To Zero "
                     "dao \n 5: Mau sac",
                     "Camera",           // Cua so hien thi
                     &threshold_type,    // Bien chiu tac dong cua trackbar nay
                     5,                  // Gia tri toi da cua bien (cua thanh truot)
                     Threshold_Convert); // Ham thuc thi khi keo thanh truot
  // Tao trackbar de chon gia tri nguong
  cv::createTrackbar("Gia tri nguong", "Camera", &threshold_value, max_value, Threshold_Convert);
  // Bat khung hinh dau tien cho trackbar
  Camera_capture();
  // Khoi chay ham thuc thi cua trackbar
  Threshold_Convert(0, 0);
  while(count < 500) {
    Camera_capture();
    Threshold_Convert(0, 0);
    /*-----------------------------------------------*/
    cv::imshow("Camera", frame_result);
    cv::waitKey(1);
    count++;
  }
  /*--- Chon khung hinh mau tu camera -----------------*/
  cv::namedWindow("Crop", cv::WINDOW_NORMAL);
  cv::resizeWindow("Crop", 300, 300);
  // Select ROI
  bool showCrosshair = false;
  bool fromCenter = false;
  cv::Rect2d r = cv::selectROI("Camera",     // Ten cua so
                               frame_result, // Ma tran chua hinh anh
                               fromCenter,
                               showCrosshair);
  // Crop image
  cv::Mat imageCrop = frame_result(r); // Cat hinh anh co kich co va vi tri r tu frame
  cv::imshow("Crop", imageCrop);
  /*--- Khoi tao tracker ------------------------------*/
  // KCF: Kernelize Correlation Filter, TrackerKCF
  // TLD: Track, learn and Detect, TrackerTLD
  // BOOSTING: AdaBoost algorithm, TrackerBoosting
  // Median Flow: TrackerMedianFlow
  // MIL: Multiple Instance Learning, TrackerMIL
  // GOTURN: Generic Object Tracking Using Regression Networks, TrackerGOTURN
  // MOSSE: Minimum Output Sum of Squared cv::Error, TrackerMOSSE
  // CSRT: Discriminative Correlation Filter with Channel and Spatial Reliability, TrackerCSRT
  cv::Ptr<cv::Tracker> tracker = cv::TrackerTLD::create();
  if(!tracker->init(frame_result, r)) {
    printf("ERROR: khong the khoi dong tracker\r\n");
    return -1;
  }
  if(tracker == NULL) {
    printf("ERROR: loi khoi dong tracker\r\n");
    return -1;
  }
  /*---------- Doc camera lien tuc -------------------*/
  cv::Point pt[1024]; // mang chua cac vi tri dich chuyen cua khung hinh mau
  int pt_idx = 0;     // bien kiem soat vi tri hien tai cua mang pt
  while(1) {
    time_t rawtime;      // bien lay thoi gian
    struct tm* timeinfo; // struct lay thoi gian
    char timetext[32];   // chuoi hien thi thoi gian

    time(&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(timetext, 32, "%Y/%m/%d %H:%M:%S", timeinfo);

    Camera_capture();
    Threshold_Convert(0, 0);
    /*---- Tracking ---------------------------------*/
    cv::Mat image;
    frame_result.copyTo(image);
    if(threshold_type < 5) {
      cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
    }
    // update tracker
    if(tracker->update(frame_result, r)) {
      pt[pt_idx] = cv::Point(r.x, r.y);
      pt_idx++;
      if(pt_idx >= 1024)
        pt_idx = 0;
      // Ve khung chu nhat danh dau
      cv::rectangle(image, r, cv::Scalar(255, 0, 0), 2, 1);
      // Ve danh dau cac vi tri da dich chuyen
      for(int i = 0; i < pt_idx; i++) { cv::circle(image, pt[i], 3, cv::Scalar(0, 0, 255), -1); }
      if((r.x >= 0) && (r.y >= 0) && (r.x + r.width <= image.cols) && (r.y + r.height <= image.rows)) {
        if(threshold_type < 5)
          imageCrop = frame_gray(r); // Crop tu anh xam
        else
          imageCrop = frame(r); // Crop tu anh mau
        printf("#%s# Location: %.1f, %.1f\r\n", timetext, r.x, r.y);
      } else {
        printf("#%s# Out of Frame: %.1f, %.1f\r\n", timetext, r.x, r.y);
      }
    } else {
      printf("#%s# Not detected!\r\n", timetext);
    }
    cv::imshow("Camera", image);
    cv::imshow("Crop", imageCrop);
    /*-----------------------------------------------*/
    cv::waitKey(1);
  }
  printf("Tat camera\r\n");
  cap.release();
  cv::destroyAllWindows();
  return 0;
}
