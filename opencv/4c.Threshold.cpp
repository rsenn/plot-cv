#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
//#include <iostream>

//using namespace cv;
using namespace std;

int
main() {
  cv::VideoCapture cap(0);
  if(!cap.isOpened()) {
    printf("ERROR: khong the mo camera\r\n");
    return -1;
  }
  while(1) {
    /*-------- Mo anh goc --------------------------*/
    cv::Mat image;
    cap >> image;
    if(image.empty()) {
      printf("Hinh bi loi\r\n");
      return -1;
    }
    cv::namedWindow("Anh goc", WINDOW_NORMAL);
    cv::resizeWindow("Anh goc", 300, 300);
    cv::imshow("Anh goc", image);
    /*--------- Chuyen ve thang xam ------------------*/
    cv::Mat image_gray;
    cv::cvtColor(image, image_gray, COLOR_BGR2GRAY);
    cv::namedWindow("Anh GRAY", WINDOW_NORMAL);
    cv::resizeWindow("Anh GRAY", 300, 300);
    cv::imshow("Anh GRAY", image_gray);
    // cout << "GRAY value = "<< endl << " " << image_gray << endl << endl;
    /*--------- Chuyen ve muc nguong ----------------*/
    cv::Mat image_result;
    // Cac kieu chuyen doi nguong
    // 0: Binary, 1: Binary dao (inverted), 2: Cat ngon (Truncate),
    // 3: To Zero, 4: To Zero inverted
    int threshold_value = 70;
    int max_value = 255;
    int threshold_type = 0;
    cv::threshold(image_gray, image_result, threshold_value, max_value, threshold_type);
    cv::namedWindow("Anh Ket qua", WINDOW_NORMAL);
    cv::resizeWindow("Anh Ket qua", 300, 300);
    cv::imshow("Anh Ket qua", image_result);
    /*-----------------------------*/
    cv::waitKey(1);
  }
  cv::destroyAllWindows();
  return 0;
}
