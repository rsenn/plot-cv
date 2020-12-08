#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
//#include <iostream>

using namespace cv;
using namespace std;

int
main() {
  VideoCapture cap(0);
  if(!cap.isOpened()) {
    printf("ERROR: khong the mo camera\r\n");
    return -1;
  }
  while(1) {
    /*-------- Mo anh goc --------------------------*/
    Mat image;
    cap >> image;
    if(image.empty()) {
      printf("Hinh bi loi\r\n");
      return -1;
    }
    namedWindow("Anh goc", WINDOW_NORMAL);
    resizeWindow("Anh goc", 300, 300);
    imshow("Anh goc", image);
    /*--------- Chuyen ve thang xam ------------------*/
    Mat image_gray;
    cvtColor(image, image_gray, COLOR_BGR2GRAY);
    namedWindow("Anh GRAY", WINDOW_NORMAL);
    resizeWindow("Anh GRAY", 300, 300);
    imshow("Anh GRAY", image_gray);
    // cout << "GRAY value = "<< endl << " " << image_gray << endl << endl;
    /*--------- Chuyen ve muc nguong ----------------*/
    Mat image_result;
    // Cac kieu chuyen doi nguong
    // 0: Binary, 1: Binary dao (inverted), 2: Cat ngon (Truncate),
    // 3: To Zero, 4: To Zero inverted
    int threshold_value = 70;
    int max_value = 255;
    int threshold_type = 0;
    threshold(image_gray, image_result, threshold_value, max_value, threshold_type);
    namedWindow("Anh Ket qua", WINDOW_NORMAL);
    resizeWindow("Anh Ket qua", 300, 300);
    imshow("Anh Ket qua", image_result);
    /*-----------------------------*/
    waitKey(1);
  }
  destroyAllWindows();
  return 0;
}
