/*Chuong trinh phat hien vat the su dung phuong phap cascade
 */
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <opencv2/objdetect/objdetect.hpp>

#include <time.h>

using namespace cv;
using namespace std;

void
getNow(char* tt) {
  time_t rawtime;      // bien lay thoi gian
  struct tm* timeinfo; // struct lay thoi gian
  time(&rawtime);
  timeinfo = localtime(&rawtime);
  strftime(tt, 32, "%Y/%m/%d %H:%M:%S", timeinfo);
}

// Tap vat mau
String object_cascade_data = "./K/data/xml/cascade.xml";

CascadeClassifier object_cascade;

int
main() {
  VideoCapture cap(0);
  if(!cap.isOpened()) {
    printf("ERROR: khong the mo camera\r\n");
    return -1;
  }
  // Doc du lieu tu cac tap mau
  if(!object_cascade.load(object_cascade_data)) {
    printf("ERROR: khong the mo tap vat mau!\r\n");
    return -1;
  }
  // Tao cua cac cua so hien thi
  namedWindow("Camera", WINDOW_NORMAL);
  resizeWindow("Camera", 300, 300);
  namedWindow("GRAY", WINDOW_NORMAL);
  resizeWindow("GRAY", 300, 300);
  namedWindow("Equalizing", WINDOW_NORMAL);
  resizeWindow("Equalizing", 300, 300);
  while(1) {
    char timetext[32]; // chuoi hien thi thoi gian
    std::vector<Rect> objects;
    /*----- Bat hinh --------------------------------*/
    Mat frame;
    cap >> frame;
    if(frame.empty()) {
      printf("ERROR: khong the bat hinh!\r\n");
      return -1;
    }
    /*------ Chuyen ve thang xam --------------------*/
    Mat frame_gray;
    cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
    /*------- Can bang pho mau (histogram) ----------*/
    Mat frame_result;
    equalizeHist(frame_gray, frame_result);
    /*----- Phat hien doi tuong ---------------------*/
    object_cascade.detectMultiScale(frame_result,  // anh xu ly
                                    objects,       // vector ket qua
                                    1.1,           // Scale
                                    2,             // so diem xung quanh giu lai
                                    0,             // phuong phap LBP
                                    Size(24, 24)); // Kich thuoc doi tuong
    getNow(timetext);
    printf("[%s] Phat hien %d vat mau\r\n", timetext, objects.size());
    /*------ Danh dau cac doi tuong tren camera -----*/
    for(int i = 0; i < objects.size(); i++) {
      Point center(objects[i].x + objects[i].width * 0.5, objects[i].y + objects[i].height * 0.5);
      ellipse(frame,                                                 // Anh duoc ve
              center,                                                // Toa do tam
              Size(objects[i].width * 0.5, objects[i].height * 0.5), // Kich thuoc
              0,                                                     // Goc xoay
              0,                                                     // Goc bat dau
              360,                                                   // Goc ket thuc
              Scalar(0, 0, 255),                                     // Mau sac BGR
              4,                                                     // Do day vien
              8                                                      // Kieu duong net
      );
      printf("[%s] Vat mau thu %d tai: %d, %d\r\n", timetext, i, center.x, center.y);
    }
    printf("-------------------------------------\r\n");
    /*------- Hien thi ket qua -----------------------*/
    imshow("Camera", frame);
    imshow("GRAY", frame_gray);
    imshow("Equalizing", frame_result);
    /*------------------------------------------------*/
    waitKey(1);
  }
  printf("Tat camera\r\n");
  cap.release();
  destroyAllWindows();
  return 0;
}
