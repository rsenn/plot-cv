#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <opencv2/objdetect/objdetect.hpp>

#include <time.h>

// using namespace cv;
using namespace std;

void
getNow(char* tt) {
  time_t rawtime;      // bien lay thoi gian
  struct tm* timeinfo; // struct lay thoi gian
  time(&rawtime);
  timeinfo = localtime(&rawtime);
  strftime(tt, 32, "%Y/%m/%d %H:%M:%S", timeinfo);
}

// Tap mau khuon mat
cv::String face_cascade_data = "/usr/local/share/OpenCV/haarcascades/haarcascade_frontalface_alt.xml";
// Tap mau mat
cv::String eyes_cascade_data = "/usr/local/share/OpenCV/haarcascades/haarcascade_eye_tree_eyeglasses.xml";

cv::CascadeClassifier face_cascade;
cv::CascadeClassifier eyes_cascade;

int
main() {
  cv::VideoCapture cap(0);
  if(!cap.isOpened()) {
    printf("ERROR: khong the mo camera\r\n");
    return -1;
  }
  // Doc du lieu tu cac tap mau
  if(!face_cascade.load(face_cascade_data)) {
    printf("ERROR: khong the mo tap khuon mat!\r\n");
    return -1;
  }
  if(!eyes_cascade.load(eyes_cascade_data)) {
    printf("ERROR: khong the mo tap mat!\r\n");
    return -1;
  }
  // Tao cua cac cua so hien thi
  cv::namedWindow("Camera", WINDOW_NORMAL);
  cv::resizeWindow("Camera", 300, 300);
  cv::namedWindow("GRAY", WINDOW_NORMAL);
  cv::resizeWindow("GRAY", 300, 300);
  cv::namedWindow("Equalizing", WINDOW_NORMAL);
  cv::resizeWindow("Equalizing", 300, 300);
  while(1) {
    char timetext[32]; // chuoi hien thi thoi gian
    std::vector<cv::Rect> faces;
    /*----- Bat hinh --------------------------------*/
    cv::Mat frame;
    cap >> frame;
    if(frame.empty()) {
      printf("ERROR: khong the bat hinh!\r\n");
      return -1;
    }
    /*------ Chuyen ve thang xam --------------------*/
    cv::Mat frame_gray;
    cv::cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
    /*------- Can bang pho mau (histogram) ----------*/
    cv::Mat frame_result;
    cv::equalizeHist(frame_gray, frame_result);
    /*----- Phat hien khuon mat ---------------------*/
    face_cascade.detectMultiScale(frame_result,                // anh xu ly
                                  faces,                       // vector ket qua
                                  1.1,                         // Scale
                                  2,                           // so diem xung quanh giu lai
                                  0 | cv::CASCADE_SCALE_IMAGE, // Co tham so thu vien HAAR
                                  cv::Size(30, 30));           // Kich thuoc doi tuong
    getNow(timetext);
    printf("[%s] Phat hien %d khuon mat\r\n", timetext, faces.size());
    for(int i = 0; i < faces.size(); i++) {
      cv::Point center(faces[i].x + faces[i].width * 0.5, faces[i].y + faces[i].height * 0.5);
      cv::ellipse(
          frame, center, cv::Size(faces[i].width * 0.5, faces[i].height * 0.5), 0, 0, 360, cv::Scalar(0, 0, 255), 4, 8, 0);
      printf("[%s] Khuon mat thu %d tai: %d, %d\r\n", timetext, i, center.x, center.y);
      // Tim mat
      cv::Mat faceROI = frame_result(faces[i]);
      std::vector<cv::Rect> eyes;
      eyes_cascade.detectMultiScale(faceROI, eyes, 1.1, 2, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30));
      printf("[%s] Khuon mat %d co %d mat\r\n", timetext, i, eyes.size());
      for(int j = 0; j < eyes.size(); j++) {
        cv::Point center_eye(faces[i].x + eyes[j].x + eyes[j].width * 0.5, faces[i].y + eyes[j].y + eyes[j].height * 0.5);
        cv::ellipse(
            frame, center_eye, cv::Size(eyes[j].width * 0.5, eyes[j].height * 0.5), 0, 0, 360, cv::Scalar(0, 0, 255), 4, 8, 0);
      }
    }
    printf("----------------------------------\r\n");
    /*------ Hien thi ket qua -----------------------*/
    cv::imshow("Camera", frame);
    cv::imshow("GRAY", frame_gray);
    cv::imshow("Equalizing", frame_result);
    /*-----------------------------------------------*/
    cv::waitKey(1);
  }
  printf("Tat camera\r\n");
  cap.release();
  cv::destroyAllWindows();
  return 0;
}
