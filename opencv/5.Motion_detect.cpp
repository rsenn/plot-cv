#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

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
    cv::Mat image, image2;
    cap >> image;
    if(image.empty()) {
      printf("Hinh bi loi\r\n");
      return -1;
    }
    cap >> image2;
    if(image2.empty()) {
      printf("Hinh bi loi\r\n");
      return -1;
    }
    cv::namedWindow("Anh goc", WINDOW_NORMAL);
    cv::resizeWindow("Anh goc", 300, 300);
    cv::imshow("Anh goc", image2);
    /*--------- Chuyen ve thang xam ------------------*/
    cv::Mat image_gray, image_gray2;
    cv::cvtColor(image, image_gray, COLOR_BGR2GRAY);
    cv::cvtColor(image2, image_gray2, COLOR_BGR2GRAY);
    cv::namedWindow("Anh GRAY", WINDOW_NORMAL);
    cv::resizeWindow("Anh GRAY", 300, 300);
    cv::imshow("Anh GRAY", image_gray2);
    /*--------- Chuyen ve muc nguong ----------------*/
    cv::Mat image_result, image_result2;
    // Cac kieu chuyen doi nguong
    // 0: Binary, 1: Binary dao (inverted), 2: Cat ngon (Truncate),
    // 3: To Zero, 4: To Zero inverted
    int threshold_value = 70;
    int max_value = 255;
    int threshold_type = 0;
    cv::threshold(image_gray, image_result, threshold_value, max_value, threshold_type);
    cv::threshold(image_gray2, image_result2, threshold_value, max_value, threshold_type);
    /*--------- So sanh 2 khung hinh ----------------*/
    cv::Mat diff;
    cv::absdiff(image_result, image_result2, diff);
    cv::namedWindow("Anh Diff", WINDOW_NORMAL);
    cv::resizeWindow("Anh Diff", 300, 300);
    cv::imshow("Anh Diff", diff);
    /*--------- Loc bo hat nhieu trang --------------*/
    cv::Mat diff_erode;
    // Erode box
    cv::Mat box_erode = cv::getStructuringElement(MORPH_RECT, cv::Size(2, 2));
    cv::erode(diff, diff_erode, box_erode);
    cv::namedWindow("Anh Erode", WINDOW_NORMAL);
    cv::resizeWindow("Anh Erode", 300, 300);
    cv::imshow("Anh Erode", diff_erode);
    /*---- Tinh tri trung binh va do lech chuan -----*/
    cv::Scalar mean_diff, stddev_diff;
    cv::meanStdDev(diff, mean_diff, stddev_diff);
    cv::Scalar mean_erode, stddev_erode;
    cv::meanStdDev(diff_erode, mean_erode, stddev_erode);
    printf("[Diff] Mean = %.3f. Standard deviation = %.3f\r\n", mean_diff.val[0], stddev_diff.val[0]);
    printf("[Erode] Mean = %.3f. Standard deviation = %.3f\r\n", mean_erode.val[0], stddev_erode.val[0]);
    /*-- So sanh do lech chuan de xac dinh motion ---*/
    if(stddev_erode.val[0] > 5) {
      printf("[ALERT] MOTION DETECTED!\r\n");
    }
    /*-----------------------------------------------*/
    cv::waitKey(1);
  }
  cv::destroyAllWindows();
  return 0;
}
