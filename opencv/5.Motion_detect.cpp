#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

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
    Mat image, image2;
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
    namedWindow("Anh goc", WINDOW_NORMAL);
    resizeWindow("Anh goc", 300, 300);
    imshow("Anh goc", image2);
    /*--------- Chuyen ve thang xam ------------------*/
    Mat image_gray, image_gray2;
    cvtColor(image, image_gray, COLOR_BGR2GRAY);
    cvtColor(image2, image_gray2, COLOR_BGR2GRAY);
    namedWindow("Anh GRAY", WINDOW_NORMAL);
    resizeWindow("Anh GRAY", 300, 300);
    imshow("Anh GRAY", image_gray2);
    /*--------- Chuyen ve muc nguong ----------------*/
    Mat image_result, image_result2;
    // Cac kieu chuyen doi nguong
    // 0: Binary, 1: Binary dao (inverted), 2: Cat ngon (Truncate),
    // 3: To Zero, 4: To Zero inverted
    int threshold_value = 70;
    int max_value = 255;
    int threshold_type = 0;
    threshold(image_gray, image_result, threshold_value, max_value, threshold_type);
    threshold(image_gray2, image_result2, threshold_value, max_value, threshold_type);
    /*--------- So sanh 2 khung hinh ----------------*/
    Mat diff;
    absdiff(image_result, image_result2, diff);
    namedWindow("Anh Diff", WINDOW_NORMAL);
    resizeWindow("Anh Diff", 300, 300);
    imshow("Anh Diff", diff);
    /*--------- Loc bo hat nhieu trang --------------*/
    Mat diff_erode;
    // Erode box
    Mat box_erode = getStructuringElement(MORPH_RECT, Size(2, 2));
    erode(diff, diff_erode, box_erode);
    namedWindow("Anh Erode", WINDOW_NORMAL);
    resizeWindow("Anh Erode", 300, 300);
    imshow("Anh Erode", diff_erode);
    /*---- Tinh tri trung binh va do lech chuan -----*/
    Scalar mean_diff, stddev_diff;
    meanStdDev(diff, mean_diff, stddev_diff);
    Scalar mean_erode, stddev_erode;
    meanStdDev(diff_erode, mean_erode, stddev_erode);
    printf("[Diff] Mean = %.3f. Standard deviation = %.3f\r\n",
           mean_diff.val[0],
           stddev_diff.val[0]);
    printf("[Erode] Mean = %.3f. Standard deviation = %.3f\r\n",
           mean_erode.val[0],
           stddev_erode.val[0]);
    /*-- So sanh do lech chuan de xac dinh motion ---*/
    if(stddev_erode.val[0] > 5) {
      printf("[ALERT] MOTION DETECTED!\r\n");
    }
    /*-----------------------------------------------*/
    waitKey(1);
  }
  destroyAllWindows();
  return 0;
}
