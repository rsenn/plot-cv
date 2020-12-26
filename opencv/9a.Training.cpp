/*Chuong trinh Tao tap mau su dung phuong phap cascade

find negative/rawdata/ -name '*.jpg' > negative/bg.txt
mkdir xml
opencv_createsamples -info positive/info.txt -vec vector.vec -num 200 -w 24 -h 24
opencv_createsamples -vec vector.vec
opencv_traincascade -data xml/ -vec vector.vec -bg negative/bg.txt -numPos 200 -numNeg 200
-numStages 20 -acceptanceRatioBreakValue 0.004 -precalcValBufSize 300 -precalcIdxBufSize 300 -mode
ALL -w 24 -h 24

Tri so acceptanceRatioBreakValue the hien so sai so chap nhan, phu thuoc vao so luong mau. VD: 200
mau, chap nhan sai 1 mau -> ty le 0.005
*/
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <stdlib.h>

using namespace cv;
using namespace std;

int
main() {
  VideoCapture cap(0);
  // Thiet lap do phan giai o 1.3MP
  cap.set(CAP_PROP_FRAME_WIDTH, 1280);
  cap.set(CAP_PROP_FRAME_HEIGHT, 1024);
  if(!cap.isOpened()) {
    printf("ERROR: khong the mo camera 0\r\n");
    return 0;
  }
  int count_neg = 0;
  int count_pos = 0;
  system("tar -zcf data_old.zip ./data/"); // Backup du lieu cu neu co
  system("rm -rf ./data/");                // Xoa thu muc du lieu cu
  system("mkdir ./data/");                 // Tao thu muc du lieu moi
  system("mkdir ./data/neg/");             // Tao thu muc chua anh negative trong thu muc data
  system("mkdir ./data/pos/");             // Tao thu muc chua anh positive trong thu muc data
  system("touch ./data/info.txt");         // Tao file chua thong tin anh positive
  printf("\r\n===============================\r\n");
  printf("Chuong trinh huan luyen RPI\r\n\r\nHuong dan su dung:\r\n");
  printf("- Nhan 'n' de luu anh negative.\r\n");
  printf("- Nhan 'p' de luu anh potive.\r\n");
  printf("- Nhan 'c' de ket thuc.\r\n\r\n");
  //------------------------------------------
  namedWindow("Camera", WINDOW_NORMAL);
  resizeWindow("Camera", 400, 400);
  //------------------------------------------
  Mat frame;
  Mat gray_img;
  Mat imageCrop;
  char path[32];      // Mang luu duong dan file
  char lineText[256]; // Mang chua thong tin anh positive de luu vao info.txt
  float w = 100000, h = 100000;
  float crpX, crpY, crpW, crpH;
  while(1) {
    cap >> frame;       // capture frame hien tai tu camera
    if(frame.empty()) { // kiem tra frame co du lieu hay ko
      printf("ERROR: khong the bat hinh\r\n");
      break;
    }
    imshow("Camera", frame);
    char c = waitKey(1); // doc nut nhan tu ban phim
    if(c == 'c')
      break;
    else {
      switch(c) {
        case 'n':
          snprintf(path, 32, "./data/neg/neg_%d.jpg", count_neg);
          cvtColor(frame, gray_img, COLOR_BGR2GRAY);
          equalizeHist(gray_img, gray_img);
          imwrite(path, gray_img);
          count_neg++;
          printf("[NEG] Da luu %d hinh negative vao thu muc ./data/neg/\r\n", count_neg);
          break;
        case 'p':
          namedWindow("Select", WINDOW_NORMAL);
          resizeWindow("Select", 800, 600);
          // Select ROI
          bool showCrosshair = false;
          bool fromCenter = false;
          Rect2d r = selectROI("Select", frame, fromCenter, showCrosshair);
          destroyWindow("Select"); // Dong cua so sau khi da chon
          printf("[POS] Vat mau tai x=%.0f, y=%.0f, width=%.0f, heigth=%.0f\r\n", r.x, r.y, r.width, r.height);
          if((w > r.width) && (h > r.height)) { // Get smallest
            w = r.width;
            h = r.height;
          }
          snprintf(path, 32, "./data/pos/pos_%d.jpg", count_pos);
          if((r.x - 20 >= 0) && (r.y - 20 >= 0) && (r.x + 20 <= frame.cols) && (r.y + 20 <= frame.rows)) {
            Rect2d cr(r.x - 20, r.y - 20, r.width + 40, r.height + 40);
            imageCrop = frame(cr);
            snprintf(lineText,
                     256,
                     "echo \"pos/pos_%d.jpg 1 20 20 %.0f %.0f\" >> ./data/info.txt",
                     count_pos,
                     r.width,
                     r.height);
          } else {
            imageCrop = frame(r);
            snprintf(lineText,
                     256,
                     "echo \"pos/pos_%d.jpg 1 0 0 %.0f %.0f\" >> ./data/info.txt",
                     count_pos,
                     r.width,
                     r.height);
          }
          imwrite(path, imageCrop);
          system(lineText);
          count_pos++;
          printf("[POS] Da luu %d hinh positive vao thu muc ./data/pos/\r\n", count_pos);
          break;
      }
    }
  }
  printf("Tat camera\r\n");
  cap.release(); // Giai phong camera
  destroyAllWindows();
  printf("[NEG] Dang tao file danh sach...\r\n");
  system("find ./data/neg/ -name '*.jpg' > ./data/bg.txt");
  printf("[NEG] File danh sach hinh negative tai duong dan ./data/bg.txt\r\n");
  system("mkdir ./data/xml"); // Tao thu muc luu ket qua
  system("tar -zcf data_new.zip ./data/");
  printf("\r\n[TRAIN] Bat dau train....\r\n");
  // w = w/10;
  // h = h/10;
  w = 24;
  h = 24;
  printf("[TRAIN] Tao vector tap mau bang opencv_createsamples....\r\n");
  snprintf(lineText,
           256,
           "opencv_createsamples -info ./data/info.txt -vec ./data/vector.vec -num %d -w %.0f -h %.0f",
           count_pos,
           w,
           h);
  printf("%s\r\n", lineText);
  system(lineText);
  float acceptRate = 1.0 / count_neg;
  acceptRate = acceptRate * 0.5;
  printf("[TRAIN] Train bang opencv_traincascade....\r\n");
  snprintf(lineText,
           256,
           "opencv_traincascade -data ./data/xml/ -vec ./data/vector.vec -bg ./data/bg.txt -numPos "
           "%d -numNeg %d "
           "-numStages 20 -acceptanceRatioBreakValue %.5f -precalcValBufSize 300 "
           "-precalcIdxBufSize 300 -featureType "
           "LBP -minHitRate 0.999 -maxFalseAlarmRate 0.5 -w %.0f -h %.0f",
           count_pos,
           count_neg,
           acceptRate,
           w,
           h);
  printf("%s\r\n", lineText);
  system(lineText);
  printf("\r\n---------Done-----------\r\n");
  return 0;
}
