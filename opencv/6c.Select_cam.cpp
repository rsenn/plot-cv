#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

//using namespace cv;
using namespace std;

cv::Mat frame;
cv::Point selected_point;       // Vi tri vua duoc nhan
int selected_npts = 0;      // Bien dem so luong vi tri da chon
vector<cv::Point> selected_pts; // Danh sach cac vi tri
cv::Rect2d r;
void mouseHandler(int, int, int, int, void*);

int
main() {
  cv::VideoCapture cap(0);
  if(!cap.isOpened()) {
    printf("ERROR: khong the mo camera\r\n");
    return -1;
  }
  cv::namedWindow("Camera", cv::WINDOW_NORMAL);
  cv::resizeWindow("Camera", 300, 300);
  cv::setMouseCallback("Camera", mouseHandler, NULL);
  /*------- Doc lien tuc ------------------------------*/
  while(1) {
    cap >> frame;
    if(frame.empty()) {
      printf("ERROR: khong the bat hinh\r\n");
      break;
    }
    cv::rectangle(frame,                 // Anh duoc ve
              r,                     // Diem ket thuc
              cv::Scalar(255, 255, 255), // Mau sac
              2,                     // Do day net
              cv::LINE_8);
    cv::imshow("Camera", frame);
    /*-----------------------------------------------*/
    cv::waitKey(1);
  }
  printf("Tat camera\r\n");
  cap.release();
  cv::destroyAllWindows();
  return 0;
}

/*---- Ham xu ly su kien chuot --------------------------*/
void
mouseHandler(int event, int x, int y, int, void*) {
  /*--------- Chuot trai duoc bam ------------------*/
  if(event == cv::EVENT_LBUTTONDOWN) {
    selected_point = cv::Point(x, y); // Lay toa do diem duoc chon
    printf("[MOUSE] Left-Down: x=%d,y=%d,n=%d\r\n", x, y, selected_npts);
    // Ve diem tron danh dau
    cv::circle(frame,                 // Anh duoc ve
           selected_point,        // Vi tri
           2,                     // Ban kinh
           cv::Scalar(255, 255, 255), // Mau sac
           -1,                    // Do day net, -1 co nghia phu dac
           cv::LINE_8                 // Kieu net
    );
    // Luu vi tri vao mang cac vi tri
    selected_pts.push_back(selected_point);
    if(selected_npts > 0) {
      int w = abs(selected_point.x - selected_pts[selected_npts - 1].x);
      int h = abs(selected_point.y - selected_pts[selected_npts - 1].y);
      if(selected_point.x > selected_pts[selected_npts - 1].x) {
        if(selected_point.y > selected_pts[selected_npts - 1].y) {
          r.x = selected_pts[selected_npts - 1].x;
          r.y = selected_pts[selected_npts - 1].y;
        } else {
          r.x = selected_pts[selected_npts - 1].x;
          r.y = selected_point.y;
        }
      } else {
        if(selected_point.y < selected_pts[selected_npts - 1].y) {
          r.x = selected_point.x;
          r.y = selected_point.y;
        } else {
          r.x = selected_point.x;
          r.y = selected_pts[selected_npts - 1].y;
        }
      }
      r.width = w;
      r.height = h;
      selected_npts = 0;
      selected_pts.clear();
    } else {
      // Tang bien dem so cac vi tri
      selected_npts++;
    }
  }
  /*---------- Chuot trai duoc tha -----------------*/
  if(event == cv::EVENT_LBUTTONUP) {
    printf("[MOUSE] Left-Up: x=%d,y=%d,n=%d\r\n", x, y, selected_npts);
    cv::imshow("Camera", frame);
  }
  /*---------- Chuot phai duoc bam -----------------*/
  if(event == cv::EVENT_RBUTTONDOWN) {
    printf("[MOUSE] Right-Down: x=%d,y=%d\r\n", x, y);
  }
  /*------- Chuot phai duoc tha --------------------*/
  if(event == cv::EVENT_RBUTTONUP) {
    printf("[MOUSE] Right-Up: x=%d,y=%d\r\n", x, y);
  }
  /*------- Chuot giua duoc bam -------------------*/
  if(event == cv::EVENT_MBUTTONDOWN) {
    printf("[MOUSE] Middle-Down: x=%d,y=%d\r\n", x, y);
  }
  /*------- Chuot giua duoc tha -------------------*/
  if(event == cv::EVENT_MBUTTONUP) {
    printf("[MOUSE] Middle-Up: x=%d,y=%d\r\n", x, y);
  }
  /*------- Chuot dang di chuyen ----------------*/
  if(event == cv::EVENT_MOUSEMOVE) {
    printf("[MOUSE] Moved to: x=%d,y=%d\r\n", x, y);
    selected_point = cv::Point(x, y); // Lay toa do diem duoc chon
    if(selected_npts > 0) {
      int w = abs(selected_point.x - selected_pts[selected_npts - 1].x);
      int h = abs(selected_point.y - selected_pts[selected_npts - 1].y);
      if(selected_point.x > selected_pts[selected_npts - 1].x) {
        if(selected_point.y > selected_pts[selected_npts - 1].y) {
          r.x = selected_pts[selected_npts - 1].x;
          r.y = selected_pts[selected_npts - 1].y;
        } else {
          r.x = selected_pts[selected_npts - 1].x;
          r.y = selected_point.y;
        }
      } else {
        if(selected_point.y < selected_pts[selected_npts - 1].y) {
          r.x = selected_point.x;
          r.y = selected_point.y;
        } else {
          r.x = selected_point.x;
          r.y = selected_pts[selected_npts - 1].y;
        }
      }
      r.width = w;
      r.height = h;
    }
  }
}
