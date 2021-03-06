#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace std;

Mat frame;
Point selected_point;       // Vi tri vua duoc nhan
int selected_npts = 0;      // Bien dem so luong vi tri da chon
vector<Point> selected_pts; // Danh sach cac vi tri
Rect2d r;
void mouseHandler(int, int, int, int, void*);

int
main() {
  VideoCapture cap(0);
  if(!cap.isOpened()) {
    printf("ERROR: khong the mo camera\r\n");
    return -1;
  }
  namedWindow("Camera", WINDOW_NORMAL);
  resizeWindow("Camera", 300, 300);
  setMouseCallback("Camera", mouseHandler, NULL);
  /*------- Doc lien tuc ------------------------------*/
  while(1) {
    cap >> frame;
    if(frame.empty()) {
      printf("ERROR: khong the bat hinh\r\n");
      break;
    }
    rectangle(frame,                 // Anh duoc ve
              r,                     // Diem ket thuc
              Scalar(255, 255, 255), // Mau sac
              2,                     // Do day net
              LINE_8);
    imshow("Camera", frame);
    /*-----------------------------------------------*/
    waitKey(1);
  }
  printf("Tat camera\r\n");
  cap.release();
  destroyAllWindows();
  return 0;
}

/*---- Ham xu ly su kien chuot --------------------------*/
void
mouseHandler(int event, int x, int y, int, void*) {
  /*--------- Chuot trai duoc bam ------------------*/
  if(event == EVENT_LBUTTONDOWN) {
    selected_point = Point(x, y); // Lay toa do diem duoc chon
    printf("[MOUSE] Left-Down: x=%d,y=%d,n=%d\r\n", x, y, selected_npts);
    // Ve diem tron danh dau
    circle(frame,                 // Anh duoc ve
           selected_point,        // Vi tri
           2,                     // Ban kinh
           Scalar(255, 255, 255), // Mau sac
           -1,                    // Do day net, -1 co nghia phu dac
           LINE_8                 // Kieu net
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
  if(event == EVENT_LBUTTONUP) {
    printf("[MOUSE] Left-Up: x=%d,y=%d,n=%d\r\n", x, y, selected_npts);
    imshow("Camera", frame);
  }
  /*---------- Chuot phai duoc bam -----------------*/
  if(event == EVENT_RBUTTONDOWN) {
    printf("[MOUSE] Right-Down: x=%d,y=%d\r\n", x, y);
  }
  /*------- Chuot phai duoc tha --------------------*/
  if(event == EVENT_RBUTTONUP) {
    printf("[MOUSE] Right-Up: x=%d,y=%d\r\n", x, y);
  }
  /*------- Chuot giua duoc bam -------------------*/
  if(event == EVENT_MBUTTONDOWN) {
    printf("[MOUSE] Middle-Down: x=%d,y=%d\r\n", x, y);
  }
  /*------- Chuot giua duoc tha -------------------*/
  if(event == EVENT_MBUTTONUP) {
    printf("[MOUSE] Middle-Up: x=%d,y=%d\r\n", x, y);
  }
  /*------- Chuot dang di chuyen ----------------*/
  if(event == EVENT_MOUSEMOVE) {
    printf("[MOUSE] Moved to: x=%d,y=%d\r\n", x, y);
    selected_point = Point(x, y); // Lay toa do diem duoc chon
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
