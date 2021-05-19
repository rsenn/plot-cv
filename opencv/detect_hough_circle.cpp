#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <dirent.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sys/stat.h>
#include <time.h>
#include <fstream>
// #include <mysql.h>

// using namespace cv;
using namespace std;

const int ITERATOR = 1;
// FILES
const string EXTS[] = {"jpg", "png", "jpeg"};
const int EXT_SIZE = sizeof(EXTS) / sizeof(string);
const char* TXT_NAME = "coordonnees.txt";
// CIRCLES
const int MAX_RADIUS = 30;
const int MIN_RADIUS = 10;
// BDD
const char* HOSTNAME = "localhost";
const char* USER = "root";
const char* PASSWORD = "";
const char* TABLE = "hough_circles";

ofstream text;

cv::Mat
applyFilters(cv::Mat& img) {
  cv::Mat var_img;
  cv::cvtColor(img, var_img, cv::COLOR_BGR2GRAY);
  cv::Canny(var_img, var_img, 50, 150, 3);
  cv::GaussianBlur(var_img, var_img, cv::Size(9, 9), 2, 2);

  return var_img;
}

vector<cv::Vec3f>
getCircles(cv::Mat& img) {
  vector<cv::Vec3f> circles;
  cv::HoughCircles(img, circles, cv::HOUGH_GRADIENT, 1, img.rows / 8, 2, 32.0, 10, 30);
  return circles;
}

cv::Point
getCenterCoordinates(vector<cv::Vec3f> circles) {
  for(size_t i = 0; i < circles.size(); i++) {
    int radius = cvRound(circles[i][2]);
    if(radius >= MIN_RADIUS && radius <= MAX_RADIUS)
      return cv::Point(cvRound(circles[i][0]), cvRound(circles[i][1]));
  }
  return cv::Point(0, 0);
}

cv::Point
rightCircle(cv::Mat& img) {
  int width = img.cols;
  cv::Rect roi(width - 150, 0, 150, 150);
  cv::Mat sub_img = img(roi);
  cv::Mat gray = applyFilters(sub_img);
  vector<cv::Vec3f> circles = getCircles(gray);
  if(circles.size() <= 0)
    return cv::Point(0, 0);
  cv::Point center = getCenterCoordinates(circles);
  center.x += width - 150;
  return cv::Point(center.x, center.y);
}

cv::Point
leftCircle(cv::Mat& img) {
  cv::Rect roi(0, 0, 150, 150);

  cv::Mat sub_img = img(roi);
  cv::Mat gray = applyFilters(sub_img);
  vector<cv::Vec3f> circles = getCircles(gray);
  if(circles.size() <= 0)
    return cv::Point(0, 0);
  cv::Point center = getCenterCoordinates(circles);
  return cv::Point(center.x, center.y);
}

bool
isGraphicFile(string filename) {
  for(int i = 0; i < EXT_SIZE; i++) {
    size_t found = filename.find(EXTS[i]);
    if(found != string::npos)
      return 1;
  }
  return 0;
}

bool
isDir(const char* path) {
  struct stat buf;
  stat(path, &buf);
  return S_ISDIR(buf.st_mode);
}

string
fullPath(string dir, string filename) {
  return dir + filename;
}

bool
pointIsNull(cv::Point p) {
  return ((p.x == 0) && (p.y == 0));
}

void
detectCirclesFromImage(cv::Mat& img) {
  cv::Point left = leftCircle(img);
  cv::Point right = rightCircle(img);

  if(pointIsNull(left) || pointIsNull(right))
    text << "\n";
  else
    text << "|" << left.x << "|" << left.y << "|" << right.x << "|" << right.y << "\n";
}

// void finishWithError(MYSQL *con)
// {
//   fprintf(stderr, "%s\n", mysql_error(con));
//   mysql_close(con);
//   exit(1);
// }

int
main(int argc, char** argv) {
  clock_t tic = clock();

  // MYSQL *con = mysql_init(NULL);

  // if (con == NULL)
  // {
  //     fprintf(stderr, "%s\n", mysql_error(con));
  //     exit(1);
  // }

  // if ( mysql_real_connect( con, HOSTNAME, USER, PASSWORD, TABLE, 0, NULL, 0 ) == NULL )
  //   finishWithError(con);

  // if ( mysql_query(con, "DROP TABLE IF EXISTS Circles") )
  //   finishWithError(con);

  // if ( mysql_query(con, "CREATE TABLE Circles(id INT, filename CHAR, left_x INT, left_y INT, right_x INT,
  // right_y INT)") )
  //     finishWithError(con);

  // char buffer [50];
  // n = sprintf (buffer, "SELECT * FROM %s WHERE filename like %%%s%%", TABLE, filename);
  // printf ("[%s] is a string %d chars long\n",buffer,n);

  // mysql_close(con);

  DIR* dir;
  struct dirent* file;
  text.open(TXT_NAME);

  if(isDir(argv[1])) {
    dir = opendir(argv[1]);

    if(dir == NULL)
      return 0;
    string folder = argv[1];

    while((file = readdir(dir))) {
      for(int j = 0; j < ITERATOR; j++) {
        string filename = file->d_name;
        if(!isGraphicFile(filename))
          continue;
        string img_full_path = fullPath(folder, filename);
        cv::Mat img = cv::imread(img_full_path, 1);
        text << filename;
        detectCirclesFromImage(img);
      }
    }
  } else {
    if(isGraphicFile(argv[1])) {
      cv::Mat img = cv::imread(argv[1], 1);
      text << argv[1];
      detectCirclesFromImage(img);
    }
  }

  clock_t toc = clock();
  printf("Elapsed: %f seconds\n", (double)(toc - tic) / CLOCKS_PER_SEC);
  return 0;
}
