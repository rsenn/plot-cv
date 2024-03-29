#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <iomanip>
#include <math.h>

#define PI 3.14159265

// using namespace cv;
using namespace std;

///////////////////////////////////// VARIABEL ////////////////////////////////////////

char charCheckForEscKey = 0;

cv::Mat matOriginal;
cv::Mat matProcessed;

int morph_pixel = 5;

int Y1 = 7;
int Cr1 = 0;
int Cb1 = 0;
int Y2 = 170;
int Cr2 = 144;
int Cb2 = 131;

vector<Vec3f> v3fCircles;

int acc_res = 2;
int min_dis = 500;
int high_thres = 50;
int low_thres = 35;
int min_rad = 10;
int max_rad = 110;

string posisi;
float Sudut;
string S_Sudut;
float Jarak;
string S_Jarak;
float Diameter;
int x_center;
int y_center;
float R;
float d_dekat;
float d_jauh;
float x_Sudut = 160;
float y_Sudut = 490; // jika sudut pandang kamera 49,2 derajat nilai y_sudut : 490 || jika sudut
                     // pandang kamera 47,2
// derajat nilai y_sudut : 450

// y = ar + b
float a_dekat1 = 0.0528045;
float b_dekat1 = 90.084;
float a_jauh1 = 0.046262;
float b_jauh1 = 47.6737;
float j_ref_dekat1 = 50;
float j_ref_jauh1 = 100;

int batas1 = 77; // jika sudut pandang kamera 49,2 derajat nilai batas1 : 77 || jika sudut pandang
                 // kamera 47,2 derajat
// nilai batas1 : 58

float a_dekat2 = 0.046262;
float b_dekat2 = 47.6737;
float a_jauh2 = 0.0500693;
float b_jauh2 = 32.951;
float j_ref_dekat2 = 100;
float j_ref_jauh2 = 150;

int batas2 = 45; // jika sudut pandang kamera 49,2 derajat nilai batas2 : 45 || jika sudut pandang
                 // kamera 47,2 derajat
// nilai batas2 : 27

float a_dekat3 = 0.0500693;
float b_dekat3 = 32.951;
float a_jauh3 = 0.0674082;
float b_jauh3 = 22.4401;
float j_ref_dekat3 = 150;
float j_ref_jauh3 = 200;

int batas3 = 29; // jika sudut pandang kamera 49,2 derajat nilai batas3 : 29 || jika sudut pandang
                 // kamera 47,2 derajat
// nilai batas3 : 11

float a_dekat4 = 0.0674082;
float b_dekat4 = 22.4401;
float a_jauh4 = 0.0389122;
float b_jauh4 = 20.1244;
float j_ref_dekat4 = 200;
float j_ref_jauh4 = 250;

int batas4 = 17; // jika sudut pandang kamera 49,2 derajat nilai batas4 : 17 || jika sudut pandang
                 // kamera 47,2 derajat
// nilai batas4 : 0

float a_dekat5 = 0.0389122;
float b_dekat5 = 20.1244;
float a_jauh5 = 0.0232639;
float b_jauh5 = 18.7136;
float j_ref_dekat5 = 250;
float j_ref_jauh5 = 300;

///////////////////////////////////// FUNGSI ////////////////////////////////////////

class perhitungan {

public:
  float
  jarakKeTengah(float x, float y) {
    float R = sqrt((cv::pow((x - 160), 2)) + (pow((y - 120), 2)));
    return R;
  }

  float
  diaDekat(float aDekat, float bDekat, float r) {
    float dDekat = ((aDekat * r) + bDekat);
    return dDekat;
  }

  float
  diaJauh(float aJauh, float bJauh, float r) {
    float dJauh = ((aJauh * r) + bJauh);
    return dJauh;
  }

  float
  jarak(float Dia, float d_dekat, float d_jauh, float jr_dekat, float jr_jauh) {
    float Jar = (((jr_jauh - jr_dekat) * (Dia - d_dekat)) / (d_jauh - d_dekat)) + jr_dekat;
    return Jar;
  };

  float
  sudut(float x_center, float y_center) {
    float Sud = float(atan((x_Sudut - x_center) / (y_Sudut - y_center)) * 180 / PI);
    return Sud;
  }
};

///////////////////////////////////// MAIN ////////////////////////////////////////

int
main() {

  perhitungan hitung;
  cv::VideoCapture cap(0);
  cap.set(cv::CAP_PROP_FRAME_WIDTH, 320);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, 240);
  cap.set(cv::CAP_PROP_FPS, 1);
  if(cap.isOpened() == false) {
    std::cout << "cv::error: Webcam not accessed successfully\n\n";
    return (0);
  }

  cv::namedWindow("MORPH", cv::WINDOW_NORMAL);
  cv::createTrackbar("morph_pixel", "MORPH", &morph_pixel, 20);

  cv::namedWindow("ColorField", cv::WINDOW_NORMAL);
  cv::createTrackbar("Y1", "ColorField", &Y1, 255);
  cv::createTrackbar("Cr1", "ColorField", &Cr1, 255);
  cv::createTrackbar("Cb1", "ColorField", &Cb1, 255);
  cv::createTrackbar("Y2", "ColorField", &Y2, 255);
  cv::createTrackbar("Cr2", "ColorField", &Cr2, 255);
  cv::createTrackbar("Cb2", "ColorField", &Cb2, 255);

  cv::namedWindow("Circle", cv::WINDOW_NORMAL);
  cv::createTrackbar("acc_res", "Circle", &acc_res, 255);
  cv::createTrackbar("min_dis", "Circle", &min_dis, 255);
  cv::createTrackbar("high_thres", "Circle", &high_thres, 255);
  cv::createTrackbar("low_thres", "Circle", &low_thres, 255);
  cv::createTrackbar("min_rad", "Circle", &min_rad, 255);
  cv::createTrackbar("max_rad", "Circle", &max_rad, 1000);

  while(charCheckForEscKey != 27 && cap.isOpened()) {
    bool blnFrameReadSuccessfully = cap.read(matOriginal);

    if(!blnFrameReadSuccessfully || matOriginal.empty()) {
      std::cout << "cv::error: frame not cv::read from webcam\n";
      break;
    }

    cv::cvtColor(matOriginal, matProcessed, CV_BGR2YCrCb);

    cv::inRange(matProcessed, cv::Scalar(Y1, Cr1, Cb1), cv::Scalar(Y2, Cr2, Cb2), matProcessed);
    cv::bitwise_not(matProcessed, matProcessed);

    cv::erode(matProcessed, matProcessed, cv::getStructuringElement(MORPH_ELLIPSE, cv::Size(morph_pixel, morph_pixel)));

    cv::dilate(matProcessed, matProcessed, cv::getStructuringElement(MORPH_ELLIPSE, cv::Size(morph_pixel, morph_pixel)));

    cv::HoughCircles(matProcessed, v3fCircles, cv::HOUGH_GRADIENT, acc_res, min_dis, high_thres, low_thres, min_rad, max_rad);

    for(int i = 0; i < v3fCircles.size(); i++) {

      Diameter = (v3fCircles[i][2]) * 2;
      x_center = (int)v3fCircles[i][0]; // koordinat x bola
      y_center = (int)v3fCircles[i][1]; // koordinat y bola

      cv::circle(matOriginal, cv::Point(x_center, y_center), 1, cv::Scalar(255, 0, 0), cv::FILLED);
      cv::circle(matOriginal, cv::Point(x_center, y_center), (int)v3fCircles[i][2], cv::Scalar(0, 0, 255), 2);
      cv::line(matOriginal, cv::Point(160, 240), cv::Point(x_center, y_center), cv::Scalar(0, 255, 0), 2, cv::LINE_AA);

      if(y_center >= batas1) {
        R = hitung.jarakKeTengah(x_center, y_center);
        d_dekat = hitung.diaDekat(a_dekat1, b_dekat1, R);
        d_jauh = hitung.diaJauh(a_jauh1, b_jauh1, R);
        Jarak = hitung.jarak(Diameter, d_dekat, d_jauh, j_ref_dekat1, j_ref_jauh1);
      } else if(y_center < batas1 && y_center >= batas2) {
        R = hitung.jarakKeTengah(x_center, y_center);
        d_dekat = hitung.diaDekat(a_dekat2, b_dekat2, R);
        d_jauh = hitung.diaJauh(a_jauh2, b_jauh2, R);
        Jarak = hitung.jarak(Diameter, d_dekat, d_jauh, j_ref_dekat2, j_ref_jauh2);
      } else if(y_center < batas2 && y_center >= batas3) {
        R = hitung.jarakKeTengah(x_center, y_center);
        d_dekat = hitung.diaDekat(a_dekat3, b_dekat3, R);
        d_jauh = hitung.diaJauh(a_jauh3, b_jauh3, R);
        Jarak = hitung.jarak(Diameter, d_dekat, d_jauh, j_ref_dekat3, j_ref_jauh3);
      } else if(y_center < batas3 && y_center >= batas4) {
        R = hitung.jarakKeTengah(x_center, y_center);
        d_dekat = hitung.diaDekat(a_dekat4, b_dekat4, R);
        d_jauh = hitung.diaJauh(a_jauh4, b_jauh4, R);
        Jarak = hitung.jarak(Diameter, d_dekat, d_jauh, j_ref_dekat4, j_ref_jauh4);
      } else {
        R = hitung.jarakKeTengah(x_center, y_center);
        d_dekat = hitung.diaDekat(a_dekat5, b_dekat5, R);
        d_jauh = hitung.diaJauh(a_jauh5, b_jauh5, R);
        Jarak = hitung.jarak(Diameter, d_dekat, d_jauh, j_ref_dekat5, j_ref_jauh5);
      }

      if(x_center > 160) {
        posisi = "kanan";
      } else {
        posisi = "kiri";
      }

      Sudut = hitung.sudut(x_center, y_center);
      if(Sudut < 0) {
        Sudut = -Sudut;
      } else {
        Sudut = Sudut;
      }

      stringstream j;
      j << fixed << setprecision(2) << Jarak;
      S_Jarak = j.str();
      cv::putText(matOriginal,
                  S_Jarak,
                  cv::Point(x_center, (y_center + 10)),
                  FONT_HERSHEY_SIMPLEX,
                  .7,
                  cv::Scalar(255, 0, 0),
                  2,
                  8,
                  false);
      cv::putText(matOriginal,
                  "cm",
                  cv::Point((x_center + 80), (y_center + 10)),
                  FONT_HERSHEY_PLAIN,
                  .7,
                  cv::Scalar(255, 0, 0),
                  1,
                  8,
                  false);

      stringstream x;
      x << x_center;
      string S_x_center = x.str();
      cv::putText(matOriginal, "X ", cv::Point(185, 220), FONT_HERSHEY_PLAIN, .7, cv::Scalar(0, 0, 0), 1, 4, false);
      cv::putText(matOriginal, S_x_center, cv::Point(195, 220), FONT_HERSHEY_PLAIN, .7, cv::Scalar(255, 255, 255), 1, 4, false);

      stringstream y;
      y << y_center;
      string S_y_center = y.str();
      cv::putText(matOriginal, "Y ", cv::Point(185, 230), FONT_HERSHEY_PLAIN, .7, cv::Scalar(0, 0, 0), 1, 4, false);
      cv::putText(matOriginal, S_y_center, cv::Point(195, 230), FONT_HERSHEY_PLAIN, .7, cv::Scalar(255, 255, 255), 1, 4, false);

      stringstream d;
      d << Diameter;
      string S_Diameter = d.str();
      cv::putText(matOriginal, "D ", cv::Point(230, 230), FONT_HERSHEY_PLAIN, .7, cv::Scalar(0, 0, 0), 1, 4, false);
      cv::putText(matOriginal, S_Diameter, cv::Point(270, 230), FONT_HERSHEY_PLAIN, .7, cv::Scalar(255, 255, 255), 1, 4, false);

      cv::putText(matOriginal, "Posisi ", cv::Point(230, 220), FONT_HERSHEY_PLAIN, .7, cv::Scalar(0, 0, 0), 1, 4, false);
      cv::putText(matOriginal, posisi, cv::Point(270, 220), FONT_HERSHEY_PLAIN, .7, cv::Scalar(255, 255, 255), 1, 4, false);

      stringstream s;
      s << fixed << setprecision(2) << Sudut;
      S_Sudut = s.str();
      cv::putText(matOriginal, "Sudut ", cv::Point(230, 210), FONT_HERSHEY_PLAIN, .7, cv::Scalar(0, 0, 0), 1, 4, false);
      cv::putText(matOriginal, S_Sudut, cv::Point(270, 210), FONT_HERSHEY_PLAIN, .7, cv::Scalar(255, 255, 255), 1, 4, false);
    }

    cv::line(matOriginal, cv::Point(160, 240), cv::Point(160, 0), cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
    cv::imshow("tresh", matProcessed);
    cv::imshow("OUTPUT", matOriginal);

    cout << "\t X= " << x_center << "\t Y= " << y_center << "\t DIAMETER= " << Diameter << "\t JARAK= " << S_Jarak
         << "\t SUDUT= " << S_Sudut << endl;

    charCheckForEscKey = cv::waitKey(1);
  }
  return (0);
}
