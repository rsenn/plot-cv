#include "data.hpp"
#include "plot-cv.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <iostream>
#include "polygon.hpp"

const float COEFF_CURV = 120;
const float COEFF_BALL = 1; // Coefficient de la force ballon.

cv::Mat getGradient(const cv::Mat& I);      // Calcule la norme du gradient de l'image
cv::Mat applyFunction(const cv::Mat& I);    // Applique la fonction g aux valeurs.
cv::Mat* computeGradient(const cv::Mat& I); // Calcule le gradient de l'image

float
g(float x) {
  return 1 / (1 + x * x);
} // fonctin g.

bool
isvalid(float x) {
  return (x * 0 == 0);
} // vérifie que le float est bien un number.

bool
checkValidity(const cv::Mat& I) {
  int m = I.rows, n = I.cols;
  for(int i = 0; i < m; i++) {
    for(int j = 0; j < n; j++) {
      if(!isvalid(I.at<float>(i, j)))
        return false;
    }
  }
  return true;
} // vérifie que la matrice est composée de float valides.

bool
Data::is_valid_point(point_type<double> p) {
  int m = image.rows, n = image.cols;
  return (p.x >= 0 && p.x < n && p.y >= 0 && p.y < m);
}

Data::Data(cv::Mat& A) {
  cv::Mat I(A.rows, A.cols, CV_32F);
  cvtColor(A, I, CV_BGR2GRAY);
  image = A;
  bwImage = I;
  gradient = getGradient(I);
  gGradient = applyFunction(gradient);
  cv::Mat* gradientG = computeGradient(gGradient);
  gx = gradientG[0];
  gy = gradientG[1];
  polygon = Polygon();
}

void
Data::draw_next_step(double step, const cv::Mat& Image, SegmentationMode mode) {
  cv::Mat copyImage = Image.clone();
  int m = Image.rows;
  int r, c;
  // std::cout << "Drawing next step" << std::endl;
  polygon.draw_polygon(copyImage);
  int L = polygon.regular_points.size();
  point_vector<double> nextPointss;
  for(int i = 0; i < L; i++) {
    point_type<double> p = polygon.get_point(i);
    r = (int)p.y;
    c = (int)p.x;
    float g = gGradient.at<float>(r, c);
    double curv = polygon.get_curvature(i);
    cv::Vec2d normal = polygon.get_normal(i);
    float gradx = gx.at<float>(r, c);
    float grady = gy.at<float>(r, c);
    float nx = normal(0);
    float ny = normal(1);
    // std::cout << "Step" << i << ": " << g * curv  * step<< std::endl;
    float scalar = 0.;
    switch(mode) {
      case BALLOON_MODE: scalar = -(curv + COEFF_BALL) * g * step; break;
      case CURV_MODE: scalar = -curv * COEFF_CURV; break;
      case ALL_MODE:
        curv = curv + COEFF_BALL;
        scalar = (gradx * nx + grady * ny - (curv)*g) * step;
        break;
    }
    // std::cout << scalar << std::endl;
    point_type<double> nextP(scalar * nx + p.x, scalar * ny + p.y);
    //	if (is_valid_point(nextP)) {
    line(copyImage, nextP, p, cv::Scalar(0, 0, 255));
    //	cout << "Drawing line: " << i << std::endl;
    /*	} else {
            std::cout << "point_type<int> of polygon: " << (int)p.x << ", " << (int) p.y<<
       std::endl; std::cout << "Step: " << i << " is invalid." << std::endl;
            std::cout << "Scalaire: " << scalar << std::endl;
            std::cout << "Gradient: " << gradx << ", " << grady << std::endl;
            std::cout << "Normale: " << nx << ", " << ny << std::endl;
            std::cout << "Curvature: " <<curv << std::endl;
            std::cout << "g : " << g << std::endl;
            std::cout << "point_type<int>: " << nextP.x << ", " << nextP.y << std::endl;
        }*/
    // std::cout << "Step i: " <<p.x <<", " << p.y << std::endl;
  }
  imshow("images", copyImage);
}

void
Data::find_contour(double step, SegmentationMode mode) {
  int r, c;
  int L = polygon.get_regular_point_size();
  point_vector<double> nextPointss(L);
  for(int i = 0; i < L; i++) {
    point_type<double> p = polygon.get_point(i);
    // std::cout << "i: " << i << " size: " << L << std::endl;
    r = (int)p.y;
    c = (int)p.x;
    float g = gGradient.at<float>(r, c);
    double curv = polygon.get_curvature(i);
    cv::Vec2d normal = polygon.get_normal(i);
    float gradx = gx.at<float>(r, c);
    float grady = gy.at<float>(r, c);
    float nx = normal(0);
    float ny = normal(1);
    float scalar = 0.;
    switch(mode) {
      case BALLOON_MODE: scalar = -(curv + COEFF_BALL) * g * step; break;
      case CURV_MODE: scalar = -curv * COEFF_CURV; break;
      case ALL_MODE:
        curv = curv + COEFF_BALL;
        scalar = (gradx * nx + grady * ny - curv * g) * step;
        break;
    }
    point_type<double> nextP(scalar * nx + p.x, scalar * ny + p.y);
    if(is_valid_point(nextP)) {
      nextPointss.at(i) = nextP;
    } else {
      nextP.x = nx + p.x;
      nextP.y = ny + p.y;
      nextPointss.at(i) = nextP;
    }
  }
  polygon.points = nextPointss;
  polygon.update(gGradient);
}

cv::Mat
getGradient(const cv::Mat& I) {
  int m = I.rows, n = I.cols;
  cv::Mat G(m, n, CV_32F);
  for(int i = 0; i < m; i++) {
    for(int j = 0; j < n; j++) {
      float ix, iy;
      if(i == 0 || i == m - 1)
        iy = 0;
      else
        iy = (float(I.at<uchar>(i + 1, j)) - float(I.at<uchar>(i - 1, j))) / 2;
      if(j == 0 || j == n - 1)
        ix = 0;
      else
        ix = (float(I.at<uchar>(i, j + 1)) - float(I.at<uchar>(i, j - 1))) / 2;

      G.at<float>(i, j) = sqrt(ix * ix + iy * iy);
    }
  }
  return G;
}

cv::Mat*
computeGradient(const cv::Mat& I) {
  int m = I.rows, n = I.cols;
  std::cout << m << ", " << n << std::endl;
  cv::Mat Ix(m, n, CV_32F), Iy(m, n, CV_32F);
  for(int i = 0; i < m; i++) {
    for(int j = 0; j < n; j++) {
      float ix, iy;
      if(i == 0 || i == m - 1)
        iy = 0;
      else
        iy = (I.at<float>(i + 1, j) - I.at<float>(i - 1, j)) / 2;
      if(j == 0 || j == n - 1)
        ix = 0;
      else
        ix = (I.at<float>(i, j + 1) - I.at<float>(i, j - 1)) / 2;

      Ix.at<float>(i, j) = ix;
      Iy.at<float>(i, j) = iy;
    }
  }
  cv::Mat* result = new cv::Mat[2];
  result[0] = Ix;
  result[1] = Iy;
  return result;
}

cv::Mat
applyFunction(const cv::Mat& I) {
  int m = I.rows, n = I.cols;
  cv::Mat result(m, n, CV_32F);
  for(int i = 0; i < m; i++) {
    for(int j = 0; j < n; j++) {
      result.at<float>(i, j) = g(I.at<float>(i, j));
    }
  }
  return result;
}
