// Program to find the largest cv::ellipse using RANSAC and show bounding rectangles and cv::circle around
// it Author: Samarth Manoj Brahmbhatt, University of Pennsylvania

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <algorithm>
#include <math.h>

#define PI 3.14159265

using namespace std;
// using namespace cv;
using namespace Eigen;

// Class to hold RANSAC parameters
class RANSACparams {
private:
  // number of iterations
  int iter;
  // minimum number of inliers to further process a model
  int min_inliers;
  // distance cv::threshold to be conted as inlier
  float dist_thresh;
  // number of points to select randomly at each iteration
  int N;

public:
  RANSACparams(int _iter, int _min_inliers, float _dist_thresh, int _N) { // constructor
    iter = _iter;
    min_inliers = _min_inliers;
    dist_thresh = _dist_thresh;
    N = _N;
  }

  int
  get_iter() {
    return iter;
  }
  int
  get_min_inliers() {
    return min_inliers;
  }
  float
  get_dist_thresh() {
    return dist_thresh;
  }
  int
  get_N() {
    return N;
  }
};

// Class that deals with fitting an cv::ellipse, RANSAC and drawing the cv::ellipse in the image
class ellipseFinder {
private:
  cv::Mat img;                                                // input image
  std::vector<std::vector<cv::Point>> contours;               // contours in image
  cv::Mat Q;                                                  // cv::Matrix representing conic section of detected ellipse
  cv::Mat fit_ellipse(std::vector<cv::Point>);                // function to fit cv::ellipse to a contour
  cv::Mat RANSACellipse(std::vector<std::vector<cv::Point>>); // function to find cv::ellipse in contours using RANSAC
  bool is_good_ellipse(cv::Mat); // function that determines whether given conic section represents a valid ellipse
  std::vector<std::vector<cv::Point>> choose_random(std::vector<cv::Point>); // function to choose points at random from contour
  std::vector<float> distance(cv::Mat,
                              std::vector<cv::Point>); // function to return distance of points from the ellipse
  float distance(cv::Mat,
                 cv::Point);                          // overloaded function to return signed distance of point from ellipse
  void draw_ellipse(cv::Mat);                         // function to draw cv::ellipse in an image
  std::vector<cv::Point> ellipse_contour(cv::Mat);    // function to convert equation of cv::ellipse to a contour of points
  void draw_inliers(cv::Mat, std::vector<cv::Point>); // function to debug inliers

  // RANSAC parameters
  int iter, min_inliers, N;
  float dist_thresh;

public:
  ellipseFinder(cv::Mat _img, int l_canny, int h_canny, RANSACparams rp) { // constructor
    img = _img.clone();

    // Edge detection and contour extraction
    cv::Mat edges;
    cv::Canny(img, edges, l_canny, h_canny);
    std::vector<std::vector<cv::Point>> c;
    cv::findContours(edges, c, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
    // Remove small spurious short contours
    for(int i = 0; i < c.size(); i++) {
      bool is_closed = false;

      std::vector<cv::Point> _c = c[i];

      cv::Point p1 = _c.front(), p2 = _c.back();
      float d = sqrt((p1.x - p2.x) ^ 2 + (p1.y - p2.y) ^ 2);
      if(d <= 0.5)
        is_closed = true;

      d = cv::arcLength(_c, is_closed);

      if(d > 50)
        contours.push_back(_c);
    }

    iter = rp.get_iter();
    min_inliers = rp.get_min_inliers();
    N = rp.get_N();
    dist_thresh = rp.get_dist_thresh();

    Q = cv::Mat::eye(6, 1, CV_32F);

    /*
    //for debug
    cv::Mat img_show = img.clone();
    cv::drawContours(img_show, contours, -1, cv::Scalar(0, 0, 255));
    cv::imshow("Contours", img_show);
    //cv::imshow("Edges", edges);
    */
    cout << "No. of Contours = " << contours.size() << endl;
  }

  void detect_ellipse(); // final wrapper function
  void debug();          // debug function
};

std::vector<float>
ellipseFinder::distance(cv::Mat Q, std::vector<cv::Point> c) {
  std::vector<cv::Point> cv::ellipse = ellipse_contour(Q);
  std::vector<float> distances;
  for(int i = 0; i < c.size(); i++) { distances.push_back(float(cv::pointPolygonTest(cv::ellipse, c[i], true))); }

  return distances;
}

float
ellipseFinder::distance(cv::Mat Q, cv::Point p) {
  std::vector<cv::Point> cv::ellipse = ellipse_contour(Q);
  return float(cv::pointPolygonTest(cv::ellipse, p, true));
}

std::vector<std::vector<cv::Point>>
ellipseFinder::choose_random(std::vector<cv::Point> c) {
  std::vector<std::vector<cv::Point>> cr;
  std::vector<cv::Point> cr0, cr1;
  // Randomly shuffle all elements of contour
  std::random_shuffle(c.begin(), c.end());
  // Put the first N elements into cr[0] as consensus set (see Wikipedia RANSAC algorithm) and
  // the rest in cr[1] to check for inliers
  for(int i = 0; i < c.size(); i++) {
    if(i < N)
      cr0.push_back(c[i]);
    else
      cr1.push_back(c[i]);
  }
  cr.push_back(cr0);
  cr.push_back(cr1);

  return cr;
}

cv::Mat
ellipseFinder::fit_ellipse(std::vector<cv::Point> c) {
  /*
  // for debug
  cv::Mat img_show = img.clone();
  std::vector<std::vector<cv::Point> > cr;
  cr.push_back(c);
  cv::drawContours(img_show, cr, -1, cv::Scalar(0, 0, 255), 2);
  cv::imshow("Debug cv::fitEllipse", img_show);
  */
  int N = c.size();

  cv::Mat D;
  for(int i = 0; i < N; i++) {
    cv::Point p = c[i];
    cv::Mat r(1, 6, CV_32FC1);
    r = (cv::Mat_<float>(1, 6) << (p.x) * (p.x), (p.x) * (p.y), (p.y) * (p.y), p.x, p.y, 1.f);
    D.push_back(r);
  }
  cv::Mat S = D.t() * D, _S;

  double d = invert(S, _S);
  // if(d < 0.001) cout << "S matrix is singular" << endl;

  cv::Mat C = cv::Mat::zeros(6, 6, CV_32F);
  C.at<float>(2, 0) = 2;
  C.at<float>(1, 1) = -1;
  C.at<float>(0, 2) = 2;

  // Using EIGEN to calculate eigenvalues and eigenstd::vectors
  cv::Mat prod = _S * C;
  Eigen::cv::MatrixXd prod_e;
  cv2eigen(prod, prod_e);
  EigenSolver<Eigen::cv::MatrixXd> es(prod_e);

  cv::Mat evec, eval, vec(6, 6, CV_32FC1), val(6, 1, CV_32FC1);

  eigen2cv(es.eigenstd::vectors(), evec);
  eigen2cv(es.eigenvalues(), eval);

  evec.convertTo(evec, CV_32F);
  eval.convertTo(eval, CV_32F);

  // Eigen returns complex parts in the second channel (which are all 0 here) so select just the
  // first channel
  int from_to[] = {0, 0};
  cv::mixChannels(&evec, 1, &vec, 1, from_to, 1);
  cv::mixChannels(&eval, 1, &val, 1, from_to, 1);

  cv::Point maxLoc;
  cv::minMaxLoc(val, NULL, NULL, NULL, &maxLoc);

  return vec.col(maxLoc.y);
}

bool
ellipseFinder::is_good_ellipse(cv::Mat Q) {
  float a = Q.at<float>(0, 0), b = (Q.at<float>(1, 0)) / 2, c = Q.at<float>(2, 0), d = (Q.at<float>(3, 0)) / 2,
        f = (Q.at<float>(4, 0)) / 2, g = Q.at<float>(5, 0);

  if(b * b - a * c == 0)
    return false;

  float thresh = 0.09, num = 2 * (a * f * f + c * d * d + g * b * b - 2 * b * d * f - a * c * g),
        den1 = (b * b - a * c) * (sqrt((a - c) * (a - c) + 4 * b * b) - (a + c)),
        den2 = (b * b - a * c) * (-sqrt((a - c) * (a - c) + 4 * b * b) - (a + c)), a_len = sqrt(num / den1),
        b_len = sqrt(num / den2), major_axis = max(a_len, b_len), minor_axis = min(a_len, b_len);

  if(minor_axis < thresh * major_axis || num / den1 < 0.f || num / den2 < 0.f || major_axis > max(img.rows, img.cols))
    return false;
  else
    return true;
}

cv::Mat
ellipseFinder::RANSACellipse(std::vector<std::vector<cv::Point>> contours) {
  int best_overall_inlier_score = 0;
  cv::Mat Q_best = 777 * cv::Mat::ones(6, 1, CV_32FC1);
  int idx_best = -1;

  // for each contour...
  for(int i = 0; i < contours.size(); i++) {
    std::vector<cv::Point> c = contours[i];
    if(c.size() < min_inliers)
      continue;

    cv::Mat Q;
    int best_inlier_score = 0;
    for(int j = 0; j < iter; j++) {
      // ...choose points at random...
      std::vector<std::vector<cv::Point>> cr = choose_random(c);
      std::vector<cv::Point> consensus_set = cr[0], rest = cr[1];
      // ...fit cv::ellipse to those points...
      cv::Mat Q_maybe = fit_ellipse(consensus_set);
      // ...check for inliers...
      std::vector<float> d = distance(Q_maybe, rest);
      for(int k = 0; k < d.size(); k++)
        if(abs(d[k]) < dist_thresh)
          consensus_set.push_back(rest[k]);
      // ...and find the random set with the most number of inliers
      if(consensus_set.size() > min_inliers && consensus_set.size() > best_inlier_score) {
        Q = fit_ellipse(consensus_set);
        best_inlier_score = consensus_set.size();
      }
    }
    // find cotour with cv::ellipse that has the most number of inliers
    if(best_inlier_score > best_overall_inlier_score && is_good_ellipse(Q)) {
      best_overall_inlier_score = best_inlier_score;
      Q_best = Q.clone();
      if(Q_best.at<float>(5, 0) < 0)
        Q_best *= -1.f;
      idx_best = i;
    }
  }

  /*
  //for debug
  cv::Mat img_show = img.clone();
  cv::drawContours(img_show, contours, idx_best, cv::Scalar(0, 0, 255), 2);
  cv::imshow("Best Contour", img_show);

  cout << "inliers " << best_overall_inlier_score << endl;
  */
  return Q_best;
}

std::vector<cv::Point>
ellipseFinder::ellipse_contour(cv::Mat Q) {
  float a = Q.at<float>(0, 0), b = (Q.at<float>(1, 0)) / 2, c = Q.at<float>(2, 0), d = (Q.at<float>(3, 0)) / 2,
        f = (Q.at<float>(4, 0)) / 2, g = Q.at<float>(5, 0);

  std::vector<cv::Point> cv::ellipse;
  if(b * b - a * c == 0) {
    cv::ellipse.push_back(cv::Point(0, 0));
    return cv::ellipse;
  }

  cv::Point2f center((c * d - b * f) / (b * b - a * c), (a * f - b * d) / (b * b - a * c));

  float num = 2 * (a * f * f + c * d * d + g * b * b - 2 * b * d * f - a * c * g),
        den1 = (b * b - a * c) * (sqrt((a - c) * (a - c) + 4 * b * b) - (a + c)),
        den2 = (b * b - a * c) * (-sqrt((a - c) * (a - c) + 4 * b * b) - (a + c)), a_len = sqrt(num / den1),
        b_len = sqrt(num / den2), major_axis = max(a_len, b_len), minor_axis = min(a_len, b_len);

  // angle of rotation of ellipse
  float alpha = 0.f;
  if(b == 0.f && a == c)
    alpha = PI / 2;
  else if(b != 0.f && a > c)
    alpha = 0.5 * atan2(2 * b, a - c);
  else if(b != 0.f && a < c)
    alpha = PI / 2 - 0.5 * atan2(2 * b, a - c);

  // 'draw' the cv::ellipse and put it into a STL cv::Point std::vector so you can use cv::drawContours()
  int N = 200;
  float theta = 0.f;
  for(int i = 0; i < N; i++, theta += 2 * PI / N) {
    float x = center.x + major_axis * cos(theta) * cos(alpha) + minor_axis * sin(theta) * sin(alpha);
    float y = center.y - major_axis * cos(theta) * sin(alpha) + minor_axis * sin(theta) * cos(alpha);
    cv::Point p(x, y);
    if(x < img.cols && y < img.rows)
      cv::ellipse.push_back(p);
  }
  if(cv::ellipse.size() == 0)
    cv::ellipse.push_back(cv::Point(0, 0));

  return cv::ellipse;
}

void
ellipseFinder::detect_ellipse() {
  Q = RANSACellipse(contours);
  cout << "Q" << Q << endl;
  draw_ellipse(Q);
}

void
ellipseFinder::debug() {
  int i = 1; // index of contour you want to debug
  cout << "No. of points in contour " << contours[i].size() << endl;
  cv::Mat a = fit_ellipse(contours[i]);
  cv::Mat img_show = img.clone();
  cv::drawContours(img_show, contours, i, cv::Scalar(0, 0, 255), 3);
  cv::imshow("Debug contour", img_show);
  draw_inliers(a, contours[i]);
  draw_ellipse(a);
}

void
ellipseFinder::draw_ellipse(cv::Mat Q) {
  std::vector<cv::Point> cv::ellipse = ellipse_contour(Q);
  std::vector<std::vector<cv::Point>> c;
  c.push_back(cv::ellipse);
  cv::Mat img_show = img.clone();
  // draw ellipse
  cv::drawContours(img_show, c, -1, cv::Scalar(0, 0, 255), 3);

  // compute bounding shapes
  cv::RotatedRect r_rect = cv::minAreaRect(cv::ellipse);
  cv::Rect rect = cv::boundingRect(cv::ellipse);
  cv::Point2f center;
  float radius;
  cv::minEnclosingCircle(cv::ellipse, center, radius);

  // draw bounding shapes
  cv::rectangle(img_show, rect, cv::Scalar(255, 0, 255));      // magenta
  cv::circle(img_show, center, radius, cv::Scalar(255, 0, 0)); // blue
  cv::Point2f vertices[4];
  r_rect.points(vertices);
  for(int i = 0; i < 4; i++) cv::line(img_show, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 255, 0)); // green

  cv::imshow("Ellipse", img_show);
}

void
ellipseFinder::draw_inliers(cv::Mat Q, std::vector<cv::Point> c) {
  std::vector<cv::Point> cv::ellipse = ellipse_contour(Q);
  std::vector<std::vector<cv::Point>> cs;
  cs.push_back(cv::ellipse);
  cv::Mat img_show = img.clone();
  // draw all contours in thin red
  cv::drawContours(img_show, contours, -1, cv::Scalar(0, 0, 255));
  // draw cv::ellipse in thin blue
  cv::drawContours(img_show, cs, 0, cv::Scalar(255, 0, 0));
  int count = 0;
  // draw inliers as green points
  for(int i = 0; i < c.size(); i++) {
    double d = cv::pointPolygonTest(cv::ellipse, c[i], true);
    float d1 = float(d);
    if(abs(d1) < dist_thresh) {
      cv::circle(img_show, c[i], 1, cv::Scalar(0, 255, 0), -1);
      count++;
    }
  }
  cv::imshow("Debug inliers", img_show);
  cout << "inliers " << count << endl;
}

int
main() {
  cv::Mat img = cv::imread("test4.jpg");
  cv::namedWindow("Ellipse");

  // object holding RANSAC parameters, initialized using the constructor
  RANSACparams rp(400, 100, 1, 5);

  // cv::Canny thresholds
  int canny_l = 250, canny_h = 300;
  // Ellipse finder object, initialized using the constructor
  ellipseFinder ef(img, canny_l, canny_h, rp);
  ef.detect_ellipse();
  // ef.debug();

  while(char(cv::waitKey(1)) != 'q') {}

  return 0;
}
