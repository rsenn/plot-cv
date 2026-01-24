#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <math.h>
#include "polygon.hpp"
#include "color.hpp"
// using namespace cv;

#include <vector>

const int PIXEL_STEP = 15; // Ecart entre les points du polygone après ré-échantillonage.

template<class T>
static bool
segment_cut(cv::Point_<T> p1, cv::Point_<T> p2, cv::Point_<T> q1, cv::Point_<T> q2) {
  double bq, bp, aq, ap;
  ap = (p2.y - p1.y) / (p2.x - p1.x);
  aq = (q2.y - q1.y) / (q2.x - q1.x);
  bp = p1.y - ap * p1.x;
  bq = q1.y - aq * q1.x;
  if(ap == aq)
    return false;
  double xintersect = (bq - bp) / (ap - aq);
  if(xintersect == q1.x || xintersect == q2.x)
    return false;
  if(xintersect == p1.x || xintersect == p2.x)
    return false;
  double tq = (xintersect - q1.x) / (q2.x - q1.x);
  double tp = (xintersect - p1.x) / (p2.x - p1.x);
  if(tp > 0.01 && tp < 0.99 && tq > 0.01 && tq < 0.99) {
    std::cout << "Tp: " << tp << std::endl;
    std::cout << "Tq: " << tq << std::endl;
    return true;
  }
  return false;
}

template<class T>
static cv::Point_<T>
convex_point(cv::Point_<T> begin_point, cv::Point_<T> end_point, T t) {
  return cv::Point_<T>(t * end_point.x + (1 - t) * begin_point.x,
                       t * end_point.y + (1 - t) * begin_point.y);
}

template<>
Polygon_<double>::vector_type
Polygon_<double>::resampling() {
  int l = points.size();
  // std::cout << "Initial points: " << l << std::endl;
  vector_type new_points;
  point_type begin_point = points[0];
  point_type end_point = begin_point;
  double pas = 0;
  for(int i = 1; i <= l; i++) {
    end_point = points[i % l];
    if(norm(begin_point - end_point) > PIXEL_STEP) {
      pas = PIXEL_STEP / (norm(begin_point - end_point));
      for(double t = 0; t <= 1; t += pas) {
        point_type intermediate_point = convex_point(begin_point, end_point, t);
        if(norm(intermediate_point - end_point) > PIXEL_STEP) {
          new_points.push_back(intermediate_point);
        }
      }
    } else {
      new_points.push_back(begin_point);
    }
    begin_point = end_point;
  }

  // std::cout<< "Rechantillonage: " << new_points.size() << " points." <<endl;
  return new_points;
}

template<>
Polygon_<double>::vec2vec_type
Polygon_<double>::compute_tangents() {
  int L = regular_points.size();
  vec2vec_type tangentes(L);
  if(regular_points.size() < 3)
    return tangentes;

  for(int i = 1; i <= L; i++) {
    point_type before = regular_points[(i - 1) % L];
    point_type after = regular_points[(i + 1) % L];
    vec2_type tangente((after.x - before.x) / 2, (after.y - before.y) / 2);
    tangentes[i % L] = tangente;
  }
  return tangentes;
}

template<>
Polygon_<double>::vec2vec_type
Polygon_<double>::compute_normals() {
  int L = regular_points.size();
  vec2vec_type normals;
  for(int i = 0; i < L; i++) {
    vec2_type normal(-tangentes[i](1), tangentes[i](0));
    if(norm(normal) == 0) {
      // std::cout << i << " normal de norme nulle" << std::endl;
      normals.push_back(normal);
    } else {
      vec2_type norme1 = (1 / norm(normal)) * normal;
      normals.push_back(norme1);
    }
  }
  return normals;
}

template<>
Polygon_<double>::vec2vec_type
Polygon_<double>::compute_deriv() {
  int L = regular_points.size();
  vec2vec_type secondes(L);
  for(int i = 1; i <= L; i++) {
    point_type before = regular_points[(i - 1) % L];
    point_type current = regular_points[i % L];
    point_type after = regular_points[(i + 1) % L];
    vec2_type derivee(after.x - 2 * current.x + before.x, after.y - 2 * current.y + before.y);
    secondes[i % L] = derivee;
  }
  return secondes;
}

template<>
Polygon_<double>::values_type
Polygon_<double>::compute_curvature() {
  int L = regular_points.size();
  values_type curvature(L);
  double xp, yp, xpp, ypp;
  for(int i = 0; i < L; i++) {
    xp = tangentes[i](0);
    yp = tangentes[i](1);
    xpp = deriveeseconde[i](0);
    ypp = deriveeseconde[i](1);
    curvature[i] = (xpp * yp - xp * ypp) / pow((xp * xp + yp * yp), 1.5);
  }
  return curvature;
}

template<>
Polygon_<double>::vector_type
Polygon_<double>::delete_loop() {
  int L = regular_points.size();
  if(L < 2)
    return regular_points;
  vector_type new_points;
  point_type p1, p2, q1, q2;
  int cut1, cut2;
  bool found = false;
  for(int i = 0; i < L; i++) {
    if(found)
      break;
    p1 = regular_points[i % L];
    p2 = regular_points[(i + 1) % L];
    for(int j = 0; j < L; j++) {
      if(found)
        break;
      if(j != i && j != (i + 1 % L)) {
        q1 = regular_points[j % L];
        q2 = regular_points[(j + 1) % L];
        if(segment_cut(p1, p2, q1, q2)) {
          found = true;
          int b1 = j - i;
          int b2 = i - j;
          b1 = (b1 < 0) ? (b1 + L) : b1;
          b2 = (b2 < 0) ? (b2 + L) : b2;
          if(b1 > b2) {
            cut1 = (i + 1);
            cut2 = j;
          } else {
            cut1 = (j + 1);
            cut2 = i;
          }
        }
      }
    }
  }
  if(found) {
    std::cout << "Found loop: " << cut1 << ", " << cut2 << std::endl;
    // waitKey();
    if(cut2 < cut1) {
      cut2 += L;
    }
    for(int k = cut1; k <= cut2; k++) {
      new_points.push_back(regular_points[k % L]);
    }
    return new_points;
  } else {
    return regular_points;
  }
}

// Détermine si le segment p1 p2 coupt q1 q2.
template<> Polygon_<double>::Polygon_() {
  vector_type pts;
  points = vector_type();
  vec2vec_type norm;
  regular_points = pts;
  tangentes = norm;
  normals = norm;
  deriveeseconde = norm;
  curvature = values_type();
}

template<> Polygon_<double>::Polygon_(const vector_type& pts) {
  std::cout << "New polygon with " << pts.size() << " points." << std::endl;
  points = pts;
  regular_points = resampling();
  tangentes = compute_tangents();
  normals = compute_normals();
  deriveeseconde = compute_deriv();
  curvature = compute_curvature();
}

template<>
void
Polygon_<double>::add_point(point_type p) {
  points.push_back(p);
  regular_points = resampling();
  tangentes = compute_tangents();
  normals = compute_normals();
  deriveeseconde = compute_deriv();
  curvature = compute_curvature();
}

template<>
void
Polygon_<double>::update(const cv::Mat& G) {
  regular_points = resampling();
  regular_points = delete_loop();
  tangentes = compute_tangents();
  normals = compute_normals();
  deriveeseconde = compute_deriv();
  curvature = compute_curvature();
}

template<>
void
Polygon_<double>::replace_point(int i, point_type p) {
  regular_points.at(i) = p;
}

template<>
void
Polygon_<double>::draw_polygon(const cv::Mat& Image) {
  if(points.size() < 2)
    return;
  // std::cout << "Drawing polygon with " << points.size() << std::endl;
  point_type begin_point = regular_points[0];
  point_type end_point = begin_point;
  for(int i = 1; i <= regular_points.size(); i++) {
    end_point = regular_points[i % regular_points.size()];
    circle(Image, begin_point, 2, color_type(255, 255, 0), 5);
    line(Image, begin_point, end_point, color_type(0, 255, 0));
    begin_point = end_point;
  }
  // draw_normals(Image);
  // draw_curvatures(Image);
  imshow("images", Image);
}

template<>
void
Polygon_<double>::draw_normals(const cv::Mat& Image) {
  if(points.size() < 3)
    return;
  for(int i = 0; i < normals.size(); i++) {
    double x = normals[i](0);
    double y = normals[i](1);
    point_type p(20 * x + regular_points[i].x, 20 * y + regular_points[i].y);
    // std::cout << p.x <<", " << p.y << std::endl;
    line(Image, regular_points[i], p, color_type(0, 0, 255));
  }
}

template<>
void
Polygon_<double>::draw_curvatures(const cv::Mat& Image) {
  if(points.size() < 3)
    return;
  for(int i = 0; i < curvature.size(); i++) {
    double x = normals[i](0);
    double y = normals[i](1);
    point_type p(600 * curvature[i] * x + regular_points[i].x,
                 600 * curvature[i] * y + regular_points[i].y);
    // std::cout << p.x <<", " << p.y << std::endl;
    line(Image, regular_points[i], p, color_type(255, 120, 255));
  }
}

template<>
double
Polygon_<double>::get_curvature(int i) {
  return curvature.at(i);
}

template<>
Polygon_<double>::vec2_type
Polygon_<double>::get_normal(int i) {
  return normals.at(i);
}

template<>
Polygon_<double>::point_type
Polygon_<double>::get_point(int i) {
  return regular_points.at(i);
}

template<> Polygon_<double>::~Polygon_() {
}
