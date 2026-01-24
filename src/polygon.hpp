#ifndef POLYGON_HPP
#define POLYGON_HPP

#pragma once
#include <opencv2/highgui/highgui.hpp>

#include <vector>

template<class T> class Polygon_ {
public:
  typedef cv::Point_<T> point_type;
  typedef std::vector<point_type> vector_type;
  typedef T value_type;
  typedef std::vector<T> values_type;
  typedef cv::Vec<T, 2> vec2_type;
  typedef std::vector<vec2_type> vec2vec_type;

  vector_type points;         // cv::Points du polygone.
  vector_type regular_points; // cv::Points du polygone reechantilloné.

  Polygon_();                          // ctor vide
  Polygon_(const vector_type& points); // ctor avec les points du polygon.

  void update(const cv::Mat& G);           // Met à jour les tangentes etcs.
  void add_point(point_type p);            // adds a point to the polygon.
  void draw_polygon(const cv::Mat& Image); // draws the polygon on a given image.
  void draw_normals(const cv::Mat& Image);
  void draw_curvatures(const cv::Mat& Image);

  vector_type resampling(); // adds regular points on the polygon.
  vec2vec_type compute_tangents();
  vec2vec_type compute_normals();
  vec2vec_type compute_deriv();
  values_type compute_curvature();
  vec2_type get_normal(int i);
  value_type get_curvature(int i);
  point_type get_point(int i);
  void replace_point(int i, point_type p);
  int get_regular_point_size() { return regular_points.size(); };
  vector_type delete_loop(); // détecte les boucles internes et supprime les
                             // points de la plus petite.
  ~Polygon_();               // destructor

private:
  vec2vec_type tangentes; // Tangentes au polygone rééchantillonné.
  vec2vec_type normals;   // Normales au polygone rééchantillonné.
  vec2vec_type deriveeseconde;
  values_type curvature;
};

typedef Polygon_<float> Polygon2f;
typedef Polygon_<double> Polygon2d;
typedef Polygon_<int> Polygon2i;

typedef Polygon2d Polygon;

#endif // defined POLYGON_HPP
