#pragma once
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

#include <vector>
class Polygon {
public:
  vector<Point2d> points;             // Points du polygone.
  vector<Point2d> regularPoints;      // Points du polygone reechantilloné.
  Polygon();                          // ctor vide
  Polygon(vector<Point2d> points);    // ctor avec les points du polygon.
  void update(const Mat& G);          // Met à jour les tangentes etcs.
  void addPoint(Point2d p);           // adds a point to the polygon.
  void drawPolygon(const Mat& Image); // draws the polygon on a given image.
  void drawNormales(const Mat& Image);
  void drawCurvatures(const Mat& Image);
  vector<Point2d> rechantillonage(); // adds regular points on the polygon.
  vector<Vec2d> computeTangentes();
  vector<Vec2d> computeNormales();
  vector<Vec2d> computeDeriv();
  vector<double> computeCurvature();
  Vec2d getNormale(int i);
  double getCurvature(int i);
  Point2d getPoint(int i);
  void replacePoint(int i, Point2d p);
  int
  getRegularPointSize() {
    return regularPoints.size();
  };
  vector<Point2d> deleteLoop(); // détecte les boucles internes et supprime les
                                // points de la plus petite.
  ~Polygon();                   // destructor

private:
  vector<Vec2d> tangentes; // Tangentes au polygone rééchantillonné.
  vector<Vec2d> normales;  // Normales au polygone rééchantillonné.
  vector<Vec2d> deriveeseconde;
  vector<double> curvature;
};