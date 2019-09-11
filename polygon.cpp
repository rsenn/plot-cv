#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <math.h>
#include "polygon.h"
using namespace cv;
using namespace std;

#include <vector>

const int PIXEL_STEP = 15; // Ecart entre les points du polygone après ré-échantillonage.

bool segmentCut(Point2d p1, Point2d p2, Point2d q1, Point2d q2);
// Détermine si le segment p1 p2 coupt q1 q2.

Polygon::Polygon() {
  std::vector<Point2d> pts;
  points = std::vector<Point2d>();
  std::vector<Vec2d> norm;
  regularPoints = pts;
  tangentes = norm;
  normales = norm;
  deriveeseconde = norm;
  curvature = std::vector<double>();
}

Polygon::Polygon(vector<Point2d> pts) {
  cout << "New polygon with " << pts.size() << " points." << endl;
  points = pts;
  regularPoints = rechantillonage();
  tangentes = computeTangentes();
  normales = computeNormales();
  deriveeseconde = computeDeriv();
  curvature = computeCurvature();
}

void
Polygon::addPoint(Point2d p) {
  points.push_back(p);
  regularPoints = rechantillonage();
  tangentes = computeTangentes();
  normales = computeNormales();
  deriveeseconde = computeDeriv();
  curvature = computeCurvature();
}

void
Polygon::update(const Mat& G) {
  regularPoints = rechantillonage();
  regularPoints = deleteLoop();
  tangentes = computeTangentes();
  normales = computeNormales();
  deriveeseconde = computeDeriv();
  curvature = computeCurvature();
}

void
Polygon::replacePoint(int i, Point2d p) {
  regularPoints.at(i) = p;
}

void
Polygon::drawPolygon(const Mat& Image) {
  if(points.size() < 2)
    return;
  // cout << "Drawing polygon with " << points.size() << endl;
  Point beginPoint = regularPoints[0];
  Point endPoint = beginPoint;
  for(int i = 1; i <= regularPoints.size(); i++) {
    endPoint = regularPoints[i % regularPoints.size()];
    circle(Image, beginPoint, 2, Scalar(255, 255, 0), 5);
    line(Image, beginPoint, endPoint, Scalar(0, 255, 0));
    beginPoint = endPoint;
  }
  // drawNormales(Image);
  // drawCurvatures(Image);
  imshow("images", Image);
}

void
Polygon::drawNormales(const Mat& Image) {
  if(points.size() < 3)
    return;
  for(int i = 0; i < normales.size(); i++) {
    double x = normales[i](0);
    double y = normales[i](1);
    Point2d p(20 * x + regularPoints[i].x, 20 * y + regularPoints[i].y);
    // cout << p.x <<", " << p.y << endl;
    line(Image, regularPoints[i], p, Scalar(0, 0, 255));
  }
}

void
Polygon::drawCurvatures(const Mat& Image) {
  if(points.size() < 3)
    return;
  for(int i = 0; i < curvature.size(); i++) {
    double x = normales[i](0);
    double y = normales[i](1);
    Point2d p(600 * curvature[i] * x + regularPoints[i].x, 600 * curvature[i] * y + regularPoints[i].y);
    // cout << p.x <<", " << p.y << endl;
    line(Image, regularPoints[i], p, Scalar(255, 120, 255));
  }
}

Point
convexPoint(Point beginPoint, Point endPoint, double t) {
  return Point(t * endPoint.x + (1 - t) * beginPoint.x, t * endPoint.y + (1 - t) * beginPoint.y);
}

vector<Point2d>
Polygon::rechantillonage() {
  int l = points.size();
  // cout << "Initial points: " << l << endl;
  std::vector<Point2d> newPoints;
  Point beginPoint = points[0];
  Point endPoint = beginPoint;
  double pas = 0;
  for(int i = 1; i <= l; i++) {
    endPoint = points[i % l];
    if(norm(beginPoint - endPoint) > PIXEL_STEP) {
      pas = PIXEL_STEP / (norm(beginPoint - endPoint));
      for(double t = 0; t <= 1; t += pas) {
        Point intermediatePoint = convexPoint(beginPoint, endPoint, t);
        if(norm(intermediatePoint - endPoint) > PIXEL_STEP) {
          newPoints.push_back(intermediatePoint);
        }
      }
    } else {
      newPoints.push_back(beginPoint);
    }
    beginPoint = endPoint;
  }

  // cout<< "Rechantillonage: " << newPoints.size() << " points." <<endl;
  return newPoints;
}

vector<Vec2d>
Polygon::computeTangentes() {
  int L = regularPoints.size();
  std::vector<Vec2d> tangentes(L);
  if(regularPoints.size() < 3)
    return tangentes;

  for(int i = 1; i <= L; i++) {
    Point before = regularPoints[(i - 1) % L];
    Point after = regularPoints[(i + 1) % L];
    Vec2d tangente((after.x - before.x) / 2, (after.y - before.y) / 2);
    tangentes[i % L] = tangente;
  }
  return tangentes;
}

vector<Vec2d>
Polygon::computeNormales() {
  int L = regularPoints.size();
  std::vector<Vec2d> normales;
  for(int i = 0; i < L; i++) {
    Vec2d normale(-tangentes[i](1), tangentes[i](0));
    if(norm(normale) == 0) {
      // cout << i << " normale de norme nulle" << endl;
      normales.push_back(normale);
    } else {
      Vec2d norme1 = (1 / norm(normale)) * normale;
      normales.push_back(norme1);
    }
  }
  return normales;
}

vector<Vec2d>
Polygon::computeDeriv() {
  int L = regularPoints.size();
  std::vector<Vec2d> secondes(L);
  for(int i = 1; i <= L; i++) {
    Point before = regularPoints[(i - 1) % L];
    Point current = regularPoints[i % L];
    Point after = regularPoints[(i + 1) % L];
    Vec2d derivee(after.x - 2 * current.x + before.x, after.y - 2 * current.y + before.y);
    secondes[i % L] = derivee;
  }
  return secondes;
}

vector<double>
Polygon::computeCurvature() {
  int L = regularPoints.size();
  std::vector<double> curvature(L);
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

double
Polygon::getCurvature(int i) {
  return curvature.at(i);
}

Vec2d
Polygon::getNormale(int i) {
  return normales.at(i);
}

Point2d
Polygon::getPoint(int i) {
  return regularPoints.at(i);
}

vector<Point2d>
Polygon::deleteLoop() {
  int L = regularPoints.size();
  if(L < 2)
    return regularPoints;
  vector<Point2d> newPoints;
  Point2d p1, p2, q1, q2;
  int cut1, cut2;
  bool found = false;
  for(int i = 0; i < L; i++) {
    if(found)
      break;
    p1 = regularPoints[i % L];
    p2 = regularPoints[(i + 1) % L];
    for(int j = 0; j < L; j++) {
      if(found)
        break;
      if(j != i && j != (i + 1 % L)) {
        q1 = regularPoints[j % L];
        q2 = regularPoints[(j + 1) % L];
        if(segmentCut(p1, p2, q1, q2)) {
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
    cout << "Found loop: " << cut1 << ", " << cut2 << endl;
    // waitKey();
    if(cut2 < cut1) {
      cut2 += L;
    }
    for(int k = cut1; k <= cut2; k++) {
      newPoints.push_back(regularPoints[k % L]);
    }
    return newPoints;
  } else {
    return regularPoints;
  }
}

bool
segmentCut(Point2d p1, Point2d p2, Point2d q1, Point2d q2) {
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
    cout << "Tp: " << tp << endl;
    cout << "Tq: " << tq << endl;
    return true;
  }
  return false;
}

Polygon::~Polygon() {}
