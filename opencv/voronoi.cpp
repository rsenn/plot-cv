#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;

// Draw a single point
static void
draw_point(cv::Mat& img, cv::Point2f fp, cv::Scalar color) {
  cv::circle(img, fp, 2, color, cv::FILLED, cv::LINE_AA, 0);
}

// Draw delaunay triangles
static void
draw_delaunay(cv::Mat& img, cv::Subdiv2D& subdiv, cv::Scalar delaunay_color) {

  std::vector<cv::Vec6f> triangleList;
  subdiv.getTriangleList(triangleList);
  std::vector<cv::Point> pt(3);
  cv::Size size = img.size();
  cv::Rect rect(0, 0, size.width, size.height);

  for(size_t i = 0; i < triangleList.size(); i++) {
    cv::Vec6f t = triangleList[i];
    pt[0] = cv::Point(cvRound(t[0]), cvRound(t[1]));
    pt[1] = cv::Point(cvRound(t[2]), cvRound(t[3]));
    pt[2] = cv::Point(cvRound(t[4]), cvRound(t[5]));

    // Draw rectangles completely inside the image.
    if(rect.contains(pt[0]) && rect.contains(pt[1]) && rect.contains(pt[2])) {
      cv::line(img, pt[0], pt[1], delaunay_color, 1, cv::LINE_AA, 0);
      cv::line(img, pt[1], pt[2], delaunay_color, 1, cv::LINE_AA, 0);
      cv::line(img, pt[2], pt[0], delaunay_color, 1, cv::LINE_AA, 0);
    }
  }
}

// Draw voronoi diagram
static void
draw_voronoi(cv::Mat& img, cv::Subdiv2D& subdiv) {
  std::vector<vector<cv::Point2f>> facets;
  std::vector<cv::Point2f> centers;
  subdiv.getVoronoiFacetList(std::vector<int>(), facets, centers);

  std::vector<cv::Point> ifacet;
  std::vector<vector<cv::Point>> ifacets(1);

  for(size_t i = 0; i < facets.size(); i++) {
    ifacet.resize(facets[i].size());
    for(size_t j = 0; j < facets[i].size(); j++) ifacet[j] = facets[i][j];

    cv::Scalar color;
    color[0] = rand() & 255;
    color[1] = rand() & 255;
    color[2] = rand() & 255;
    cv::fillConvexPoly(img, ifacet, color, 8, 0);

    ifacets[0] = ifacet;
    cv::polylines(img, ifacets, true, cv::Scalar(), 1, cv::LINE_AA, 0);
    cv::circle(img, centers[i], 3, cv::Scalar(), cv::FILLED, cv::LINE_AA, 0);
  }
}

int
main(int argc, char** argv) {

  // Define window names
  std::string win_delaunay = "Delaunay Triangulation";
  std::string win_voronoi = "Voronoi Diagram";

  // Turn on animation while drawing triangles
  bool animate = true;

  // Define colors for drawing.
  cv::Scalar delaunay_color(255, 255, 255), points_color(0, 0, 255);

  // Read in the image.
  cv::Mat img = imread("image.jpg");

  // Keep a copy around
  cv::Mat img_orig = img.clone();

  // Rectangle to be used with cv::Subdiv2D
  cv::Size size = img.size();
  cv::Rect rect(0, 0, size.width, size.height);

  // Create an instance of cv::Subdiv2D
  cv::Subdiv2D subdiv(rect);

  // Create a std::vector of points.
  std::vector<cv::Point2f> points;

  // Read in the points from a text file
  std::ifstream ifs("points.txt");
  int x, y;
  while(ifs >> x >> y) { points.push_back(cv::Point2f(x, y)); }

  // Insert points into subdiv
  for(std::vector<cv::Point2f>::iterator it = points.begin(); it != points.end(); it++) {
    subdiv.insert(*it);
    // Show animation
    if(animate) {
      cv::Mat img_copy = img_orig.clone();
      // Draw delaunay triangles
      draw_delaunay(img_copy, subdiv, delaunay_color);
      cv::imshow(win_delaunay, img_copy);
      cv::waitKey(100);
    }
  }

  // Draw delaunay triangles
  draw_delaunay(img, subdiv, delaunay_color);

  // Draw points
  for(std::vector<cv::Point2f>::iterator it = points.begin(); it != points.end(); it++) { draw_point(img, *it, points_color); }

  // Allocate space for Voronoi Diagram
  cv::Mat img_voronoi = Mat::zeros(img.rows, img.cols, CV_8UC3);

  // Draw Voronoi diagram
  draw_voronoi(img_voronoi, subdiv);

  // Show results.
  cv::imshow(win_delaunay, img);
  cv::imshow(win_voronoi, img_voronoi);
  cv::waitKey(0);

  return 0;
}
