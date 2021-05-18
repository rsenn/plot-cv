#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

//using namespace cv;
using namespace std;

static void
help(char** argv) {
  cout << "\nThis program demonstrates iterative construction of\n"
          "delaunay triangulation and voronoi tessellation.\n"
          "It draws a random set of points in an image and then delaunay triangulates them.\n"
          "Usage: \n";
  cout << argv[0];
  cout << "\n\nThis program builds the triangulation interactively, you may stop this process by\n"
          "hitting any key.\n";
}

static void
draw_subdiv_point(cv::Mat& img, cv::Point2f fp, cv::Scalar color) {
  cv::circle(img, fp, 3, color, FILLED, cv::LINE_8, 0);
}

static void
draw_subdiv(cv::Mat& img, cv::Subdiv2D& subdiv, cv::Scalar delaunay_color) {
#if 1
  vector<Vec6f> triangleList;
  subdiv.getTriangleList(triangleList);
  vector<cv::Point> pt(3);

  for(size_t i = 0; i < triangleList.size(); i++) {
    Vec6f t = triangleList[i];
    pt[0] = cv::Point(cvRound(t[0]), cvRound(t[1]));
    pt[1] = cv::Point(cvRound(t[2]), cvRound(t[3]));
    pt[2] = cv::Point(cvRound(t[4]), cvRound(t[5]));
    cv::line(img, pt[0], pt[1], delaunay_color, 1, cv::LINE_AA, 0);
    cv::line(img, pt[1], pt[2], delaunay_color, 1, cv::LINE_AA, 0);
    cv::line(img, pt[2], pt[0], delaunay_color, 1, cv::LINE_AA, 0);
  }
#else
  vector<cv::Vec4f> edgeList;
  subdiv.getEdgeList(edgeList);
  for(size_t i = 0; i < edgeList.size(); i++) {
    cv::Vec4f e = edgeList[i];
    cv::Point pt0 = cv::Point(cvRound(e[0]), cvRound(e[1]));
    cv::Point pt1 = cv::Point(cvRound(e[2]), cvRound(e[3]));
    cv::line(img, pt0, pt1, delaunay_color, 1, cv::LINE_AA, 0);
  }
#endif
}

static void
locate_point(cv::Mat& img, cv::Subdiv2D& subdiv, cv::Point2f fp, cv::Scalar active_color) {
  int e0 = 0, vertex = 0;

  subdiv.locate(fp, e0, vertex);

  if(e0 > 0) {
    int e = e0;
    do {
      cv::Point2f org, dst;
      if(subdiv.edgeOrg(e, &org) > 0 && subdiv.edgeDst(e, &dst) > 0)
        cv::line(img, org, dst, active_color, 3, cv::LINE_AA, 0);

      e = subdiv.getEdge(e, cv::Subdiv2D::NEXT_AROUND_LEFT);
    } while(e != e0);
  }

  draw_subdiv_point(img, fp, active_color);
}

static void
paint_voronoi(cv::Mat& img, cv::Subdiv2D& subdiv) {
  vector<vector<cv::Point2f>> facets;
  vector<cv::Point2f> centers;
  subdiv.getVoronoiFacetList(vector<int>(), facets, centers);

  vector<cv::Point> ifacet;
  vector<vector<cv::Point>> ifacets(1);

  for(size_t i = 0; i < facets.size(); i++) {
    ifacet.cv::resize(facets[i].size());
    for(size_t j = 0; j < facets[i].size(); j++) ifacet[j] = facets[i][j];

    cv::Scalar color;
    color[0] = rand() & 255;
    color[1] = rand() & 255;
    color[2] = rand() & 255;
    cv::fillConvexPoly(img, ifacet, color, 8, 0);

    ifacets[0] = ifacet;
    cv::polylines(img, ifacets, true, cv::Scalar(), 1, cv::LINE_AA, 0);
    cv::circle(img, centers[i], 3, cv::Scalar(), FILLED, cv::LINE_AA, 0);
  }
}

int
main(int argc, char** argv) {
  cv::CommandLineParser parser(argc, argv, "{help h||}");
  if(parser.has("help")) {
    help(argv);
    return 0;
  }

  cv::Scalar active_facet_color(0, 0, 255), delaunay_color(255, 255, 255);
  cv::Rect rect(0, 0, 600, 600);

  cv::Subdiv2D subdiv(rect);
  cv::Mat img(rect.size(), CV_8UC3);

  img = cv::Scalar::all(0);
  string win = "Delaunay Demo";
  cv::imshow(win, img);

  for(int i = 0; i < 200; i++) {
    cv::Point2f fp((float)(rand() % (rect.width - 10) + 5), (float)(rand() % (rect.height - 10) + 5));

    locate_point(img, subdiv, fp, active_facet_color);
    cv::imshow(win, img);

    if(cv::waitKey(100) >= 0)
      break;

    subdiv.insert(fp);

    img = cv::Scalar::all(0);
    draw_subdiv(img, subdiv, delaunay_color);
    cv::imshow(win, img);

    if(cv::waitKey(100) >= 0)
      break;
  }

  img = cv::Scalar::all(0);
  paint_voronoi(img, subdiv);
  cv::imshow(win, img);

  cv::waitKey(0);

  return 0;
}
