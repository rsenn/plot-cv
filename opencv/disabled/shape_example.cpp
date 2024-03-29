/*
 * shape_context.cpp -- Shape context demo for shape matching
 */

#include <opencv2/shape.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/utility.hpp>
#include <iostream>
#include <string>

using namespace std;
// using namespace cv;

static void
help() {
  printf("\n"
         "This program demonstrates a method for shape comparisson based on Shape Context\n"
         "You should run the program providing a number between 1 and 20 for selecting an image in "
         "the folder "
         "../data/shape_sample.\n"
         "Call\n"
         "./shape_example [number between 1 and 20, 1 default]\n\n");
}

static std::vector<cv::Point>
simpleContour(const cv::Mat& currentQuery, int n = 300) {
  std::vector<std::vector<cv::Point>> _contoursQuery;
  std::vector<cv::Point> contoursQuery;
  cv::findContours(currentQuery, _contoursQuery, RETR_LIST, CHAIN_APPROX_NONE);
  for(size_t border = 0; border < _contoursQuery.size(); border++) {
    for(size_t p = 0; p < _contoursQuery[border].size(); p++) { contoursQuery.push_back(_contoursQuery[border][p]); }
  }

  // In case actual number of points is less than n
  int dummy = 0;
  for(int cv::add = (int)contoursQuery.size() - 1; cv::add < n; cv::add++) {
    contoursQuery.push_back(contoursQuery[dummy++]); // adding dummy values
  }

  // Uniformly sampling
  random_shuffle(contoursQuery.begin(), contoursQuery.end());
  std::vector<cv::Point> cont;
  for(int i = 0; i < n; i++) { cont.push_back(contoursQuery[i]); }
  return cont;
}

int
main(int argc, char** argv) {
  string path = "../data/shape_sample/";
  cv::CommandLineParser parser(argc, argv, "{help h||}{@input|1|}");
  if(parser.has("help")) {
    help();
    return 0;
  }
  int indexQuery = parser.get<int>("@input");
  if(!parser.check()) {
    parser.printErrors();
    help();
    return 1;
  }
  if(indexQuery < 1 || indexQuery > 20) {
    help();
    return 1;
  }
  cv::Ptr<cv::ShapeContextDistanceExtractor> mysc = cv::createShapeContextDistanceExtractor();

  cv::Size sz2Sh(300, 300);
  stringstream queryName;
  queryName << path << indexQuery << ".png";
  cv::Mat query = cv::imread(queryName.str(), IMREAD_GRAYSCALE);
  cv::Mat queryToShow;
  cv::resize(query, queryToShow, sz2Sh);
  cv::imshow("QUERY", queryToShow);
  cv::moveWindow("TEST", 0, 0);
  std::vector<cv::Point> contQuery = simpleContour(query);
  int bestcv::Match = 0;
  float bestDis = FLT_MAX;
  for(int ii = 1; ii <= 20; ii++) {
    if(ii == indexQuery)
      continue;
    cv::waitKey(30);
    stringstream iiname;
    iiname << path << ii << ".png";
    cout << "name: " << iiname.str() << endl;
    cv::Mat iiIm = cv::imread(iiname.str(), 0);
    cv::Mat iiToShow;
    cv::resize(iiIm, iiToShow, sz2Sh);
    cv::imshow("TEST", iiToShow);
    cv::moveWindow("TEST", sz2Sh.width + 50, 0);
    std::vector<cv::Point> contii = simpleContour(iiIm);
    float dis = mysc->computeDistance(contQuery, contii);
    if(dis < bestDis) {
      bestcv::Match = ii;
      bestDis = dis;
    }
    std::cout << " distance between " << queryName.str() << " and " << iiname.str() << " is: " << dis << std::endl;
  }
  cv::destroyWindow("TEST");
  stringstream bestname;
  bestname << path << bestcv::Match << ".png";
  cv::Mat iiIm = cv::imread(bestname.str(), 0);
  cv::Mat bestToShow;
  cv::resize(iiIm, bestToShow, sz2Sh);
  cv::imshow("BEST MATCH", bestToShow);
  cv::moveWindow("BEST MATCH", sz2Sh.width + 50, 0);

  return 0;
}
