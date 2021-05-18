// Program to illustrate SIFT keypoint and descriptor extraction, and matching using FLANN
// Author: Samarth Manoj Brahmbhatt, University of Pennsylvania

#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/features2d/features2d.hpp>

//using namespace cv;
using namespace std;

int
main() {
  cv::Mat train = cv::imread("template.jpg"), train_g;
  cv::cvtColor(train, train_g, cv::COLOR_BGR2GRAY);

  // detect SIFT keypoints and extract descriptors in the train image
  vector<KeyPoint> train_kp;
  cv::Mat train_desc;

  SiftFeatureDetector featureDetector;
  featureDetector.detect(train_g, train_kp);
  SiftDescriptorExtractor featureExtractor;
  featureExtractor.compute(train_g, train_kp, train_desc);

  // FLANN based descriptor matcher object
  FlannBasedMatcher matcher;
  vector<cv::Mat> train_desc_collection(1, train_desc);
  matcher.cv::add(train_desc_collection);
  matcher.train();

  // cv::VideoCapture object
  cv::VideoCapture cap(0);

  unsigned int frame_count = 0;

  while(char(cv::waitKey(1)) != 'q') {
    double t0 = cv::getTickCount();
    cv::Mat test, test_g;
    cap >> test;
    if(test.empty())
      continue;

    cv::cvtColor(test, test_g, cv::COLOR_BGR2GRAY);

    // detect SIFT keypoints and extract descriptors in the test image
    vector<KeyPoint> test_kp;
    cv::Mat test_desc;
    featureDetector.detect(test_g, test_kp);
    featureExtractor.compute(test_g, test_kp, test_desc);

    // match train and test descriptors, getting 2 nearest neighbors for all test descriptors
    vector<vector<DMatch>> matches;
    matcher.knnMatch(test_desc, matches, 2);

    // filter for good matches according to Lowe's algorithm
    vector<DMatch> good_matches;
    for(int i = 0; i < matches.size(); i++) {
      if(matches[i][0].distance < 0.6 * matches[i][1].distance)
        good_matches.push_back(matches[i][0]);
    }

    cv::Mat img_show;
    drawMatches(test, test_kp, train, train_kp, good_matches, img_show);
    cv::imshow("Matches", img_show);

    cout << "Frame rate = " << cv::getTickFrequency() / (cv::getTickCount() - t0) << endl;
  }

  return 0;
}
