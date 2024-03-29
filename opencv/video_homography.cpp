/*
 * video_homography.cpp
 *
 *  Created on: Oct 18, 2010
 *      Author: erublee
 */

#include <iostream>
#include <opencv2/opencv_modules.hpp>

#ifdef HAVE_OPENCV_CALIB3D

#include <opencv2/calib3d.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <list>
#include <vector>

using namespace std;
// using namespace cv;
// using namespace cv::xfeatures2d;

static void
help(char** av) {
  cout << "\nThis program demonstrated the use of features2d with the Fast corner detector and brief "
          "descriptors\n"
       << "to track planar objects by computing their homography from the key (training) image to the "
          "query (test) "
          "image\n\n"
       << endl;
  cout << "usage: " << av[0] << " <video device number>\n" << endl;
  cout << "The following keys do stuff:" << endl;
  cout << "  t : grabs a reference frame to match against" << endl;
  cout << "  l : makes the reference frame new every frame" << endl;
  cout << "  q or escape: quit" << endl;
}

namespace {
void
drawMatchesRelative(const vector<cv::KeyPoint>& train,
                    const vector<cv::KeyPoint>& query,
                    std::vector<cv::DMatch>& matches,
                    cv::Mat& img,
                    const vector<unsigned char>& mask = vector<unsigned char>()) {
  for(int i = 0; i < (int)matches.size(); i++) {
    if(mask.empty() || mask[i]) {
      cv::Point2f pt_new = query[matches[i].queryIdx].pt;
      cv::Point2f pt_old = train[matches[i].trainIdx].pt;

      cv::line(img, pt_new, pt_old, cv::Scalar(125, 255, 125), 1);
      cv::circle(img, pt_new, 2, cv::Scalar(255, 0, 125), 1);
    }
  }
}

// Takes a descriptor and turns cv::it into an xy point
void
keypoints2points(const vector<cv::KeyPoint>& in, vector<cv::Point2f>& out) {
  out.clear();
  out.reserve(in.size());
  for(size_t i = 0; i < in.size(); ++i) { out.push_back(in[i].pt); }
}

// Takes an xy point and appends that to a keypoint structure
void
points2keypoints(const vector<cv::Point2f>& in, vector<cv::KeyPoint>& out) {
  out.clear();
  out.reserve(in.size());
  for(size_t i = 0; i < in.size(); ++i) { out.push_back(cv::KeyPoint(in[i], 1)); }
}

// Uses computed homography H to warp original input points to new planar position
void
warpKeypoints(const cv::Mat& H, const vector<cv::KeyPoint>& in, vector<cv::KeyPoint>& out) {
  vector<cv::Point2f> pts;
  keypoints2points(in, pts);
  vector<cv::Point2f> pts_w(pts.size());
  cv::Mat m_pts_w(pts_w);
  perspectiveTransform(cv::Mat(pts), m_pts_w, H);
  points2keypoints(pts_w, out);
}

// Converts matching indices to xy points
void
matches2points(const vector<cv::KeyPoint>& train,
               const vector<cv::KeyPoint>& query,
               const std::vector<cv::DMatch>& matches,
               std::vector<cv::Point2f>& pts_train,
               std::vector<cv::Point2f>& pts_query) {

  pts_train.clear();
  pts_query.clear();
  pts_train.reserve(matches.size());
  pts_query.reserve(matches.size());

  size_t i = 0;

  for(; i < matches.size(); i++) {

    const DMatch& dmatch = matches[i];

    pts_query.push_back(query[dmatch.queryIdx].pt);
    pts_train.push_back(train[dmatch.trainIdx].pt);
  }
}

void
resetH(cv::Mat& H) {
  H = cv::Mat::eye(3, 3, CV_32FC1);
}
} // namespace

int
main(int ac, char** av) {

  if(ac != 2) {
    help(av);
    return 1;
  }

  cv::Ptr<BriefDescriptorExtractor> brief = BriefDescriptorExtractor::create(32);

  cv::VideoCapture capture;
  capture.open(atoi(av[1]));
  if(!capture.isOpened()) {
    help(av);
    cout << "capture device " << atoi(av[1]) << " failed to open!" << endl;
    return 1;
  }

  cout << "following keys do stuff:" << endl;
  cout << "t : grabs a reference frame to match against" << endl;
  cout << "l : makes the reference frame new every frame" << endl;
  cout << "q or escape: quit" << endl;

  cv::Mat frame;

  vector<DMatch> matches;

  BFMatcher desc_matcher(brief->defaultNorm());

  vector<cv::Point2f> train_pts, query_pts;
  vector<cv::KeyPoint> train_kpts, query_kpts;
  vector<unsigned char> match_mask;

  cv::Mat gray;

  bool ref_live = true;

  cv::Mat train_desc, query_desc;
  cv::Ptr<FastFeatureDetector> detector = FastFeatureDetector::create(10, true);

  cv::Mat H_prev = cv::Mat::eye(3, 3, CV_32FC1);
  for(;;) {
    capture >> frame;
    if(frame.empty())
      break;

    cv::cvtColor(frame, gray, cv::COLOR_RGB2GRAY);

    detector->detect(gray, query_kpts);           // Find interest points
    brief->compute(gray, query_kpts, query_desc); // Compute brief descriptors at each keypoint location

    if(!train_kpts.empty()) {

      vector<cv::KeyPoint> test_kpts;
      warpKeypoints(H_prev.inv(), query_kpts, test_kpts);

      // cv::Mat mask = windowedMatchingMask(test_kpts, train_kpts, 25, 25);
      desc_matcher.match(query_desc, train_desc, matches, cv::Mat());
      drawKeypoints(frame, test_kpts, frame, cv::Scalar(255, 0, 0), DrawMatchesFlags::DRAW_OVER_OUTIMG);

      matches2points(train_kpts, query_kpts, matches, train_pts, query_pts);

      if(matches.size() > 5) {
        cv::Mat H = cv::findHomography(train_pts, query_pts, cv::RANSAC, 4, match_mask);
        if(cv::countNonZero(cv::Mat(match_mask)) > 15) {
          H_prev = H;
        } else
          resetH(H_prev);
        drawMatchesRelative(train_kpts, query_kpts, matches, frame, match_mask);
      } else
        resetH(H_prev);

    } else {
      H_prev = cv::Mat::eye(3, 3, CV_32FC1);
      cv::Mat out;
      drawKeypoints(gray, query_kpts, out);
      frame = out;
    }

    cv::imshow("frame", frame);

    if(ref_live) {
      train_kpts = query_kpts;
      query_desc.copyTo(train_desc);
    }
    char key = (char)cv::waitKey(2);
    switch(key) {
      case 'l':
        ref_live = true;
        resetH(H_prev);
        break;
      case 't':
        ref_live = false;
        train_kpts = query_kpts;
        query_desc.copyTo(train_desc);
        resetH(H_prev);
        break;
      case 27:
      case 'q': return 0; break;
    }
  }
  return 0;
}

#else

int
main() {
  std::cerr << "OpenCV was built without calib3d module" << std::endl;
  return 0;
}

#endif
