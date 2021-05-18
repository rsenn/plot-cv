#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <stdio.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/xfeatures2d.hpp>

//using namespace cv;
//using namespace cv::xfeatures2d;
using namespace std;

bool OVERLAY_ENABLED = true;
bool VALID_H = false;

void readme();
void SURFAlgorithm(cv::Mat& scene_img,
                   cv::Ptr<Feature2D>& surf,
                   vector<KeyPoint>& keypoints1,
                   vector<KeyPoint>& keypoints2,
                   cv::Mat& descriptors1,
                   cv::Mat& descriptors2,
                   cv::Mat& H,
                   bool& Hrel);
void drawBox(std::vector<cv::Point2f>& scene_corners, cv::Mat& img_show);
void getCameraData(std::string filename, cv::Mat& cameraMatrix2, cv::Mat& distCoeffs2);
void drawAxis(std::vector<cv::Point2f>& imagePoints, cv::Mat& img_show);

/* @function main */
int
main(int argc, char* argv[]) {
  if(argc < 2) {
    readme();
    return -1;
  }

  //-- Read objectImage ( the object to be "detected" )
  cv::Mat objectImage = cv::imread(argv[1]);
  cv::Mat gray_object;
  cv::cvtColor(objectImage, gray_object, cv::COLOR_BGR2GRAY);

  //-- Get the corners from the objectImage ( the object to be "detected" )
  vector<cv::Point2f> obj_corners(4);
  obj_corners[0] = cv::Point(0, 0);
  obj_corners[1] = cv::Point(objectImage.cols, 0);
  obj_corners[2] = cv::Point(objectImage.cols, objectImage.rows);
  obj_corners[3] = cv::Point(0, objectImage.rows);
  vector<cv::Point2f> scene_corners(4);
  vector<cv::Point3f> objectPoints(4);
  vector<cv::Point3f> axis(4);
  vector<cv::Point2f> imagePoints(4);

  //-- Create 3D corners from the objectImage ( the object to be "detected" )
  for(int i = 0; i < (int)obj_corners.size(); i++) { objectPoints[i] = cv::Point3f(obj_corners[i]); }

  //-- 3D object to render
  axis[0] = cv::Point3f(0.0, 0.0, 0.0);
  axis[1] = cv::Point3f(50.0, 0.0, 0.0);
  axis[2] = cv::Point3f(0.0, 50.0, 0.0);
  axis[3] = cv::Point3f(0.0, 0.0, -50.0);

  cv::VideoCapture cv::cap("/dev/video0"); // open the video file for reading
  if(!cv::cap.isOpened())              // if not success, exit program
  {
    cout << "Cannot open the video file" << endl;
    return -1;
  }

  cv::cap.set(cv::CAP_PROP_POS_MSEC, 300); // start the video at 300ms
  cv::cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
  cv::cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
  double fps = cv::cap.get(cv::CAP_PROP_FPS); // get the frames per seconds of the video
  cout << "Frame per seconds : " << fps << endl;
  cout << "Width : " << cv::cap.get(cv::CAP_PROP_FRAME_WIDTH) << endl;   // Width of the frames in the video stream.
  cout << "Height : " << cv::cap.get(cv::CAP_PROP_FRAME_HEIGHT) << endl; // Height of the frames in the video stream.
  cout << "Frame per seconds : " << fps << endl;
  cv::imshow("Target Object", objectImage);
  cv::namedWindow("MyVideo", cv::WINDOW_AUTOSIZE); // create a window called "MyVideo"

  // Load camera calibration data
  cv::Mat cameraMatrix, distCoeffs;
  string filename = "../out_camera_data.xml";
  getCameraData(filename, cameraMatrix, distCoeffs);
  cout << "camera matrix: " << cameraMatrix << endl << "distortion coeffs: " << distCoeffs << endl;

  // Homography matrix, translation and rotation vectors
  cv::Mat frame, H;
  cv::Mat rvec(3, 1, cv::DataType<double>::type);
  cv::Mat tvec(3, 1, cv::DataType<double>::type);

  //-- Detect objectImage keypoints and extract descriptors using FastFeatureDetector
  cv::Ptr<Feature2D> surf = SURF::create();
  vector<KeyPoint> keypoints1, keypoints2;
  cv::Mat descriptors1, descriptors2;
  surf->detectAndCompute(gray_object, cv::Mat(), keypoints1, descriptors1);

  while(1) {
    bool bSuccess = cv::cap.cv::read(frame); // cv::read a new frame from video
    if(!bSuccess)                    // if not success, break loop
    {
      cout << "Cannot cv::read the frame from video file" << endl;
      break;
    }

    // Compute Homography matrix H
    SURFAlgorithm(frame, surf, keypoints1, keypoints2, descriptors1, descriptors2, H, VALID_H);

    // Project object corners in the scene prespective
    perspectiveTransform(obj_corners, scene_corners, H);

    // Compute the perspective projection of the 3D object to be rendered in the scene
    cv::solvePnP(objectPoints, scene_corners, cameraMatrix, distCoeffs, rvec, tvec);

    // Project 3D object in the scene
    cv::projectPoints(axis, rvec, tvec, cameraMatrix, distCoeffs, imagePoints);

    // Display output
    if(VALID_H) {
      drawBox(scene_corners, frame);
      drawAxis(imagePoints, frame);
    }
    cv::imshow("MyVideo", frame); // show the frame in "MyVideo" window

    if(cv::waitKey(30) == 27) // wait for 'esc' key press for 30 ms. If 'esc' key is pressed, break loop
    {
      cout << "esc key is pressed by user" << endl;
      break;
    }
  }
  return 0;
}

/* @function readme */
void
readme() {
  std::cout << " Usage: ./rtTracker <target_img> " << std::endl;
}

void
drawBox(vector<cv::Point2f>& scene_corners, cv::Mat& img_show)
// Draw the contour of the detected object
{
  //-- Draw lines between the corners (the mapped object in the scene - image_2 )
  cv::line(img_show, scene_corners[0], scene_corners[1], cv::Scalar(255, 255, 0), 4);
  cv::line(img_show, scene_corners[1], scene_corners[2], cv::Scalar(255, 255, 0), 4);
  cv::line(img_show, scene_corners[2], scene_corners[3], cv::Scalar(255, 255, 0), 4);
  cv::line(img_show, scene_corners[3], scene_corners[0], cv::Scalar(255, 255, 0), 4);
}

void
drawAxis(vector<cv::Point2f>& axis, cv::Mat& img_show)
// Render 3D object
{
  //-- Draw lines between the corners (the mapped object in the scene - image_2 )
  cv::line(img_show, axis[0], axis[1], cv::Scalar(255, 0, 0), 5);
  cv::line(img_show, axis[0], axis[2], cv::Scalar(0, 255, 0), 5);
  cv::line(img_show, axis[0], axis[3], cv::Scalar(0, 0, 255), 5);
}

void
getCameraData(std::string filename, cv::Mat& cameraMatrix2, cv::Mat& distCoeffs2) {
  cv::FileStorage fs2(filename, cv::FileStorage::READ);

  fs2["camera_matrix"] >> cameraMatrix2;
  fs2["distortion_coefficients"] >> distCoeffs2;

  fs2.release();
}

void
SURFAlgorithm(cv::Mat& scene_img,
              cv::Ptr<Feature2D>& surf,
              vector<KeyPoint>& keypoints1,
              vector<KeyPoint>& keypoints2,
              cv::Mat& descriptors1,
              cv::Mat& descriptors2,
              cv::Mat& H,
              bool& VALID_H) {
  cv::Mat gray_scene, H1, outlier_mask;
  cv::cvtColor(scene_img, gray_scene, cv::COLOR_BGR2GRAY);
  if(!gray_scene.data) {
    std::cout << " --(!) cv::Error reading scene image " << std::endl;
  }

  //-- Step 1: Detect the keypoints and extract descriptors using FastFeatureDetector
  surf->detectAndCompute(scene_img, cv::Mat(), keypoints2, descriptors2);

  //-- Step 2: Find the closest matches between descriptors from the first image to the second
  BFMatcher matcher;
  vector<DMatch> matches;
  matcher.match(descriptors1, descriptors2, matches);

  //-- Localize the object
  std::vector<cv::Point2f> obj;
  std::vector<cv::Point2f> scene;
  for(size_t i = 0; i < matches.size(); i++) {
    //-- Get the keypoints from the good matches
    obj.push_back(keypoints1[matches[i].queryIdx].cv::pt);
    scene.push_back(keypoints2[matches[i].trainIdx].cv::pt);
  }
  H = cv::findHomography(obj, scene, cv::RANSAC, 3, outlier_mask);
  if(cv::sum(outlier_mask)[0] > 40) {
    VALID_H = true;
  } else {
    VALID_H = false;
  };
}
