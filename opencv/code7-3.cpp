// Program to count the number of objects using morphology operations and the cv::watershed transform
// Author: Samarth Manoj Brahmbhatt, University of Pennsylvania

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>

// using namespace cv;
using namespace std;

class objectCounter {
private:
  cv::Mat image, gray, markers, output;
  int count;

public:
  objectCounter(cv::Mat); // constructor
  void get_markers();     // function to get markers for cv::watershed segmentation
  int count_objects();    // function to implement cv::watershed segmentation and count catchment basins
};

objectCounter::objectCounter(cv::Mat _image) {
  image = _image.clone();
  cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  cv::imshow("image", image);
}

void
objectCounter::get_markers() {
  // equalize histogram of image to improve contrast
  cv::Mat im_e;
  cv::equalizeHist(gray, im_e);
  // cv::imshow("im_e", im_e);

  // cv::dilate to remove small black spots
  cv::Mat strel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(9, 9));
  cv::Mat im_d;
  cv::dilate(im_e, im_d, strel);
  // cv::imshow("im_d", im_d);

  // open and close to highlight objects
  strel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(19, 19));
  cv::Mat im_oc;
  cv::morphologyEx(im_d, im_oc, cv::MORPH_OPEN, strel);
  cv::morphologyEx(im_oc, im_oc, cv::MORPH_CLOSE, strel);
  // cv::imshow("im_oc", im_oc);

  // adaptive cv::threshold to create binary image
  cv::Mat th_a;
  cv::adaptiveThreshold(im_oc, th_a, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 105, 0);
  // cv::imshow("th_a", th_a);

  // cv::erode binary image twice to separate regions
  cv::Mat th_e;
  cv::erode(th_a, th_e, strel, cv::Point(-1, -1), 2);
  // cv::imshow("th_e", th_e);

  std::vector<std::vector<cv::Point>> c, contours;
  std::vector<cv::Vec4i> heirarchy;
  cv::findContours(th_e, c, heirarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);

  // remove very small contours
  for(int idx = 0; idx >= 0; idx = heirarchy[idx][0])
    if(cv::contourArea(c[idx]) > 20)
      contours.push_back(c[idx]);

  cout << "Extracted " << contours.size() << " contours" << endl;

  count = contours.size();
  markers.create(image.rows, image.cols, CV_32SC1);
  for(int idx = 0; idx < contours.size(); idx++) cv::drawContours(markers, contours, idx, cv::Scalar::all(idx + 1), -1, 8);
}

int
objectCounter::count_objects() {
  cv::watershed(image, markers);

  // colors generated randomly to make the output look pretty
  std::vector<cv::Vec3b> colorTab;
  for(int i = 0; i < count; i++) {
    int b = cv::theRNG().uniform(0, 255);
    int g = cv::theRNG().uniform(0, 255);
    int r = cv::theRNG().uniform(0, 255);

    colorTab.push_back(cv::Vec3b((uchar)b, (uchar)g, (uchar)r));
  }

  // cv::watershed output image
  cv::Mat wshed(markers.size(), CV_8UC3);

  // paint the cv::watershed output image
  for(int i = 0; i < markers.rows; i++)
    for(int j = 0; j < markers.cols; j++) {
      int index = markers.at<int>(i, j);
      if(index == -1)
        wshed.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 255, 255);
      else if(index <= 0 || index > count)
        wshed.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);
      else
        wshed.at<cv::Vec3b>(i, j) = colorTab[index - 1];
    }

  // superimpose the cv::watershed image with 50% transparence on the grayscale original image
  cv::Mat imgGray;
  cv::cvtColor(gray, imgGray, cv::COLOR_GRAY2BGR);
  wshed = wshed * 0.5 + imgGray * 0.5;
  cv::imshow("Segmentation", wshed);

  return count;
}

int
main() {
  cv::Mat im = cv::imread("fruit.jpg");

  objectCounter oc(im);
  oc.get_markers();

  int count = oc.count_objects();

  cout << "Counted " << count << " fruits." << endl;

  while(char(cv::waitKey(1)) != 'q') {}

  return 0;
}
