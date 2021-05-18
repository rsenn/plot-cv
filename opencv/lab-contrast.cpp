#include <stdio.h>
#include <stdlib.h>
#include <opencv2/core/core.hpp>
#include <vector> // vector
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>

int
main(int argc, char** argv) {
  int clim = 4;
  if(argc == 3)
    clim = atoi(argv[2]);

  // READ RGB color image and convert cv::it to Lab
  cv::Mat bgr_image = cv::imread(argv[1]);
  cv::Mat lab_image;
  cv::cvtColor(bgr_image, lab_image, CV_BGR2Lab);

  // Extract the L channel
  std::vector<cv::Mat> lab_planes(3);
  cv::split(lab_image, lab_planes); // now we have the L image in lab_planes[0]

  // apply the CLAHE cv::algorithm to the L channel
  cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
  clahe->setClipLimit(clim);
  cv::Mat dst;
  clahe->apply(lab_planes[0], dst);

  // Merge the the color planes back into an Lab image
  dst.copyTo(lab_planes[0]);
  cv::merge(lab_planes, lab_image);

  // convert back to RGB
  cv::Mat image_clahe;
  cv::cvtColor(lab_image, image_clahe, CV_Lab2BGR);

  // display the results  (you might also want to see lab_planes[0] before and after).
  cv::imshow("image original", bgr_image);
  cv::imshow("image CLAHE", image_clahe);
  cv::imwrite("out.png", image_clahe);
  cv::waitKey();
}
