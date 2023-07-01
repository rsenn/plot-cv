#include <opencv2/core/utility.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <cstring>
#include "samples_utility.hpp"

using namespace std;
// using namespace cv;

// prototype of the functino for feature extractor
void sobelExtractor(const cv::Mat img, const cv::Rect roi, cv::Mat& feat);

int
main(int argc, char** argv) {
  // show help
  if(argc < 2) {
    cout << " Usage: tracker <video_name>\n"
            " examples:\n"
            " example_tracking_kcf Bolt/img/%04d.jpg\n"
            " example_tracking_kcf faceocc2.webm\n"
         << endl;
    return 0;
  }

  // declares all required variables
  Rect2d roi;
  cv::Mat frame;

  //! [param]
  cv::TrackerKCF::Params param;
  param.desc_pca = cv::TrackerKCF::GRAY | cv::TrackerKCF::CN;
  param.desc_npca = 0;
  param.compress_feature = true;
  param.compressed_size = 2;
  //! [param]

  // create a tracker object
  //! [create]
  Ptr<cv::TrackerKCF> tracker = cv::TrackerKCF::create(param);
  //! [create]

  //! [setextractor]
  tracker->setFeatureExtractor(sobelExtractor);
  //! [setextractor]

  // set input video
  std::string video = argv[1];
  cv::VideoCapture cap(video);

  // get bounding box
  cap >> frame;
  roi = cv::selectROI("tracker", frame);

  // quit if ROI was not selected
  if(roi.width == 0 || roi.height == 0)
    return 0;

  // initialize the tracker
  tracker->init(frame, roi);

  // perform the tracking process
  printf("Start the tracking process, press ESC to quit.\n");
  for(;;) {
    // get frame from the video
    cap >> frame;

    // stop the program if no more images
    if(frame.rows == 0 || frame.cols == 0)
      break;

    // update the tracking result
    tracker->update(frame, roi);

    // draw the tracked object
    cv::rectangle(frame, roi, cv::Scalar(255, 0, 0), 2, 1);

    // show image with the tracked object
    cv::imshow("tracker", frame);

    // quit on ESC button
    if(cv::waitKey(1) == 27)
      break;
  }

  return 0;
}

void
sobelExtractor(const cv::Mat img, const cv::Rect roi, cv::Mat& feat) {
  cv::Mat sobel[2];
  cv::Mat patch;
  cv::Rect region = roi;

  //! [insideimage]
  // extract patch inside the image
  if(roi.x < 0) {
    region.x = 0;
    region.width += roi.x;
  }
  if(roi.y < 0) {
    region.y = 0;
    region.height += roi.y;
  }
  if(roi.x + roi.width > img.cols)
    region.width = img.cols - roi.x;
  if(roi.y + roi.height > img.rows)
    region.height = img.rows - roi.y;
  if(region.width > img.cols)
    region.width = img.cols;
  if(region.height > img.rows)
    region.height = img.rows;
  //! [insideimage]

  patch = img(region).clone();
  cv::cvtColor(patch, patch, COLOR_BGR2GRAY);

  //! [padding]
  // cv::add some padding to compensate when the patch is outside image border
  int addTop, addBottom, addLeft, addRight;
  addTop = region.y - roi.y;
  addBottom = (roi.height + roi.y > img.rows ? roi.height + roi.y - img.rows : 0);
  addLeft = region.x - roi.x;
  addRight = (roi.width + roi.x > img.cols ? roi.width + roi.x - img.cols : 0);

  cv::copyMakeBorder(patch, patch, addTop, addBottom, addLeft, addRight, BORDER_REPLICATE);
  //! [padding]

  //! [sobel]
  cv::Sobel(patch, sobel[0], CV_32F, 1, 0, 1);
  cv::Sobel(patch, sobel[1], CV_32F, 0, 1, 1);

  cv::merge(sobel, 2, feat);
  //! [sobel]

  //! [postprocess]
  feat.convertTo(feat, CV_64F);
  feat = feat / 255.0 - 0.5; // cv::normalize to range -0.5 .. 0.5
  //! [postprocess]
}
