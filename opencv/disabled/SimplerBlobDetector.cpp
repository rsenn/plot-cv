// ------------------------------------------------------------------------------------------------
// SimplerBlobDetector.cpp
// Joost van Stuijvenberg
// Avans Hogeschool Breda
// August 2018
//
// Simplified version of OpenCV's cv::SimpleBlobDetector, for educational purposes. The code obtained
// from https://github.com/opencv/opencv/blob/master/modules/features2d/src/blobdetector.cpp has
// been stripped from all unnecessary details and augmented with comments. Furthermore, it was
// converted into a standalone application.
// ------------------------------------------------------------------------------------------------

#include <iostream>
#include <cassert>

#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//using namespace cv;
using namespace std;

// ------------------------------------------------------------------------------------------------
// Center struct, as used in findBlobs().
// ------------------------------------------------------------------------------------------------
struct Center {
  cv::Point2d location;
  double radius;
  double confidence;
};

// ------------------------------------------------------------------------------------------------
// Globals.
// ------------------------------------------------------------------------------------------------
cv::SimpleBlobDetector::Params params;
int window = 0;

// ------------------------------------------------------------------------------------------------
// onMouseClick() - Mouse click callback handler.
// ------------------------------------------------------------------------------------------------
// PRE  : event contains the mouse event type
// PRE  : x and y contain the click location
// PRE  : userdata may be a reference to the cv::Mat object that caught the mouse click
// POST : grayvalue of the clicked pixel is written to console output
// ------------------------------------------------------------------------------------------------
void
onMouseClick(int event, int x, int y, int flags, void* userdata) {
  if(event == cv::EVENT_LBUTTONDOWN) {
    assert(userdata != nullptr);
    assert(x > 0 && y > 0 && x < ((cv::Mat*)userdata)->size().width && y < ((cv::Mat*)userdata)->size().height);

    cv::Mat image(*(cv::Mat*)userdata);
    int gr = image.at<uchar>(y, x);
    cout << "Grayvalue = " << gr << endl;
  }
}

// ------------------------------------------------------------------------------------------------
// findBlobs() - Uses the cv::findContours()-function to find contours in the binary image.
// ------------------------------------------------------------------------------------------------
// PRE  : _binaryImage contains a valid binary image
// PRE  : centers is a reference to a std::vector of Center instances
// POST : centers contains Center objects for each blob that was found
// ------------------------------------------------------------------------------------------------
void
findBlobs(InputArray _image, InputArray _binaryImage, std::vector<Center>& centers, int cv::threshold) {
  cv::Scalar contourColor = cv::Scalar(255, 0, 0);                 // Blue
  cv::Scalar filteredByAreaColor = cv::Scalar(0, 255, 0);          // Green
  cv::Scalar filteredByCircularityColor = cv::Scalar(255, 255, 0); // Cyan
  cv::Scalar filteredByInertiaColor = cv::Scalar(0, 255, 255);     // Yellow
  cv::Scalar filteredByConvexityColor = cv::Scalar(255, 0, 255);
  cv::Scalar filteredByColorColor = cv::Scalar(255, 255, 255); // White
  cv::Scalar selectedColor = cv::Scalar(0, 0, 255);            // Red

  cv::Mat image = _image.getcv::Mat(), binaryImage = _binaryImage.getcv::Mat();
  centers.clear();

  // Find contours in the binary image using the cv::findContours()-function. Let this function
  // return a list of contours only (no hierarchical data).
  std::vector<std::vector<cv::Point>> contours;
  cv::Mat tmpBinaryImage = binaryImage.clone();
  cv::findContours(tmpBinaryImage, contours, RETR_LIST, CHAIN_APPROX_NONE);

  // Now process all the contours that were found.
  cv::Mat inter;
  cv::cvtColor(binaryImage, inter, cv::COLOR_GRAY2BGR);
  for(size_t contourIdx = 0; contourIdx < contours.size(); contourIdx++) {
    Center center;
    center.confidence = 1;
    cv::Moments moms = cv::moments(cv::Mat(contours[contourIdx]));

    // Show the current contour in blue, initially.
    cv::drawContours(inter, contours, contourIdx, contourColor, 3);
    cv::drawContours(inter, contours, contourIdx, 3);

    // If filtering by area is requested, see if the area of the current contour
    // is within the specified min and max limits, excluding min and including max.
    if(params.filterByArea) {
      double area = moms.m00;
      if(area < params.minArea || area >= params.maxArea) {
        cv::drawContours(inter, contours, contourIdx, filteredByAreaColor, 3);
        continue;
      }
    }

    // If filtering by circularity is requested, see if the current contour has a
    // sufficient circularity.
    if(params.filterByCircularity) {
      double area = moms.m00;
      double perimeter = cv::arcLength(cv::Mat(contours[contourIdx]), true);
      double ratio = 4 * CV_PI * area / (perimeter * perimeter);
      if(ratio < params.minCircularity || ratio >= params.maxCircularity) {
        cv::drawContours(inter, contours, contourIdx, filteredByCircularityColor, 3);
        continue;
      }
    }

    // If filtering by inertia is requested,
    if(params.filterByInertia) {
      double denominator = std::sqrt(std::cv::pow(2 * moms.mu11, 2) + std::pow(moms.mu20 - moms.mu02, 2));
      const double eps = 1e-2;
      double ratio;
      if(denominator > eps) {
        double cosmin = (moms.mu20 - moms.mu02) / denominator;
        double sinmin = 2 * moms.mu11 / denominator;
        double cosmax = -cosmin;
        double sinmax = -sinmin;

        double imin = 0.5 * (moms.mu20 + moms.mu02) - 0.5 * (moms.mu20 - moms.mu02) * cosmin - moms.mu11 * sinmin;
        double imax = 0.5 * (moms.mu20 + moms.mu02) - 0.5 * (moms.mu20 - moms.mu02) * cosmax - moms.mu11 * sinmax;
        ratio = imin / imax;
      } else
        ratio = 1;

      if(ratio < params.minInertiaRatio || ratio >= params.maxInertiaRatio) {
        cv::drawContours(inter, contours, contourIdx, filteredByInertiaColor, 3);
        continue;
      }

      center.confidence = ratio * ratio;
    }

    // If filtering by convexity is requested, skip this contour if the ratio between the contour
    // area and the hull area is not within the specified limits.
    if(params.filterByConvexity) {
      std::vector<cv::Point> hull;
      cv::convexHull(cv::Mat(contours[contourIdx]), hull);
      double area = cv::contourArea(cv::Mat(contours[contourIdx]));
      double hullArea = cv::contourArea(cv::Mat(hull));
      double ratio = area / hullArea;
      if(ratio < params.minConvexity || ratio >= params.maxConvexity) {
        cv::drawContours(inter, contours, contourIdx, filteredByConvexityColor, 3);
        continue;
      }
    }

    // Prevent division by zero, should this contour have no area.
    if(moms.m00 == 0.0)
      continue;
    center.location = cv::Point2d(moms.m10 / moms.m00, moms.m01 / moms.m00);

    // If filtering by color was requested, skip this contour if the center pixel's color does
    // not match the specified color. Note that we are processing gray scale images here...
    if(params.filterByColor)
      if(binaryImage.at<uchar>(cvRound(center.location.y), cvRound(center.location.x)) != params.blobColor) {
        cv::drawContours(inter, contours, contourIdx, filteredByColorColor, 3);
        continue;
      }

    // By the time we reach here, the current contour apparently hasn't been filtered out,
    // so we compute the blob radius and store it as a Center in the centers std::vector.
    std::vector<double> dists;
    for(size_t pointIdx = 0; pointIdx < contours[contourIdx].size(); pointIdx++) {
      cv::Point2d pt = contours[contourIdx][pointIdx];
      dists.push_back(cv::norm(center.location - pt));
    }
    std::sort(dists.begin(), dists.end());
    center.radius = (dists[(dists.size() - 1) / 2] + dists[dists.size() / 2]) / 2.;
    centers.push_back(center);

    // If a contour has not been filtered out, overwrite it in red.
    cv::drawContours(inter, contours, contourIdx, selectedColor, 2);
  }

  // Let the window border reflect the number of found and filtered contours.
  ostringstream os;
  os << "Threshold: " << cv::threshold << ", number of contours found: " << contours.size();
  os << ", remaining after filtering: " << centers.size();
  cv::imshow(os.str(), inter);
  cv::moveWindow(os.str(), 100 + window * 50, 100 + window++ * 50);
}

// ------------------------------------------------------------------------------------------------
// detect()- Detects blobs in an image.
// ------------------------------------------------------------------------------------------------
// PRE  : image contains an image
// PRE  : keypoints is a reference to a std::vector of Keycv::Point instances
// PRE  : mask is an ROI matrix with nonzero values for pixels of interest
// POST : keypoints contains a Keycv::Point instance for each blob that was found
// ------------------------------------------------------------------------------------------------
void
detect(InputArray image, std::vector<Keycv::Point>& keypoints, InputArray mask) {
  keypoints.clear();

  // When necessary, convert the supplied image to a gray scale image.
  cv::Mat grayscaleImage;
  if(image.channels() == 3 || image.channels() == 4)
    cv::cvtColor(image, grayscaleImage, COLOR_BGR2GRAY);
  else
    grayscaleImage = image.getcv::Mat();

  // We only support 8 bit images.
  if(grayscaleImage.type() != CV_8UC1)
    CV_Error(cv::Error::StsUnsupportedFormat, "Blob detector only supports 8-bit images!");

  std::vector<std::vector<Center>> centers;
  for(double thresh = params.minThreshold; thresh < params.maxThreshold; thresh += params.thresholdStep) {
    // For each cv::threshold value from the specified minimum to the specified maximum using the
    // specified step size, generate a binary image and find the centers of any blobs in it.
    cv::Mat binarizedImage;
    cv::threshold(grayscaleImage, binarizedImage, thresh, 255, THRESH_BINARY);
    std::vector<Center> curCenters;
    findBlobs(grayscaleImage, binarizedImage, curCenters, thresh);

    // Now that we have the centers of any blobs in the image, cancel out all the centers that
    // are within the minimum distance between blobs.
    std::vector<std::vector<Center>> newCenters;
    for(size_t i = 0; i < curCenters.size(); i++) {
      bool isNew = true;
      for(size_t j = 0; j < centers.size(); j++) {
        double dist = cv::norm(centers[j][centers[j].size() / 2].location - curCenters[i].location);
        isNew = dist >= params.minDistBetweenBlobs && dist >= centers[j][centers[j].size() / 2].radius &&
                dist >= curCenters[i].radius;
        if(!isNew) {
          centers[j].push_back(curCenters[i]);
          size_t k = centers[j].size() - 1;
          while(k > 0 && centers[j][k].radius < centers[j][k - 1].radius) {
            centers[j][k] = centers[j][k - 1];
            k--;
          }
          centers[j][k] = curCenters[i];
          break;
        }
      }
      if(isNew)
        newCenters.emplace_back(std::vector<Center>(1, curCenters[i]));
    }
    copy(newCenters.begin(), newCenters.end(), back_inserter(centers));
  }

  // Now convert the centers that were found into Keycv::Points. Skip contours with
  // a repeat count lesser than the minimum repeatability.
  for(size_t i = 0; i < centers.size(); i++) {
    if(centers[i].size() < params.minRepeatability)
      continue;
    cv::Point2d sumcv::Point(0, 0);
    double normalizer = 0;
    for(size_t j = 0; j < centers[i].size(); j++) {
      sumcv::Point += centers[i][j].confidence * centers[i][j].location;
      normalizer += centers[i][j].confidence;
    }
    sumcv::Point *= (1. / normalizer);
    Keycv::Point kpt(sumcv::Point, (float)(centers[i][centers[i].size() / 2].radius) * 2.0f);
    keypoints.push_back(kpt);
  }

  // Filter the Keycv::Points by the supplied mask, if any.
  if(!mask.empty())
    Keycv::PointsFilter::runByPixelsMask(keypoints, mask.getcv::Mat());
}

// ------------------------------------------------------------------------------------------------
// main()
// ------------------------------------------------------------------------------------------------
int
main(int argc, char** argv) {
  /* Needed to generate the file (saved for reference only). Please be aware that there is a
     bug in OpenCV that prevents the closing element </opencv_storage> from being written.
     Make sure you cv::add it to the generated XML file manually, or reading the file will crash. */
  // cv::FileStorage storage("../BlobDetectionParameters.xml", cv::FileStorage::WRITE);
  // params.cv::write(storage);

  // Read the blob detection parameters from the designated file.
  cv::FileStorage storage("../BlobDetectionParameters.xml", cv::FileStorage::READ);
  cv::FileNode node = storage["opencv_storage"];
  params.cv::read(node);

  // This program requires a filename as the first command cv::line parameter. Read the
  // image from the specified file, if possible.
  if(argc < 2) {
    cout << "Usage: cv::SimpleBlobDetector {filename} [lightOnDark]" << endl;
    exit(-1);
  }

  cv::Mat image = cv::imread(argv[1], cv::LOAD_IMAGE_COLOR);
  if(!image.data) {
    cout << "Could not open file " << argv[1] << endl;
    exit(-1);
  }

  // An optional second parameter specifies whether or not the image contains bright
  // objects on a dark background (default) or not.
  bool lightObjectsOnDarkBackground = true;
  if(argc > 2) {
    string bright = argv[2];
    if(bright.compare("true") != 0 && bright.compare("false") != 0) {
      cout << "Usage: cv::SimpleBlobDetector {filename} (true|false)" << endl;
      exit(-1);
    }
    if(bright.compare("false") == 0)
      lightObjectsOnDarkBackground = false;
  }

  // Convert the image to a gray scale image and invert it (B/W) when necessary.
  // Then perform opening to facilitate detection of contours on adjacent objects.
  // Note that these steps are NOT part of cv::SimpleBlobDetector's algorithms.
  cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
  if(!lightObjectsOnDarkBackground)
    cv::bitwise_not(image, image);
  cv::dilate(image, image, cv::Mat::ones(5, 5, CV_32F));
  cv::erode(image, image, cv::Mat::ones(5, 5, CV_32F));

  // Show the original (converted to grayscale) and allow clicking. This shows the
  // actual gray value under the cursor, which may help in determining which threshold
  // values are appropriate.
  cv::imshow("Original", image);
  cv::setMouseCallback("Original", onMouseClick, &image);
  cv::moveWindow("Original", 400, 0);

  // Find blobs in the image, using the blob detector parameters specified above.
  // Show the coordinates of all keypoints that were found.
  cv::Mat mask, result;
  std::vector<Keycv::Point> keypoints;
  detect(image, keypoints, mask);
  for(Keycv::Point k : keypoints) cout << "(" << k.pt.x << "," << k.pt.y << ")" << endl;
  drawKeypoints(image, keypoints, result, cv::Scalar(0, 0, 255), Drawcv::MatchesFlags::DRAW_RICH_KEYPOINTS);
  ostringstream os;
  os << "Result: " << keypoints.size() << " keypoints.";
  cv::imshow(os.str(), result);
  cv::moveWindow(os.str(), 800, 0);

  // Wait for a key.
  cv::waitKey(0);
  exit(0);
}
