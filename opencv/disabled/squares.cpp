// The "Square Detector" program.
// It loads several images sequentially and tries to find squares in
// each image

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <math.h>
#include <string.h>

// using namespace cv;
using namespace std;

static void
help() {
  cout << "\nA program using pyramid scaling, cv::Canny, contours, contour simpification and\n"
          "memory storage (it's got it all folks) to find\n"
          "squares in a list of images pic1-6.png\n"
          "Returns sequence of squares detected on the image.\n"
          "the sequence is stored in the specified memory storage\n"
          "Call:\n"
          "./squares\n"
          "Using OpenCV version %s\n"
       << CV_VERSION << "\n"
       << endl;
}

int thresh = 50, N = 11;
const char* wndname = "Square Detection Demo";

// helper function:
// finds a cosine of angle between std::vectors
// from pt0->pt1 and from pt0->pt2
static double
angle(cv::Point pt1, cv::Point pt2, cv::Point pt0) {
  double dx1 = pt1.x - pt0.x;
  double dy1 = pt1.y - pt0.y;
  double dx2 = pt2.x - pt0.x;
  double dy2 = pt2.y - pt0.y;
  return (dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
}

// returns sequence of squares detected on the image.
// the sequence is stored in the specified memory storage
static void
findSquares(const cv::Mat& image, std::vector<std::vector<cv::Point>>& squares) {
  squares.clear();

  cv::Mat pyr, timg, gray0(image.size(), CV_8U), gray;

  // down-scale and upscale the image to filter out the noise
  pyrDown(image, pyr, cv::Size(image.cols / 2, image.rows / 2));
  pyrUp(pyr, timg, image.size());
  std::vector<std::vector<cv::Point>> contours;

  // find squares in every color plane of the image
  for(int c = 0; c < 3; c++) {
    int ch[] = {c, 0};
    cv::mixChannels(&timg, 1, &gray0, 1, ch, 1);

    // try several cv::threshold levels
    for(int l = 0; l < N; l++) {
      // hack: use cv::Canny instead of zero cv::threshold level.
      // cv::Canny helps to catch squares with gradient shading
      if(l == 0) {
        // apply cv::Canny. Take the upper cv::threshold from slider
        // and set the lower to 0 (which forces edges merging)
        cv::Canny(gray0, gray, 0, thresh, 5);
        // cv::dilate canny output to remove potential
        // holes between edge segments
        cv::dilate(gray, gray, cv::Mat(), cv::Point(-1, -1));
      } else {
        // apply cv::threshold if l!=0:
        //     tgray(x,y) = gray(x,y) < (l+1)*255/N ? 255 : 0
        gray = gray0 >= (l + 1) * 255 / N;
      }

      // find contours and store them all as a list
      cv::findContours(gray, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

      std::vector<cv::Point> approx;

      // test each contour
      for(size_t i = 0; i < contours.size(); i++) {
        // approximate contour with accuracy proportional
        // to the contour perimeter
        cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true) * 0.02, true);

        // square contours should have 4 vertices after approximation
        // relatively large area (to filter out noisy contours)
        // and be convex.
        // Note: absolute value of an area is used because
        // area may be positive or negative - in accordance with the
        // contour orientation
        if(approx.size() == 4 && fabs(cv::contourArea(cv::Mat(approx))) > 1000 && cv::isContourConvex(cv::Mat(approx))) {
          double maxCosine = 0;

          for(int j = 2; j < 5; j++) {
            // find the maximum cosine of the angle between joint edges
            double cosine = fabs(angle(approx[j % 4], approx[j - 2], approx[j - 1]));
            maxCosine = MAX(maxCosine, cosine);
          }

          // if cosines of all angles are small
          // (all angles are ~90 degree) then cv::write quandrange
          // vertices to resultant sequence
          if(maxCosine < 0.3)
            squares.push_back(approx);
        }
      }
    }
  }
}

// the function draws all the squares in the image
static void
drawSquares(cv::Mat& image, const std::vector<std::vector<cv::Point>>& squares) {
  for(size_t i = 0; i < squares.size(); i++) {
    const cv::Point* p = &squares[i][0];
    int n = (int)squares[i].size();
    cv::polylines(image, &p, &n, 1, true, cv::Scalar(0, 255, 0), 3, cv::LINE_AA);
  }

  cv::imshow(wndname, image);
}

int
main(int /*argc*/, char** /*argv*/) {
  static const char* names[] = {"pic1.png", "pic2.png", "pic3.png", "pic4.png", "pic5.png", "pic6.png", 0};
  help();
  cv::namedWindow(wndname, 1);
  std::vector<std::vector<cv::Point>> squares;

  for(int i = 0; names[i] != 0; i++) {
    cv::Mat image = cv::imread(names[i], 1);
    if(image.empty()) {
      cout << "Couldn't load " << names[i] << endl;
      continue;
    }

    findSquares(image, squares);
    drawSquares(image, squares);

    int c = cv::waitKey();
    if((char)c == 27)
      break;
  }

  return 0;
}
