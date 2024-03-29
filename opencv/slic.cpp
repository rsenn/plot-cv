#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/utility.hpp>

#include <opencv2/ximgproc.hpp>

#include <ctype.h>
#include <stdio.h>
#include <iostream>

// using namespace cv;
// using namespace cv::ximgproc;
using namespace std;

static const char* window_name = "SLIC Superpixels";

static const char* keys = "{h help      | | help menu}"
                          "{c camera    |0| camera id}"
                          "{i image     | | image file}"
                          "{a algorithm |1| SLIC(0),SLICO(1),MSLIC(2)}";

int
main(int argc, char** argv) {
  cv::CommandLineParser cmd(argc, argv, keys);
  if(cmd.has("help")) {
    cmd.about("This program demonstrates SLIC superpixels using OpenCV class cv::ximgproc::SuperpixelSLIC.\n"
              "If no image file is supplied, try to open a webcam.\n"
              "Use [space] to toggle output mode, ['q' or 'Q' or 'esc'] to exit.\n");
    cmd.printMessage();
    return 0;
  }
  int capture = cmd.get<int>("camera");
  cv::String img_file = cmd.get<cv::String>("image");
  int algorithm = cmd.get<int>("algorithm");
  int region_size = 50;
  int ruler = 30;
  int min_element_size = 50;
  int num_iterations = 3;
  bool use_video_capture = img_file.empty();

  cv::VideoCapture cap;
  cv::Mat input_image;

  if(use_video_capture) {
    if(!cap.open(capture)) {
      cout << "Could not initialize capturing..." << capture << "\n";
      return -1;
    }
  } else {
    input_image = cv::imread(img_file);
    if(input_image.empty()) {
      cout << "Could not open image..." << img_file << "\n";
      return -1;
    }
  }

  cv::namedWindow(window_name, 0);
  cv::createTrackbar("Algorithm", window_name, &algorithm, 2, 0);
  cv::createTrackbar("Region size", window_name, &region_size, 200, 0);
  cv::createTrackbar("Ruler", window_name, &ruler, 100, 0);
  cv::createTrackbar("Connectivity", window_name, &min_element_size, 100, 0);
  cv::createTrackbar("Iterations", window_name, &num_iterations, 12, 0);

  cv::Mat result, mask;
  int display_mode = 0;

  for(;;) {
    cv::Mat frame;
    if(use_video_capture)
      cap >> frame;
    else
      input_image.copyTo(frame);

    if(frame.empty())
      break;

    result = frame;
    cv::Mat converted;
    cv::cvtColor(frame, converted, cv::COLOR_BGR2HSV);

    double t = (double)cv::getTickCount();

    cv::Ptr<cv::ximgproc::SuperpixelSLIC> slic =
        cv::ximgproc::createSuperpixelSLIC(converted, algorithm + cv::ximgproc::SLIC, region_size, float(ruler));
    slic->iterate(num_iterations);
    if(min_element_size > 0)
      slic->enforceLabelConnectivity(min_element_size);

    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "SLIC" << (algorithm ? 'O' : ' ') << " segmentation took " << (int)(t * 1000) << " ms with "
         << slic->getNumberOfSuperpixels() << " superpixels" << endl;

    // get the contours for displaying
    slic->getLabelContourMask(mask, true);
    result.setTo(cv::Scalar(0, 0, 255), mask);

    // display output
    switch(display_mode) {
      case 0: // superpixel contours
        cv::imshow(window_name, result);
        break;
      case 1: // mask
        cv::imshow(window_name, mask);
        break;
      case 2: { // labels array
        // use the last x bit to determine the color. Note that this does not
        // guarantee that 2 neighboring superpixels have different colors.
        // retrieve the segmentation result
        cv::Mat labels;
        slic->getLabels(labels);
        const int num_label_bits = 2;
        labels &= (1 << num_label_bits) - 1;
        labels *= 1 << (16 - num_label_bits);
        cv::imshow(window_name, labels);
        break;
      }
    }

    int c = cv::waitKey(1) & 0xff;
    if(c == 'q' || c == 'Q' || c == 27)
      break;
    else if(c == ' ')
      display_mode = (display_mode + 1) % 3;
  }

  return 0;
}
