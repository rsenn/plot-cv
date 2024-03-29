#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>

// using namespace cv;
using namespace std;

static void
help(char** argv) {
  cout << "\nThis sample shows you how to cv::read a sequence of images using the cv::VideoCapture "
          "interface.\n"
       << "Usage: " << argv[0] << " <image_mask> (example mask: example_%02d.jpg)\n"
       << "Image mask defines the name variation for the input images that have to be cv::read as a "
          "sequence. \n"
       << "Using the mask example_%02d.jpg will cv::read in images labeled as 'example_00.jpg', "
          "'example_01.jpg', etc."
       << endl;
}

int
main(int argc, char** argv) {
  if(argc != 2) {
    help(argv);
    return 1;
  }

  string first_file = argv[1];
  cv::VideoCapture sequence(first_file);

  if(!sequence.isOpened()) {
    cerr << "Failed to open the image sequence!\n" << endl;
    return 1;
  }

  cv::Mat image;
  cv::namedWindow("Image sequence | press ESC to close", 1);

  for(;;) {
    // Read in image from sequence
    sequence >> image;

    // If no image was retrieved -> end of sequence
    if(image.empty()) {
      cout << "End of Sequence" << endl;
      break;
    }

    cv::imshow("Image sequence | press ESC to close", image);

    if(cv::waitKey(500) == 27)
      break;
  }

  return 0;
}
