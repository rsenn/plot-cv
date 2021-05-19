#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

int
main(int argc, char** argv) {
  std::string in;
  cv::CommandLineParser parser(argc, argv, "{@input|../samples/data/corridor.jpg|input image}{help h||show help message}");
  if(parser.has("help")) {
    parser.printMessage();
    return 0;
  }
  in = parser.get<std::string>("@input");
  cv::Mat image = cv::imread(in, cv::IMREAD_GRAYSCALE);
  if(image.empty()) {
    return -1;
  }
  // Create FLD detector
  // Param               Default value   Description
  // length_threshold    10            - Segments shorter than this will be discarded
  // distance_threshold  1.41421356    - A point placed from a hypothesis line
  //                                     segment farther than this will be
  //                                     regarded as an outlier
  // canny_th1           50            - First threshold for
  //                                     hysteresis procedure in Canny()
  // canny_th2           50            - Second threshold for
  //                                     hysteresis procedure in Canny()
  // canny_aperture_size 3             - Aperturesize for the sobel
  //                                     operator in Canny()
  // do_merge            false         - If true, incremental merging of segments
  //                                     will be perfomred
  int length_threshold = 10;
  float distance_threshold = 1.41421356f;
  double canny_th1 = 50.0;
  double canny_th2 = 50.0;
  int canny_aperture_size = 3;
  bool do_merge = false;
  cv::Ptr<cv::ximgproc::FastLineDetector> fld = cv::ximgproc::createFastLineDetector(
      length_threshold, distance_threshold, canny_th1, canny_th2, canny_aperture_size, do_merge);
  std::vector<cv::Vec4f> lines_fld;
  // Because of some CPU's power strategy, it seems that the first running of
  // an algorithm takes much longer. So here we run the algorithm 10 times
  // to see the algorithm's processing time with sufficiently warmed-up
  // CPU performance.
  for(int run_count = 0; run_count < 10; run_count++) {
    double freq = cv::getTickFrequency();
    lines_fld.clear();
    int64 start = cv::getTickCount();
    // Detect the lines with FLD
    fld->detect(image, lines_fld);
    double duration_ms = double(cv::getTickCount() - start) * 1000 / freq;
    std::cout << "Elapsed time for FLD " << duration_ms << " ms." << std::endl;
  }
  // Show found lines with FLD
  cv::Mat line_image_fld(image);
  fld->drawSegments(line_image_fld, lines_fld);
  cv::imshow("FLD result", line_image_fld);
  cv::moveWindow("FLD result", 0, 0);
  cv::waitKey(0);
  return 0;
}
