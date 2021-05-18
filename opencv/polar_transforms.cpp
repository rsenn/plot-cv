#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

//using namespace cv;

static void
help(void) {
  printf("\nThis program illustrates Linear-Polar and Log-Polar image transforms\n"
         "Usage :\n"
         "./polar_transforms [[camera number -- Default 0],[path_to_filename]]\n\n");
}

int
main(int argc, char** argv) {
  cv::VideoCapture capture;
  cv::Mat log_polar_img, lin_polar_img, recovered_log_polar, recovered_lin_polar_img;

  help();

  cv::CommandLineParser parser(argc, argv, "{@input|0|}");
  std::string arg = parser.get<std::string>("@input");

  if(arg.size() == 1 && isdigit(arg[0]))
    capture.open(arg[0] - '0');
  else
    capture.open(arg.c_str());

  if(!capture.isOpened()) {
    const char* name = argv[0];
    fprintf(stderr, "Could not initialize capturing...\n");
    fprintf(stderr, "Usage: %s <CAMERA_NUMBER>    , or \n       %s <VIDEO_FILE>\n", name, name);
    return -1;
  }

  cv::namedWindow("Linear-Polar", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("Log-Polar", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("Recovered Linear-Polar", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("Recovered Log-Polar", cv::WINDOW_AUTOSIZE);

  cv::moveWindow("Linear-Polar", 20, 20);
  cv::moveWindow("Log-Polar", 700, 20);
  cv::moveWindow("Recovered Linear-Polar", 20, 350);
  cv::moveWindow("Recovered Log-Polar", 700, 350);

  for(;;) {
    cv::Mat frame;
    capture >> frame;

    if(frame.empty())
      break;

    cv::Point2f center((float)frame.cols / 2, (float)frame.rows / 2);
    double radius = (double)frame.cols / 4;
    double M = (double)frame.cols / cv::log(radius);

    cv::logPolar(frame, log_polar_img, center, M, cv::INTER_LINEAR + cv::WARP_FILL_OUTLIERS);
    cv::linearPolar(frame, lin_polar_img, center, radius, cv::INTER_LINEAR + cv::WARP_FILL_OUTLIERS);

    cv::logPolar(log_polar_img, recovered_log_polar, center, M, cv::WARP_INVERSE_MAP + cv::INTER_LINEAR);
    cv::linearPolar(lin_polar_img, recovered_lin_polar_img, center, radius, cv::WARP_INVERSE_MAP + cv::INTER_LINEAR + cv::WARP_FILL_OUTLIERS);

    cv::imshow("Log-Polar", log_polar_img);
    cv::imshow("Linear-Polar", lin_polar_img);
    cv::imshow("Recovered Linear-Polar", recovered_lin_polar_img);
    cv::imshow("Recovered Log-Polar", recovered_log_polar);

    if(cv::waitKey(10) >= 0)
      break;
  }

  cv::waitKey(0);
  return 0;
}
