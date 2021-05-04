
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

int
main(int argc, char** argv) {
  const cv::String filename = "../an-tronics/images/fm/Two-Transistor-Regenerative-Receiver-Schematic-Circuit-Diagram.jpg";
  // Loads an image
  cv::Mat src = cv::imread(filename, cv::IMREAD_COLOR);
  // Check if image is loaded fine
  if(src.empty()) {
    printf(" Error opening image\n");
    printf(" Program Arguments: [image_name -- default %s] \n", filename.c_str());
    return EXIT_FAILURE;
  }
  cv::UMat gray;
  cv::InputArray input(src);
  cv::OutputArray output(gray);

  cv::cvtColor(input, output, cv::COLOR_BGR2GRAY);
  cv::imshow("gray", gray);
  cv::medianBlur(gray, gray, 5);
  std::vector<cv::Vec3f> circles;
  cv::HoughCircles(gray,
                   circles,
                   cv::HOUGH_GRADIENT,
                   1,
                   gray.rows / 16, // change this value to detect circles with different distances to each other
                   100,
                   30,
                   1,
                   30 // change the last two parameters
                      // (min_radius & max_radius) to detect larger circles
  );
  for(size_t i = 0; i < circles.size(); i++) {
    cv::Vec3i c = circles[i];
    cv::Point center = cv::Point(c[0], c[1]);
    // circle center
    cv::circle(src, center, 1, cv::Scalar(0, 100, 100), 3, cv::LINE_AA);
    // circle outline
    int radius = c[2];
    cv::circle(src, center, radius, cv::Scalar(255, 0, 255), 3, cv::LINE_AA);
  }
  cv::imshow("detected circles", src);
  cv::waitKey();
  return EXIT_SUCCESS;
}
