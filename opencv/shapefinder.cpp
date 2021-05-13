// Finds a shape from the input image.
// Example:
//    ./shapefinder -i square -s square -c #FF0000 -o img.png
//    args  description
//    -i    input image. (eg. circle, rectangle, square, multiple)
//    -s    shape to detect. (eg. circle, rectangle, square)
//    -c    color. (eg. #FF0000, FF0000)
//    -o    output image. (img.png)

#include <shapefinder.h>

map<string, string> args;

int
main(int argc, char* argv[]) {
  load_arguments(argc, argv);

  if(detect_shape()) {
    cout << "detected a shape!" << endl;
  }

  cin.get();
  return 0;
}

void
load_arguments(int argc, char* argv[]) {
  int cnt = (argc - 1) / 2;

  cout << "num_arguments: " << cnt << endl;

  for(int i = 0; i < cnt; i++) { args[argv[(2 * i) + 1]] = argv[(2 * i) + 2]; }

  for(auto it = args.cbegin(); it != args.cend(); it++) {
    cout << "args[\"" << it->first << "\"] = \"" << it->second << "\"\n";
  }
}

short*
get_rgb(string hex_string) {
  if(hex_string.at(0) == '#') {
    hex_string.erase(0, 1);
  }
  short* output = (short*)malloc(sizeof(short) * 3);
  output[0] = stoi(hex_string.substr(0, 2), nullptr, 16);
  output[1] = stoi(hex_string.substr(2, 2), nullptr, 16);
  output[2] = stoi(hex_string.substr(4, 2), nullptr, 16);
  return output;
}

bool
detect_shape() {
  bool found_shape = false;

  short* rgb_color = get_rgb(args["-c"]);

  string file_directory = "rsc/" + args["-i"] + ".ppm";
  cout << "Reading image: " << file_directory << endl;

  Mat img = imread(file_directory);
  if(img.empty()) {
    return false;
  }

  // cout << "Image data: " << img << endl;

  // Creates a grayscale version of the image.
  Mat gray;
  cvtColor(img, gray, cv::COLOR_BGR2GRAY);

  Mat edges;

  // Using otsu thresholding to get low & high value.
  double high = threshold(gray, edges, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
  double low = 0.5 * high;
  // Creates an image that contains edges based on the grayscale image.
  Canny(gray, edges, low, high);

  // Find contours
  vector<vector<Point>> contours;
  findContours(edges.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  Mat output = img.clone();

  for(int i = 0; i < contours.size(); i++) {
    cout << i << endl;
    vector<Point> approx;
    // Approximate contour with accuracy proportional
    // to the contour perimeter
    approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true) * 0.02, true);

    // Skip small or non-convex objects
    if(fabs(contourArea(contours[i])) < (img.size().height * img.size().width) / 2500 || !isContourConvex(approx)) {
      continue;
    }

    if(approx.size() == 4) {
      double bottom_x = approx[1].x - (fabs(approx[3].x - approx[1].x) / 20);
      double bottom_y = approx[1].y + (fabs(approx[3].y - approx[1].y) / 20);

      // approx index order
      // 03
      // 12

      double top_x = approx[3].x + (fabs(approx[3].x - approx[0].x) / 20);
      double top_y = approx[3].y - (fabs(approx[1].y - approx[0].y) / 20);

      double width = fabs(approx[0].x - approx[3].x);
      double height = fabs(approx[0].y - approx[1].y);

      bool isSquare = false;

      // Allow small difference betweeen width and height when detecting
      // squares.
      if(fabs(width - height) < (width + height) / 20) {
        // Square
        isSquare = true;
      } else {
        // Rectangle
        isSquare = false;
      }

      if(isSquare && args["-s"] == "square" || !isSquare && args["-s"] == "rectangle") {
        cout << "top_x = " << top_x << endl;
        rectangle(
            output, Point(bottom_x, bottom_y), Point(top_x, top_y), Scalar(rgb_color[2], rgb_color[1], rgb_color[0]), 3, 8, 0);
        found_shape = true;
      }
    } else {
      vector<Vec3f> circles;

      // Apply the Hough Transform to find the circles
      HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1, gray.rows / 8, 200, 100, 0, 0);

      // Draw the circles detected
      for(size_t i = 0; i < circles.size(); i++) {
        Point center = Point(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]) + (cvRound(circles[i][2]) / 10);

        if(args["-s"] == "circle") {
          // circle outline
          circle(output, center, radius, Scalar(rgb_color[2], rgb_color[1], rgb_color[0]), 3, 8, 0);
          found_shape = true;
        }
      }
    }
  }

  namedWindow("output", WINDOW_NORMAL);
  imshow("output", output);

  imwrite(args["-o"], output);

  waitKey(0);
  destroyAllWindows();

  free(rgb_color);
  return found_shape;
}
