#include <opencv2/opencv.hpp>

//using namespace cv;
using namespace std;

void
showReducedImage(string title, cv::Mat img) {
  cv::Mat temp;
  cv::resize(img, temp, cv::Size(), 0.3, 0.3, cv::INTER_AREA);

  cv::imshow(title, temp);
}

static double
angle(cv::Point pt1, cv::Point pt2, cv::Point pt0) {
  double dx1 = pt1.x - pt0.x;
  double dy1 = pt1.y - pt0.y;
  double dx2 = pt2.x - pt0.x;
  double dy2 = pt2.y - pt0.y;
  return (dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
}

void
find_squares(cv::Mat& image, vector<vector<cv::Point>>& squares) {
  // cv::blur will enhance edge detection
  cv::Mat blurred(image);
  cv::medianBlur(image, blurred, 9);

  cv::Mat gray0(blurred.size(), CV_8U), gray;
  vector<vector<cv::Point>> contours;

  // find squares in every color plane of the image
  for(int c = 0; c < image.channels(); c++) {
    int ch[] = {c, 0};
    cv::mixChannels(&blurred, 1, &gray0, 1, ch, 1);

    // try several cv::threshold levels
    const int threshold_level = 2;
    for(int l = 0; l < threshold_level; l++) {
      // Use cv::Canny instead of zero cv::threshold level!
      // cv::Canny helps to catch squares with gradient shading
      if(l == 0) {
        cv::Canny(gray0, gray, 10, 20, 3); //

        // Dilate helps to remove potential holes between edge segments
        cv::dilate(gray, gray, cv::Mat(), cv::Point(-1, -1));
      } else {
        gray = gray0 >= (l + 1) * 255 / threshold_level;
      }

      // Find contours and store them in a list
      cv::findContours(gray, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

      // Test contours
      vector<cv::Point> approx;
      for(size_t i = 0; i < contours.size(); i++) {
        // approximate contour with accuracy proportional
        // to the contour perimeter
        cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(Mat(contours[i]), true) * 0.02, true);

        // Note: absolute value of an area is used because
        // area may be positive or negative - in accordance with the
        // contour orientation
        if(approx.size() == 4 && fabs(cv::contourArea(cv::Mat(approx))) > 1000 && cv::isContourConvex(Mat(approx))) {
          double maxCosine = 0;

          for(int j = 2; j < 5; j++) {
            double cosine = fabs(angle(approx[j % 4], approx[j - 2], approx[j - 1]));
            maxCosine = MAX(maxCosine, cosine);
          }

          if(maxCosine < 0.3)
            squares.push_back(approx);
        }
      }
    }
  }
}

// the function draws all the squares in the image
static void
drawSquares(cv::Mat& image, const vector<vector<cv::Point>>& squares) {
  for(size_t i = 0; i < squares.size(); i++) {
    const cv::Point* p = &squares[i][0];

    int n = (int)squares[i].size();
    // dont detect the border
    if(p->x > 3 && p->y > 3)
      cv::polylines(image, &p, &n, 1, true, cv::Scalar(0, 255, 0), 3, cv::LINE_AA);
  }

  showReducedImage("squares", image);
  cv::waitKey(0);
}

void
detectEdgesByHoughLines(cv::Mat mat) {
  cv::cvtColor(mat, mat, cv::COLOR_BGR2GRAY);
  cv::GaussianBlur(mat, mat, cv::Size(3, 3), 0);
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Point(9, 9));
  cv::Mat dilated;
  cv::dilate(mat, dilated, kernel);

  cv::Mat edges;
  cv::Canny(dilated, edges, 84, 3);

  std::vector<cv::Vec4i> lines;
  lines.clear();
  cv::HoughLinesP(edges, lines, 1, CV_PI / 180, 25, 10, 20);
  std::vector<cv::Vec4i>::iterator it = lines.begin();
  for(; it != lines.end(); ++it) {
    cv::Vec4i l = *it;
    cv::line(edges, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255, 0, 0), 2, 8);
  }

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_TC89_KCOS);
  std::vector<std::vector<cv::Point>> contoursCleaned;
  for(int i = 0; i < contours.size(); i++) {
    if(cv::arcLength(contours[i], false) > 100)
      contoursCleaned.push_back(contours[i]);
  }
  std::vector<std::vector<cv::Point>> contoursArea;

  for(int i = 0; i < contoursCleaned.size(); i++) {
    if(cv::contourArea(contoursCleaned[i]) > 10000) {
      contoursArea.push_back(contoursCleaned[i]);
    }
  }
  std::vector<std::vector<cv::Point>> contoursDraw(contoursCleaned.size());
  for(int i = 0; i < contoursArea.size(); i++) { cv::approxPolyDP(cv::Mat(contoursArea[i]), contoursDraw[i], 40, true); }

  cv::Mat drawing = cv::Mat::zeros(mat.size(), CV_8UC1);
  cv::drawContours(drawing, contoursDraw, -1, cv::Scalar(255), -1);

  cv::imshow("lines", drawing);
  cv::waitKey(0);
}

void
fill_Holes(cv::Mat image) {
  cv::Mat gray;
  cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

  cv::Mat image_thresh;
  cv::threshold(gray, image_thresh, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

  // Loop through the border pixels and if they're black, cv::floodFill from there
  cv::Mat mask;
  image_thresh.copyTo(mask);
  for(int i = 0; i < mask.cols; i++) {
    if(mask.at<char>(0, i) == 0) {
      cv::floodFill(mask, cv::Point(i, 0), 255, 0, 10, 10);
    }
    if(mask.at<char>(mask.rows - 1, i) == 0) {
      cv::floodFill(mask, cv::Point(i, mask.rows - 1), 255, 0, 10, 10);
    }
  }

  for(int i = 0; i < mask.rows; i++) {
    if(mask.at<char>(i, 0) == 0) {
      cv::floodFill(mask, cv::Point(0, i), 255, 0, 10, 10);
    }
    if(mask.at<char>(i, mask.cols - 1) == 0) {
      cv::floodFill(mask, cv::Point(mask.cols - 1, i), 255, 0, 10, 10);
    }
  }

  // Compare mask with original.
  cv::Mat newImage;
  image.copyTo(newImage);
  for(int row = 0; row < mask.rows; ++row) {
    for(int col = 0; col < mask.cols; ++col) {
      if(mask.at<char>(row, col) == 0) {
        newImage.at<char>(row, col) = 255;
      }
    }
  }

  cv::imshow("filled image", mask);
  cv::imshow("Final image", newImage);
  // cv::imwrite("final.jpg", newImage);
  cv::waitKey(0);
}

void
findDocumentEdges(cv::Mat img) {

  cv::Mat tmp;
  cv::resize(img, tmp, cv::Size(), 0.3, 0.3);

  detectEdgesByHoughLines(tmp);
}

void
findGraphValues(cv::Mat graph) {
  vector<vector<cv::Point>> cnts;

  vector<cv::Vec4i> hierarchy;

  cout << "computing contours " << endl;

  cv::Mat thresh;
  cv::threshold(graph.clone(), thresh, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

  showReducedImage("cv::threshold", thresh);

  cv::findContours(graph.clone(), cnts, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

  // draw contours
  cv::Mat cntImg = cv::Mat::zeros(graph.size(), CV_8UC1);

  for(int i = 0; i < cnts.size(); i++) cv::drawContours(cntImg, cnts, i, cv::Scalar(255));

  showReducedImage("contours", cntImg);
  cv::waitKey(0);
}

vector<float>
findUniqueValues(const cv::Mat& input, bool sort = false) {
  if(input.channels() > 1 || input.type() != CV_32F) {
    cerr << "unique !!! Only works with CV_32F 1-channel cv::Mat" << endl;
    return vector<float>();
  }

  vector<float> out;
  for(int y = 0; y < input.rows; ++y) {
    const float* row_ptr = input.ptr<float>(y);
    for(int x = 0; x < input.cols; ++x) {
      float value = row_ptr[x];

      if(find(out.begin(), out.end(), value) == out.end())
        out.push_back(value);
    }
  }

  if(sort)
    std::sort(out.begin(), out.end());

  return out;
}

bool
findKMeansClusters(cv::Mat img, int k) {
  vector<cv::Mat> imgRGB;
  cv::split(img, imgRGB);

  int n = img.rows * img.cols;
  cv::Mat img3xN(n, 3, CV_8U);

  for(int i = 0; i != 3; ++i) imgRGB[i].reshape(1, n).copyTo(img3xN.col(i));

  img3xN.convertTo(img3xN, CV_32F);

  cv::Mat bestLabels;
  cv::kmeans(img3xN, k, bestLabels, TermCriteria(), 10, KMEANS_RANDOM_CENTERS);

  bestLabels = bestLabels.reshape(0, img.rows);
  cv::convertScaleAbs(bestLabels, bestLabels, int(255 / k));

  cv::Mat labelImg;
  bestLabels.convertTo(labelImg, CV_32F);

  cout << "finding unique values: " << endl;

  vector<float> uniqueValues = findUniqueValues(labelImg, false);

  cout << "unique label values: " << endl;

  for(int i = 0; i < uniqueValues.size(); i++) cout << uniqueValues[i] << endl;

  vector<cv::Mat> labels;

  for(int i = 0; i < k; i++) {
    cv::Mat tmp = cv::Mat::zeros(bestLabels.size(), CV_8UC1);
    labels.push_back(tmp);
  }

  for(int r = 0; r < bestLabels.rows; r++)
    for(int c = 0; c < bestLabels.cols; c++)
      for(int i = 0; i < k; i++)
        if((int)bestLabels.at<uchar>(r, c) == (int)uniqueValues[i])
          labels[i].at<uchar>(r, c) = (int)uniqueValues[i];

  for(int i = 0; i < k; i++) {
    stringstream ss;
    ss << (i + 1);
    string winName = "label";
    winName.append(ss.str());
    showReducedImage(winName, labels[i]);
  }

  cv::waitKey(0);

  int userLabel = -1;
  cout << "choose label image corresponding to graph values " << endl;

  while(userLabel < 0 || userLabel > k) {
    cout << "choose number less than " << k << " but >= 0" << endl;

    cin >> userLabel;

    cout << "you entered: " << userLabel << endl;
  }

  cout << "showing selected label image: " << endl;

  showReducedImage("selected", labels[userLabel]);
  cv::waitKey(0);

  cout << "cv::write selected to file" << endl;
  cv::imwrite("selected.jpg", labels[userLabel]);

  /*cout << "want to proceed?y/n" << endl;
  string userChoice = "";
  getline(cin, userChoice);

  cout << "you entered: " << userChoice << endl;

  if(strcmp(userChoice.c_str(), "n"))
  {
    cout << "Bye bye ... " << endl;
    return false;
  }*/

  findGraphValues(labels[userLabel]);

  /*
    showReducedImage("result", bestLabels);
      cv::waitKey();
  */
  return true;
}

int
main(int argc, char** argv) {
  if(argc < 3)
    cout << "use case: test_project.exe <path/to/image> <no of clusters> " << endl;

  // cv::read image
  cv::Mat input = cv::imread(argv[1]);

  showReducedImage("input", input);

  cv::waitKey(0);

  // find document edges
  findDocumentEdges(input.clone());

  //// number of clusters
  // int k = atoi(argv[2]);

  // cout << "no of clusters: " << k << endl;

  // cv::Mat hsv;
  // cv::cvtColor(input, hsv, cv::COLOR_BGR2HSV);

  // showReducedImage("hsv", hsv);
  // cv::waitKey(0);

  // cout << "finding color clusters" << endl;

  // findKMeansClusters(hsv, k);

  return 1;
}
