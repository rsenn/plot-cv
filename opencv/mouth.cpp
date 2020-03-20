/*
 * Author: Samyak Datta (datta[dot]samyak[at]gmail.com)
 *
 *
 */

#include <iostream>
#include <cmath>
#include <cfloat>
#include <climits>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>

using namespace std;
using namespace cv;

cv::Mat_<Vec3b> extractFaceROI(cv::Mat_<Vec3b> image, string face_cascade_path);
cv::Mat_<Vec3b> extractMouthROI(cv::Mat_<Vec3b> face_image);

cv::Mat_<Vec3b> equalizeImage(cv::Mat_<Vec3b> image_BGR);
cv::Mat_<uchar> transformPseudoHue(cv::Mat_<Vec3b> image);
pair<double, double> Stats(cv::Mat_<double> pseudo_hue_plane);
cv::Mat_<uchar> transformCIELAB(cv::Mat_<Vec3b> image_BGR);
cv::Mat_<uchar> transformLUX(cv::Mat_<Vec3b> image_BGR);
cv::Mat_<uchar> transformModifiedLUX(cv::Mat_<Vec3b> image_BGR);

pair<double, double> returnImageStats(const cv::Mat_<uchar>& image);
cv::Mat_<uchar> binaryThresholding(const cv::Mat_<uchar>& image, const pair<double, double>& stats);
int returnLargestContourIndex(std::vector<std::vector<cv::Point>> contours);
int findClosest(std::vector<int> x_contour, int x);

int
main(int argc, char** argv) {
  if(argc != 3) {
    cout << "Paramters missing\n";
    return -1;
  }

  const string input_image_path = argv[1];
  const string face_cascade_path = argv[2];

  cv::Mat_<Vec3b> image_BGR = imread(input_image_path);
  cv::Mat_<Vec3b> face = extractFaceROI(image_BGR, face_cascade_path);
  cv::Mat_<Vec3b> mouth = extractMouthROI(face);

  cv::Mat_<uchar> pseudo_hue_plane = transformPseudoHue(mouth);
  cv::Mat_<uchar> pseudo_hue_bin =
      binaryThresholding(pseudo_hue_plane, returnImageStats(pseudo_hue_plane));

  // A clone image is required because findContours() modifies the input image
  cv::Mat binary_clone = pseudo_hue_bin.clone();
  std::vector<std::vector<cv::Point>> contours;
  findContours(binary_clone, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

  // Initialize blank image (for drawing contours)
  cv::Mat_<Vec3b> image_contour(pseudo_hue_bin.size());
  for(int i = 0; i < image_contour.rows; ++i) {
    for(int j = 0; j < image_contour.cols; ++j) {
      image_contour(i, j)[0] = 0;
      image_contour(i, j)[1] = 0;
      image_contour(i, j)[2] = 0;
    }
  }

  // Draw largest contour on the blank image
  int largest_contour_idx = returnLargestContourIndex(contours);
  std::vector<cv::Point> largest_contour = contours[largest_contour_idx];
  std::vector<int> x_contour(largest_contour.size());
  std::vector<int> y_contour(largest_contour.size());
  for(int i = 0; i < largest_contour.size(); ++i) {
    cv::Point_<int> pt = largest_contour[i];
    x_contour[i] = pt.x;
    y_contour[i] = pt.y;

    image_contour(pt.y, pt.x)[0] = 255;
    image_contour(pt.y, pt.x)[1] = 255;
    image_contour(pt.y, pt.x)[2] = 255;
  }

  // Find points with minimum and maximum x co-rodinates
  int min_x = INT_MAX, max_x = INT_MIN;
  int min_x_idx = -1, max_x_idx = -1;
  for(int i = 0; i < x_contour.size(); ++i) {
    if(x_contour[i] < min_x) {
      min_x = x_contour[i];
      min_x_idx = i;
    }

    if(x_contour[i] > max_x) {
      max_x = x_contour[i];
      max_x_idx = i;
    }
  }
  int min_y = y_contour[min_x_idx], max_y = y_contour[max_x_idx];

  // Find x-cordinate of point which is mid-way between the 2 corner points.
  int actual_mid_x = (min_x + max_x) / 2;
  int closest_x_idx = findClosest(x_contour, actual_mid_x);
  int closest_mid_x = x_contour[closest_x_idx];

  int count = 0;
  std::vector<int> mid_y_values;
  for(int i = 0; i < x_contour.size(); ++i) {
    if(x_contour[i] == closest_mid_x) {
      ++count;
      mid_y_values.push_back(y_contour[i]);
    }
  }

  // Mark end-points
  circle(image_contour, cv::Point(min_x, min_y), 3.0, Scalar(0, 0, 255), -1, 8);
  circle(image_contour, cv::Point(max_x, max_y), 3.0, Scalar(0, 0, 255), -1, 8);
  line(image_contour, cv::Point(min_x, min_y), cv::Point(max_x, max_y), Scalar(0, 0, 255), 1, 8);

  // Mark mid-points
  for(int i = 0; i < mid_y_values.size(); ++i) {
    circle(image_contour, cv::Point(closest_mid_x, mid_y_values[i]), 3.0, Scalar(0, 0, 255), -1, 8);
    line(image_contour,
         cv::Point(closest_mid_x, mid_y_values[i]),
         cv::Point(min_x, min_y),
         Scalar(0, 0, 255),
         1,
         8);
    line(image_contour,
         cv::Point(closest_mid_x, mid_y_values[i]),
         cv::Point(max_x, max_y),
         Scalar(0, 0, 255),
         1,
         8);
  }

  // imshow("Face-ROI", face);
  imshow("Mouth-ROI", mouth);
  // imshow("Input-Image", image_BGR);
  imshow("Contour", image_contour);
  // imshow("Pseudo-Hue", pseudo_hue_plane);
  // imshow("Pseudo-Hue-Binary", pseudo_hue_bin);

  /*
   * cv::Mat_<uchar> chrominance_plane = transformLUX(image_BGR);
   * cv::Mat_<uchar> modified_chrominance_plane = transformModifiedLUX(image_BGR);
   *
   * imshow("Chrominance-Plane", chrominance_plane);
   * imshow("Modified-Chrominance-Plane", modified_chrominance_plane);
   *
   */

  waitKey(0);
  return 0;
}

cv::Mat_<Vec3b>
extractFaceROI(cv::Mat_<Vec3b> image, string face_cascade_path) {
  CascadeClassifier face_cascade;
  std::vector<Rect_<int>> faces;

  face_cascade.load(face_cascade_path);
  face_cascade.detectMultiScale(image, faces, 1.15, 3, 0 | CASCADE_SCALE_IMAGE, Size(30, 30));

  cv::Mat_<Vec3b> face_ROI;
  for(int i = 0; i < faces.size(); ++i) {
    Rect_<int> face = faces[i];

    int face_rows = face.height;
    int face_cols = face.width;

    face_ROI = image(Rect(face.x, face.y, face_cols, face_rows));

    /*
    int roi_x = face.x, roi_y = face.y + ((2 * face_rows) / 3);
    int roi_rows = (face_rows - roi_y), roi_cols = face_cols;

    rectangle(image, cv::Point(roi_x, roi_y), cv::Point(roi_x+roi_cols, roi_y+roi_rows),
                            Scalar(255, 0, 0), 1, 4);

    */
  }
  return face_ROI;
}

cv::Mat_<Vec3b>
extractMouthROI(cv::Mat_<Vec3b> face_image) {
  int face_rows = face_image.rows;
  int face_cols = face_image.cols;

  int mouth_x = (face_cols / 4), mouth_y = (3 * face_rows) / 4;
  int mouth_rows = (face_rows - mouth_y), mouth_cols = (face_cols / 2);

  return face_image(Rect(mouth_x, mouth_y, mouth_cols, mouth_rows));
}

/*
 * Equalize a BGR image by converting it to the YCrCb space and
 * performing histogram equalization on the Y-plane before
 * merging back
 */
cv::Mat_<Vec3b>
equalizeImage(cv::Mat_<Vec3b> image_BGR) {
  std::vector<cv::Mat> channels;
  cv::Mat_<Vec3b> image_eq;

  cvtColor(image_BGR, image_eq, CV_BGR2YCrCb);
  split(image_eq, channels);
  equalizeHist(channels[0], channels[0]);
  merge(channels, image_eq);
  cvtColor(image_eq, image_eq, CV_YCrCb2BGR);

  return image_eq;
}

// Extract the pseudo-hue plane
cv::Mat_<uchar>
transformPseudoHue(cv::Mat_<Vec3b> image) {
  cv::Mat_<double> pseudo_hue(image.size());
  cv::Mat_<uchar> pseudo_hue_norm(image.size());

  int B = 0, G = 0, R = 0;
  for(int i = 0; i < image.rows; ++i) {
    for(int j = 0; j < image.cols; ++j) {
      B = image(i, j)[0];
      G = image(i, j)[1];
      R = image(i, j)[2];

      if(R == 0)
        pseudo_hue.at<double>(i, j) = 0.0;
      else
        pseudo_hue.at<double>(i, j) = (double)R / (R + G);
    }
  }
  pair<double, double> statistics = Stats(pseudo_hue);
  double Hmin = statistics.first;
  double Hmax = statistics.second;
  double Hrange = (Hmax - Hmin);

  for(int i = 0; i < image.rows; ++i) {
    for(int j = 0; j < image.cols; ++j) {
      double temp = (pseudo_hue.at<double>(i, j) - Hmin) / Hrange;
      pseudo_hue_norm.at<uchar>(i, j) = round(temp * 255);
    }
  }

  return pseudo_hue_norm;
}

pair<double, double>
Stats(cv::Mat_<double> pseudo_hue_plane) {
  double Hmax = DBL_MIN;
  double Hmin = DBL_MAX;

  for(int i = 0; i < pseudo_hue_plane.rows; ++i) {
    for(int j = 0; j < pseudo_hue_plane.cols; ++j) {
      if(pseudo_hue_plane.at<double>(i, j) >= Hmax + numeric_limits<double>::epsilon())
        Hmax = pseudo_hue_plane.at<double>(i, j);
      if(pseudo_hue_plane.at<double>(i, j) <= Hmin + numeric_limits<double>::epsilon())
        Hmin = pseudo_hue_plane.at<double>(i, j);
    }
  }
  return make_pair(Hmin, Hmax);
}

// CIELAB transformation and using the A-channel
cv::Mat_<uchar>
transformCIELAB(cv::Mat_<Vec3b> image_BGR) {
  cv::Mat_<Vec3b> image_Lab;
  std::vector<cv::Mat> channels;
  cvtColor(image_BGR, image_Lab, CV_BGR2Lab);
  split(image_Lab, channels);

  cv::Mat_<uchar> image_a = channels[1];
  return image_a;
}

cv::Mat_<uchar>
transformLUX(cv::Mat_<Vec3b> image_BGR) {
  cv::Mat_<uchar> U(image_BGR.size());

  int B = 0, G = 0, R = 0, L_int = 0, u_int = 0;
  double L = 0.0, u = 0.0;
  for(int i = 0; i < image_BGR.rows; ++i) {
    for(int j = 0; j < image_BGR.cols; ++j) {
      B = image_BGR(i, j)[0];
      G = image_BGR(i, j)[1];
      R = image_BGR(i, j)[2];

      L = (pow(R + 1, 0.3) * pow(G + 1, 0.6) * pow(B + 1, 0.1)) - 1;
      L_int = round(L);

      if(R > L_int) {
        u = (256 * (L_int + 1)) / (R + 1);
        u_int = round(u);
        U.at<uchar>(i, j) = u_int;
      } else
        U.at<uchar>(i, j) = 255;
    }
  }
  return U;
}

cv::Mat_<uchar>
transformModifiedLUX(cv::Mat_<Vec3b> image_BGR) {
  cv::Mat_<uchar> Ucap(image_BGR.size());

  int B = 0, G = 0, R = 0, u_cap_int = 0;
  double u_cap = 0.0;
  for(int i = 0; i < image_BGR.rows; ++i) {
    for(int j = 0; j < image_BGR.cols; ++j) {
      B = image_BGR(i, j)[0];
      G = image_BGR(i, j)[1];
      R = image_BGR(i, j)[2];

      if(R > G) {
        u_cap = (256 * G) / R;
        u_cap_int = round(u_cap);
        Ucap.at<uchar>(i, j) = u_cap_int;
      } else
        Ucap.at<uchar>(i, j) = 255;
    }
  }
  return Ucap;
}

pair<double, double>
returnImageStats(const cv::Mat_<uchar>& image) {
  double mean = 0.0, std_dev = 0.0;
  int total_pixels = (image.rows * image.cols);

  int intensity_sum = 0;
  for(int i = 0; i < image.rows; ++i) {
    for(int j = 0; j < image.cols; ++j) intensity_sum += image.at<uchar>(i, j);
  }
  mean = (double)intensity_sum / total_pixels;

  int sum_sq = 0;
  for(int i = 0; i < image.rows; ++i) {
    for(int j = 0; j < image.cols; ++j)
      sum_sq += ((image.at<uchar>(i, j) - mean) * (image.at<uchar>(i, j) - mean));
  }
  std_dev = sqrt((double)sum_sq / total_pixels);

  return make_pair(mean, std_dev);
}

cv::Mat_<uchar>
binaryThresholding(const cv::Mat_<uchar>& image, const pair<double, double>& stats) {
  cv::Mat_<uchar> image_binary(image.size());

  double Z = 0.9;
  double threshold = stats.first + (Z * stats.second);
  for(int i = 0; i < image.rows; ++i) {
    for(int j = 0; j < image.cols; ++j) {
      if(image.at<uchar>(i, j) >= threshold + numeric_limits<double>::epsilon())
        image_binary.at<uchar>(i, j) = 255;
      else
        image_binary.at<uchar>(i, j) = 0;
    }
  }
  return image_binary;
}

int
returnLargestContourIndex(std::vector<std::vector<cv::Point>> contours) {
  int max_contour_size = 0;
  int max_contour_idx = -1;
  for(int i = 0; i < contours.size(); ++i) {
    if(contours[i].size() > max_contour_size) {
      max_contour_size = contours[i].size();
      max_contour_idx = i;
    }
  }
  return max_contour_idx;
}

int
findClosest(std::vector<int> x_contour, int x) {
  std::vector<int> diff(x_contour.size());
  for(int i = 0; i < x_contour.size(); ++i) diff[i] = (int)abs(x_contour[i] - x);

  // Find the minimum value in the diff array
  int min_diff = INT_MAX, min_diff_idx = -1;
  for(int i = 0; i < diff.size(); ++i) {
    if(diff[i] < min_diff) {
      min_diff = diff[i];
      min_diff_idx = i;
    }
  }
  return min_diff_idx;
}
