#include <opencv2/opencv.hpp>

// using namespace cv;
using namespace std;

void createCookBook(cv::Mat staveReg, int rightIndex);
void removeStaves(cv::Mat image, Vector<int> firstVec, Vector<int> endVec, cv::Mat resultImage);
void findNotes(cv::Mat staveReg, Vector<float> staveLoc);
string type2str(int type);

#define quarterIndex 16
#define halfIndex 35
#define holeIndex 20
#define NOTE_THRESHOLD 20

cv::Mat templateQuarter;
cv::Mat templateHalf;
cv::Mat templateHole;

int
main(int argc, char** argv) {
  const char* filename = argc >= 2 ? argv[1] : "pic1.jpg";
  cv::Mat src = cv::imread(filename, 0);
  if(src.empty()) {
    cout << "can not open " << filename << endl;
    return -1;
  }

  cv::Mat bw, dest, imbw;
  cv::threshold(src, bw, 0, 1, THRESH_BINARY_INV | THRESH_OTSU); // Convert to BW image using Otsu
  cv::threshold(src, imbw, 0, 255, THRESH_BINARY | THRESH_OTSU);
  cv::Mat rowsums, colsums;
  int rows = bw.size[0];
  int cols = bw.size[1];
  reduce(bw, rowsums, 1, CV_REDUCE_SUM, CV_32SC1);
  reduce(bw, colsums, 0, CV_REDUCE_SUM, CV_32SC1);

  cv::Mat rowsums2, colsums2, colsums3;
  cv::normalize(colsums, colsums2, 0, 255, NORM_MINMAX, CV_8UC1);
  cv::threshold(colsums2, colsums3, 5, 1, THRESH_BINARY);

  int colthresh = cv::sum(colsums3)[0] * 8 / 10;

  compare(rowsums, colthresh, rowsums2, CMP_GT); // Now, rows with a staff cv::line have the value 255

  // The following few lines just print out the lines in red on the picture and display it
  // cv::cvtColor(src, dest, CV_GRAY2RGB);

  // for(int i = 0; i < rows; i++) {
  // if (rowsums2.at<unsigned char>(i, 0) == 255) {
  //   cv::line(dest, cv::Point2d(0, i), cv::Point2d(cols - 1, i), cv::Scalar(0, 0, 255));
  // }
  //}

  // cv::namedWindow( "Hough Circle Transform Demo", cv::WINDOW_AUTOSIZE );
  // cv::imshow( "Hough Circle Transform Demo", dest );
  // cv::waitKey();
  // End display code

  // The next step is to find staff cv::line spacing:

  Vector<int> staffrows;
  Vector<float> avgrows;
  Vector<int> staffbegins;
  Vector<int> staffends;

  int first = -1;
  int last = -1;

  for(int i = 0; i < rows; i++) {
    if(rowsums2.at<unsigned char>(i, 0) == 255) {
      staffrows.push_back(i);
      if(i - last != 1) {
        if(first != -1) {
          avgrows.push_back(((float)last + (float)first) / 2.0);
          staffbegins.push_back(first); // making a std::vector of all the beginning staffline locations
          staffends.push_back(last);    // making a std::vector of all the end staffline locations
        }
        first = i;
      }
      last = i;
    }
  }
  if(first != -1) {
    avgrows.push_back(((float)last + (float)first) / 2.0);
    staffbegins.push_back(first); // making a std::vector of all the beginning staffline locations
    staffends.push_back(last);    // making a std::vector of all the end staffline locations
  }

  // Now staffrows has all the rows with staff lines and avgrows has the center of these lines.

  if(avgrows.size() % 5 != 0) {
    cout << "An cv::error might have occured in staff cv::line detection.";
  }
  if(avgrows.size() < 5) {
    cout << "No staves were detected, exiting now.";
    return -1;
  }

  // Let's calculate average staff distance as the average between the first 5 lines
  // aka distance from cv::line 1 to cv::line 5 divided by 4

  float avgStaffDistance = (avgrows[4] - avgrows[0]) / 4.0;

  cout << "Average staff distance is " << avgStaffDistance << ".\n";

  int stavesInGroup = 1;
  int barLineIndex = 5;

  while(barLineIndex < avgrows.size()) {
    cv::Mat lilColSums;
    int found = 0;
    int startrow = (int)(avgrows[barLineIndex - 1] + avgStaffDistance);
    int endrow = (int)(avgrows[barLineIndex] - avgStaffDistance);
    int target = (endrow - startrow) * 8 / 10;
    cv::Mat testcv::Mat = bw.rowRange(startrow, endrow);
    reduce(testcv::Mat, lilColSums, 0, CV_REDUCE_SUM, CV_32SC1);
    for(int i = 0; i < cols; i++) {
      if(lilColSums.at<int>(0, i) > target) {
        found = 1;
        break;
      }
    }
    if(!found) {
      break;
    }
    stavesInGroup += 1;
    barLineIndex += 5;
  }

  // We find groups of staves (staves played together at the same time) by looking for vertical
  // lines joining adjacent staves.  The number of staves in the first group is stored in
  // "stavesInGroup". Right now we assume every group in the piece of music has the same number of
  // staves.

  cout << "Staves in group: " << stavesInGroup << "\n";
  cout << "Total staves: " << avgrows.size() / 5 << "\n";

  if(((avgrows.size() / 5) % stavesInGroup) != 0) {
    cout << "Bad number of stave groups, incompatible with total number of staves.  Exiting.";
    return -1;
  }

  barLineIndex = 0;
  int group = 0;

  // REMOVE STAVE REGIONS HERE

  cv::Mat cleanImage = imbw;

  // string ty =  type2str( cleanImage.type() );
  // printf("cv::Matrix: %s %dx%d \n", ty.c_str(), cleanImage.cols, cleanImage.rows);

  removeStaves(bw, staffbegins, staffends, cleanImage);

  cv::namedWindow("Before Image", cv::WINDOW_AUTOSIZE);
  cv::imshow("After", cleanImage);
  cv::waitKey();

  // NOW TRY BOUNDING BOX- use imbw for thresh

  int p = 0;

  while(barLineIndex < avgrows.size()) {
    int startrow = (int)(avgrows[barLineIndex] - 3 * avgStaffDistance);
    int endrow = (int)(avgrows[barLineIndex + 4] + 3 * avgStaffDistance);
    if(startrow < 0) {
      startrow = 0;
    }
    if(endrow > rows) {
      endrow = rows;
    }
    cv::Mat testcv::Mat = cleanImage.rowRange(startrow, endrow);

    // Anurag TODO: Here is the matrix with the cv::line.
    // avgStaffDistance has the average distance between lines in a staff.
    // avgrows is a std::vector with the approximate centers of all the staff lines.
    // avgrows[barLineIndex - barLineIndex+4] will have the centers of the region you're processing.
    // staffrows is like avgrows, but it has all the rows considered to be staff lines, in case
    //   staff lines are more than a pixel thick.  You can do some magic to find all the lines you
    //   need to erase.
    // stavesInGroup has the number of staves grouped together, group has the current group index.

    // The next 3 lines show you an easy way to display an image using opencv.

    // assumption, all staff lines will have the same thickness.

    // In order to remove staff lines for processing need to do the following things.

    // 1. Find thickness of the staff line

    // 2. traverse horizontally along each staff cv::line and remove staff cv::line,
    // unless pixel above thickness/2 from center or below thickness/2 from center
    // in which case you dont' remove it for now.

    /*
    findNotes(testcv::Mat,avgrows);
    cv::namedWindow( "Hough Circle Transform Demo", cv::WINDOW_AUTOSIZE );
    cv::imshow( "Hough Circle Transform Demo", testcv::Mat );
    cv::waitKey();
    */

    // findNotes(testcv::Mat,avgrows);
    createCookBook(testcv::Mat, p);
    p++;
    barLineIndex += 5;
    group = (group + 1) % stavesInGroup;
  }

  // now have a cookbook for quarter note and half note, lets do a rough template matching
  barLineIndex = 0;

  while(barLineIndex < avgrows.size()) {
    int startrow = (int)(avgrows[barLineIndex] - 3 * avgStaffDistance);
    int endrow = (int)(avgrows[barLineIndex + 4] + 3 * avgStaffDistance);
    if(startrow < 0) {
      startrow = 0;
    }
    if(endrow > rows) {
      endrow = rows;
    }
    cv::Mat testcv::Mat = cleanImage.rowRange(startrow, endrow);
    findNotes(testcv::Mat, avgrows);
    barLineIndex += 5;
    group = (group + 1) % stavesInGroup;
  }

  return 0;
}

void
removeStaves(cv::Mat image, Vector<int> firstVec, Vector<int> endVec, cv::Mat resultImage) {

  // loop through each first vec, make the lines white in result Image from firstVec value to endVec
  // value if pixel preceding first vec in image is not black or image succeeding end vec in image
  // is not black

  for(int i = 0; i < firstVec.size(); i++) {
    int sb = firstVec[i]; // index of beginning of staff cv::line area in image
    int se = endVec[i];

    for(int j = 0; j < image.size[1]; j++) {

      int a = (int)image.at<uchar>(sb - 1, j); // check pixel value above staff cv::line at column
      int b = (int)image.at<uchar>(se + 1, j); // check pixel value below staff cv::line at column

      if((a == 0) || (b == 0)) {
        // this means staff cv::line is surrounded by whitespace and is clear to erase at this spot
        int k = 0;
        int thick = se - sb + 1; // thick is thickness of current staffline
        while(thick) {
          resultImage.at<uchar>(sb + k, j) = 255;
          k++; // k placeholder index for iterating and whiting out staff line
          thick--;
        }
      }
    }
  }
}

void
createCookBook(cv::Mat staveReg, int rightIndex) {

  int thresh = 100;
  int max_thresh = 255;
  RNG rng(12345);
  cv::Mat threshold_output;
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy; // never used but needed as an input arg for some reason

  cv::threshold(staveReg, threshold_output, thresh, 255, THRESH_BINARY);
  /// Find contours
  cv::findContours(threshold_output, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

  /// Approximate contours to polygons + get bounding rects and circles
  std::vector<std::vector<cv::Point>> contours_poly(contours.size());
  std::vector<cv::Rect> boundRect(contours.size());

  for(int i = 0; i < contours.size(); i++) {
    cv::approxPolyDP(cv::Mat(contours[i]), contours_poly[i], 3, true);
    boundRect[i] = cv::boundingRect(cv::Mat(contours_poly[i]));
  }

  /// Draw polygonal contour + bonding rects + circles
  cv::Mat drawing = cv::Mat::zeros(threshold_output.size(), CV_8UC3);
  for(int i = 0; i < contours.size(); i++) {
    cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
    cv::drawContours(drawing, contours, i, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
  }

  /// Show in a window
  /*cv::namedWindow( "Contours", cv::WINDOW_AUTOSIZE );
     cv::imshow( "Contours", drawing );
     cv::waitKey();*/

  // Extract Templates for notes for now its hardcoded

  if(rightIndex == 0) {
    int startCol = boundRect[quarterIndex].tl().x;
    int endCol = boundRect[quarterIndex].br().x;
    int startRow = boundRect[quarterIndex].tl().y;
    int endRow = boundRect[quarterIndex].br().y;
    templateQuarter = staveReg.rowRange(startRow, endRow);
    templateQuarter = templateQuarter.colRange(startCol, endCol);

    /*cv::namedWindow( "Quarter Note", cv::WINDOW_AUTOSIZE );
    cv::imshow( "Quarter Note", templateQuarter);
    cv::waitKey();*/
  }

  if(rightIndex == 1) {
    int startCol = boundRect[halfIndex].tl().x;
    int endCol = boundRect[halfIndex].br().x;
    int startRow = boundRect[halfIndex].tl().y;
    int endRow = boundRect[halfIndex].br().y;
    templateHalf = staveReg.rowRange(startRow, endRow);
    templateHalf = templateHalf.colRange(startCol, endCol);

    /*cv::namedWindow( "Half Note", cv::WINDOW_AUTOSIZE );
    cv::imshow( "Half Note", templateHalf);
    cv::waitKey()*/
    ;
  }

  if(rightIndex == 6) {
    int startCol = boundRect[holeIndex].tl().x;
    int endCol = boundRect[holeIndex].br().x;
    int startRow = boundRect[holeIndex].tl().y;
    int endRow = boundRect[holeIndex].br().y;
    templateHole = staveReg.rowRange(startRow, endRow);
    templateHole = templateHole.colRange(startCol, endCol);
    /* cv::namedWindow( "Whole Note", cv::WINDOW_AUTOSIZE );
    cv::imshow( "Whole Note", templateHole);
    cv::waitKey()*/
    ;
  }
}
void
findNotes(cv::Mat staveReg, Vector<float> staveLoc) {

  int thresh = 100;
  int max_thresh = 255;
  RNG rng(12345);
  cv::Mat threshold_output;
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;

  cv::namedWindow("original", cv::WINDOW_AUTOSIZE);
  cv::imshow("Original Stave", staveReg);
  cv::waitKey();

  cv::threshold(staveReg, threshold_output, thresh, 255, THRESH_BINARY);
  /// Find contours
  cv::findContours(threshold_output, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

  /// Approximate contours to polygons + get bounding rects and circles
  std::vector<std::vector<cv::Point>> contours_poly(contours.size());
  std::vector<cv::Rect> boundRect(contours.size());

  for(int i = 0; i < contours.size(); i++) {
    cv::approxPolyDP(cv::Mat(contours[i]), contours_poly[i], 3, true);
    boundRect[i] = cv::boundingRect(cv::Mat(contours_poly[i]));
  }

  /// Draw polygonal contour + bonding rects + circles
  cv::Mat drawing = cv::Mat::zeros(threshold_output.size(), CV_8UC3);
  for(int i = 0; i < contours.size(); i++) {
    cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
    cv::rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0);
  }

  cv::namedWindow("Rectangles", cv::WINDOW_AUTOSIZE);
  cv::imshow("Corect", drawing);
  cv::waitKey();

  for(int i = 0; i < contours.size(); i++) {
    // now have all contours and all bounding rectangles for contours
    // screen for any rectangles that are of size smaller than the cookbook templates.
    // if of appropriate size, run localmin max stuff for template matching withing window vs
    // template to find matches. so for this

    // couple things to think about.
    // can continuously apply localminMax to the template until it falls below a certain threshold

    // for any cv::rectangle that you find a match above a certain cv::threshold, record a cv::rectangle with
    // coded colors, but eventually a note struct @ that location.
    cv::Mat result;
    int match_method = CV_TM_SQDIFF;

    matchTemplate(staveReg, templateQuarter, result, match_method);
    cv::normalize(result, result, 0, 1, NORM_MINMAX, -1, cv::Mat());
    char* result_window = "Result window";
    cv::namedWindow(result_window, cv::WINDOW_AUTOSIZE);
    cv::imshow(result_window, result);
    cv::waitKey();
    if(boundRect[i].area() >= templateQuarter.size[0] * templateQuarter.size[1] &&
       boundRect[i].area() <= 3 * (templateQuarter.size[0] * templateQuarter.size[1])) {
      // know you're in business

      // construct a new cv::Mat from boundRect coordinates in staveRegion

      cv::Mat regio;
      int startCol = boundRect[i].tl().x;
      int endCol = boundRect[i].br().x;
      int startRow = boundRect[i].tl().y;
      int endRow = boundRect[i].br().y;
      regio = staveReg.rowRange(startRow, endRow);
      regio = regio.colRange(startCol, endCol);

      int result_cols = regio.cols - templateQuarter.cols + 1;
      int result_rows = regio.rows - templateQuarter.rows + 1;
      result.create(result_cols, result_rows, CV_32FC1);

      matchTemplate(staveReg, templateQuarter, result, match_method);
      cv::normalize(result, result, 0, 1, NORM_MINMAX, -1, cv::Mat());

      char* result_window = "Result window";
      cv::namedWindow(result_window, cv::WINDOW_AUTOSIZE);
      cv::imshow(result_window, result);
      cv::waitKey();

      /// Localizing the best match with minMaxLoc
      double minVal;
      double maxVal;
      cv::Point minLoc;
      cv::Point maxLoc;
      cv::Point matchLoc;

      cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());

      /// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other
      /// methods, the higher the better
      if(match_method == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED) {
        matchLoc = minLoc;
      } else {
        matchLoc = maxLoc;
      }

      /// Show me what you got

      if(maxVal > NOTE_THRESHOLD) {

        cv::rectangle(staveReg,
                      cv::Point(matchLoc.x + boundRect[i].tl().x, boundRect[i].tl().y + matchLoc.y),
                      cv::Point(matchLoc.x + boundRect[i].br().x, boundRect[i].br().y + matchLoc.y),
                      cv::Scalar::all(0),
                      2,
                      8,
                      0);
      }

      cv::imshow("stave section", staveReg);
      cv::waitKey();
    }
  }
}

string
type2str(int type) {
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch(depth) {
    case CV_8U: r = "8U"; break;
    case CV_8S: r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default: r = "User"; break;
  }

  r += "C";
  r += (chans + '0');

  return r;
}
