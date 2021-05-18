/*************************************************************************
 *Author: Joseph Gonzales
 *Course: COSC 4590-001
 *Final Project
 *************************************************************************/

#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
//using namespace cv;

int thresh = 175;
int max_thresh = 250;
int ratio = 3;

cv::Mat cannyThreshold(Mat& img, int e);
cv::Mat morph(Mat& img);
cv::Mat blue(Mat& img);
void writeTo(cv::Mat& img, string& name);

// main
int
main(/*int argc, char** argv*/) {
  /*if(!strcmp(argv[1], "-h") || !strcmp(argv[1], "--help")) cout << "\nHelp: status flags\nargument 1:\n-h
  or --help: to display this help information\nProgram usage:\nThe program will take a image in and give the
  user a range of choices for modifying the image.\nEnter a value to modify the image; some modifiers will
  prompt for a cv::threshold value between 1 - 255"; else	{*/
  // Create 3 matrixes, one for the source image, one for color convert, and one for any modifications
  char input = '7';
  string name;

  do {
    cv::Mat src, gray, mod;

    cout << "Please enter the name of an image to be cv::read: ";
    string arg;
    cin >> arg;
    src = cv::imread(arg);
    if(src.empty()) {
      cout << "Image could not be cv::read" << endl;
      return -1;
    }
    // remove any file name extensions such as .png or .jpg
    arg.replace(arg.length() - 4, arg.length(), "");

    // Convert source to gray
    cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
    do {
      mod = gray.clone();
      // Prompt
      cout << "Enter a number to select the methods that will process this image\n1. Morphologically "
              "Close\n2. cv::Canny "
              "Edge Detection\n3. cv::Canny + Ellipse fitting\n4. Close + cv::Canny + Ellipse\n5. Blue removal\n6. "
              "Load a new "
              "image\n7. Exit the program"
           << endl;
      cin >> input;

      cv::namedWindow("Source", cv::WINDOW_AUTOSIZE);
      cv::imshow("Source", src);

      switch(input) {
        case '1': // Morphologically close only
          mod = morph(gray);
          cv::waitKey(0);
          name = arg + "_closed.png";
          break;
        case '2': // cv::Canny edge detect
          mod = cannyThreshold(gray, 0);
          name = arg + "_threshold.png";
          break;
        case '3': // cv::Canny edge detect with cv::ellipse fitting
          mod = cannyThreshold(gray, 1);
          name = arg + "_threshold-cv::ellipse.png";
          break;
        case '4': // Morphological closing with canny edge detect and cv::ellipse fitting
          mod = morph(gray);
          mod = cannyThreshold(mod, 1);
          name = arg + "_closed-cv::threshold-cv::ellipse.png";
          break;
        case '5': // Remove blue from an image
          mod = blue(src);
          name = arg + "_blue.png";
          break;
        case '6':
        case '7': break;
        default: std::cout << "please enter a value from 1-8" << endl;
      }
      if(input == '1' || input == '2' || input == '3' || input == '4' || input == '5')
        writeTo(mod, name);
    } while(input != '6' && input != '7');
  } while(input == '6');
  return 0;
}

// morphologically close the image
Mat
morph(cv::Mat& img) {
  // Create structured element
  cv::Mat dst, element = cv::getStructuringElement(MORPH_ELLIPSE, cv::Size(3, 3), cv::Point(-1, -1));

  // Apply the closed morphology operation
  cv::morphologyEx(img, dst, 2, element);
  cv::namedWindow("morphed", cv::WINDOW_AUTOSIZE);
  cv::imshow("morphed", dst);
  return dst;
}

// apply canny cv::threshold and cv::ellipse fitting depending on the user input
Mat
cannyThreshold(cv::Mat& img, int e) {
  cv::Mat edges, dst;
  vector<vector<cv::Point>> contours;
  vector<cv::Vec4i> hierarchy;

  // Reduce noise with a kernel 3x3
  cv::GaussianBlur(img, dst, cv::Size(3, 3), 0, 0);

  char choice;
  do {
    cout << "Enter a cv::threshold: ";
    cin >> thresh;
    // Detect edges using canny
    cv::Canny(img, edges, thresh, thresh * 3, 3);

    // Find contours
    cv::findContours(edges, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, cv::Point(-1, -1));

    // Using cv::Canny's output as a mask
    dst = cv::Scalar::all(0);
    // copies then inverts the outline
    img.copyTo(dst, edges);
    cv::bitwise_not(dst, dst);

    cv::namedWindow("cv::Canny Edge Detect", cv::WINDOW_AUTOSIZE);
    cv::imshow("cv::Canny Edge Detect", dst);
    cv::waitKey(0);

    cv::cvtColor(dst, dst, cv::COLOR_GRAY2BGR);
    if(e == 1) {
      vector<cv::RotatedRect> minEllipse(contours.size());
      int sizeMin = 5, sizeMax = 100;
      cout << "Set the minimum cv::ellipse size number (default 5): ";
      cin >> sizeMin;
      cout << "Set the maximum cv::ellipse size number (default 100): ";
      cin >> sizeMax;
      // checks for minimum and max sizes
      if(sizeMin >= sizeMax) {
        cout << "Minimum can not be greater than or equal to Maximum\nUsing defaults" << endl;
        sizeMin = 5;
        sizeMax = 100;
      }
      // Fit ellipses to contours based on the contours and the allotted size of the ellipse
      for(int i = 0; i < contours.size(); i++) {
        if(contours[i].size() >= sizeMin && contours[i].size() < sizeMax)
          minEllipse[i] = cv::minAreaRect(cv::Mat(contours[i]));
      }

      // Draw contours+ ellipses
      for(int i = 0; i < contours.size(); i++) {
        cv::Scalar color = cv::Scalar(255, 0, 0);
        // Draw ellipse
        cv::ellipse(dst, minEllipse[i], color, 2, 8);
      }

      cv::namedWindow("Ellipse fit", cv::WINDOW_AUTOSIZE);
      cv::imshow("Ellipse fit", dst);
      // cv::waitKey(0);
    }
    cv::waitKey(0);
    cout << "try again? (n to continue the program)" << endl;
    cin >> choice;
  } while(choice != 'n' && choice != 'N');
  return dst;
}

// method to change every blue pixel value available to white
Mat
blue(cv::Mat& img) {
  cv::Mat dst = img.clone();

  // Replace all blue pixels with white pixels
  for(int x = 0; x < img.rows; x++) {
    for(int y = 0; y < img.cols; y++) {
      if(img.at<Vec3b>(x, y) == Vec3b(255, 0, 0)) {
        // combination of all 255 creates white
        dst.at<Vec3b>(x, y)[0] = 255;
        dst.at<Vec3b>(x, y)[1] = 255;
        dst.at<Vec3b>(x, y)[2] = 255;
      }
    }
  }
  cv::namedWindow("Blue removal", cv::WINDOW_AUTOSIZE);
  cv::imshow("Blue removal", dst);
  cv::waitKey(0);
  return dst; // return
}

// save images as PNG files
void
writeTo(cv::Mat& img, string& name) {
  char confirm;

  cout << "Would you like to save this as a JPEG file? (y to confirm)" << endl; // Asks to save
  cin >> confirm;
  // If y then save image
  if(confirm == 'y' || confirm == 'Y') {
    // Vector that stores the compression parameters of the image
    vector<int> compression_params;
    // Specify the image compression technique
    compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
    // Specify the compression quality
    compression_params.push_back(0);
    // Write the image to file
    cv::imwrite(name, img, compression_params);
    // Report
    cout << "image saved" << endl;
  }

  return;
}
