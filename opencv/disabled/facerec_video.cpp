/*
 * Copyright (c) 2011. Philipp Wagner <bytefish[at]gmx[dot]de>.
 * Released to public domain under terms of the BSD Simplified license.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the organization nor the names of its contributors
 *     may be used to endorse or promote products derived from this software
 *     without specific prior written permission.
 *
 *   See <http://www.opensource.org/licenses/bsd-license>
 */

#include "opencv2/core/core.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/objdetect/objdetect.hpp"

#include <iostream>
#include <fstream>
#include <sstream>

// using namespace cv;
using namespace std;

static void
read_csv(const string& filename, vector<cv::Mat>& images, vector<int>& labels, char separator = ';') {
  std::ifstream file(filename.c_str(), ifstream::in);
  if(!file) {
    string error_message = "No valid input file was given, please check the given filename.";
    CV_Error(CV_StsBadArg, error_message);
  }
  string cv::line, path, classlabel;
  while(getline(file, cv::line)) {
    stringstream liness(cv::line);
    getline(liness, path, separator);
    getline(liness, classlabel);
    if(!path.empty() && !classlabel.empty()) {
      images.push_back(cv::imread(path, 0));
      labels.push_back(atoi(classlabel.c_str()));
    }
  }
}

int
main(int argc, const char* argv[]) {
  // Check for valid command cv::line arguments, print usage
  // if no arguments were given.
  if(argc != 4) {
    cout << "usage: " << argv[0] << " </path/to/haar_cascade> </path/to/csv.ext> </path/to/device id>" << endl;
    cout << "\t </path/to/haar_cascade> -- Path to the Haar Cascade for cv::face detection." << endl;
    cout << "\t </path/to/csv.ext> -- Path to the CSV file with the cv::face database." << endl;
    cout << "\t <device id> -- The webcam device id to grab frames from." << endl;
    exit(1);
  }
  // Get the path to your CSV:
  string fn_haar = string(argv[1]);
  string fn_csv = string(argv[2]);
  int deviceId = atoi(argv[3]);
  // These vectors hold the images and corresponding labels:
  vector<cv::Mat> images;
  vector<int> labels;
  // Read in the data (fails if no valid input filename is given, but you'll get an cv::error message):
  try {
    read_csv(fn_csv, images, labels);
  } catch(cv::Exception& e) {
    cerr << "cv::Error opening file \"" << fn_csv << "\". Reason: " << e.msg << endl;
    // nothing more we can do
    exit(1);
  }
  // Get the height from the first image. We'll need this
  // later in code to reshape the images to their original
  // size AND we need to reshape incoming faces to this size:
  int im_width = images[0].cols;
  int im_height = images[0].rows;
  // Create a FaceRecognizer and train it on the given images:
  Ptr<FaceRecognizer> model = createFisherFaceRecognizer();
  model->train(images, labels);
  // That's it for learning the Face Recognition model. You now
  // need to create the classifier for the task of Face Detection.
  // We are going to use the haar cascade you have specified in the
  // command cv::line arguments:
  //
  cv::CascadeClassifier haar_cascade;
  haar_cascade.load(fn_haar);
  // Get a handle to the Video device:
  cv::VideoCapture cap(deviceId);
  // Check if we can use this device at all:
  if(!cap.isOpened()) {
    cerr << "Capture Device ID " << deviceId << "cannot be opened." << endl;
    return -1;
  }
  // Holds the current frame from the Video device:
  cv::Mat frame;
  for(;;) {
    cap >> frame;
    // Clone the current frame:
    cv::Mat original = frame.clone();
    // Convert the current frame to grayscale:
    cv::Mat gray;
    cv::cvtColor(original, gray, cv::COLOR_BGR2GRAY);
    // Find the faces in the frame:
    vector<Rect_<int>> faces;
    haar_cascade.detectMultiScale(gray, faces);
    // At this point you have the position of the faces in
    // faces. Now we'll get the faces, make a prediction and
    // annotate it in the video. Cool or what?
    for(int i = 0; i < faces.size(); i++) {
      // Process cv::face by cv::face:
      cv::Rect face_i = faces[i];
      // Crop the cv::face from the image. So simple with OpenCV C++:
      cv::Mat cv::face = gray(face_i);
      // Resizing the cv::face is necessary for Eigenfaces and Fisherfaces. You can easily
      // verify this, by reading through the cv::face recognition tutorial coming with OpenCV.
      // Resizing IS NOT NEEDED for Local Binary Patterns Histograms, so preparing the
      // input data really depends on the algorithm used.
      //
      // I strongly encourage you to play around with the algorithms. See which work best
      // in your scenario, LBPH should always be a contender for robust cv::face recognition.
      //
      // Since I am showing the Fisherfaces algorithm here, I also show how to cv::resize the
      // cv::face you have just found:
      cv::Mat face_resized;
      cv::resize(cv::face, face_resized, cv::Size(im_width, im_height), 1.0, 1.0, INTER_CUBIC);
      // Now perform the prediction, see how easy that is:
      int prediction = model->predict(face_resized);
      // And finally cv::write all we've found out to the original image!
      // First of all draw a green cv::rectangle around the detected cv::face:
      cv::rectangle(original, face_i, CV_RGB(0, 255, 0), 1);
      // Create the text we will annotate the box with:
      string box_text = cv::format("Prediction = %d", prediction);
      // Calculate the position for annotated text (make sure we don't
      // put illegal values in there):
      int pos_x = std::max(face_i.tl().x - 10, 0);
      int pos_y = std::max(face_i.tl().y - 10, 0);
      // And now put it into the image:
      cv::putText(original, box_text, cv::Point(pos_x, pos_y), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0, 255, 0), 2.0);
    }
    // Show the result:
    cv::imshow("face_recognizer", original);
    // And display it:
    char key = (char)cv::waitKey(20);
    // Exit this loop on escape:
    if(key == 27)
      break;
  }
  return 0;
}
