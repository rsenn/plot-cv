/*
    Credits: http://docs.opencv.org/trunk/d5/de7/tutorial_dnn_googlenet.html
*/

#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
using namespace cv;
using namespace cv::dnn;

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <stdio.h>
using namespace std;

#define CAMERA_TO_USE PLEASE_FILL_ME

/* Please refer to credits
Find best class for the blob (i. e. class with maximal probability) */
void
getMaxClass(dnn::Blob& probBlob, int* classId, double* classProb) {
  Mat probMat = probBlob.matRefConst(); // reshape the blob to 1x1000 matrix
  Point classNumber;

  minMaxLoc(probMat, NULL, classProb, NULL, &classNumber);
  *classId = classNumber.x;
}

/* Please refer to credits */
std::vector<String>
readClassNames(const char* filename) {
  std::vector<String> classNames;
  std::ifstream fp(filename);
  if(!fp.is_open()) {
    std::cerr << "File with classes labels not found: " << filename << std::endl;
    exit(-1);
  }
  std::string name;
  while(!fp.eof()) {
    std::getline(fp, name);
    if(name.length())
      classNames.push_back(name.substr(name.find(' ') + 1));
  }
  fp.close();
  return classNames;
}

void
texte(Mat& image, const String& txt, int x, int y) {
  putText(image, txt, Point(x, y), FONT_HERSHEY_SIMPLEX, 0.65, Scalar(0, 0, 255), 1, 25);
}

int
main() {
  Mat matImg;       // A matricial image
  char key;         // Keyboard input
  int classId;      // Class ID for the CNN
  double classProb; // Prediction probability
  string s1;
  string s2;

  /* Initialization of AlexNet CNN */

  initModule(); // Required if OpenCV is built as static libs

  String modelTxt = "models/bvlc_alexnet/bvlc_alexnet.prototxt";
  String modelBin = "models/bvlc_alexnet/bvlc_alexnet.caffemodel";

  //! [Create the importer of Caffe model]
  Ptr<dnn::Importer> importer;
  try { // Try to import Caffe GoogleNet model
    importer = dnn::createCaffeImporter(modelTxt, modelBin);
  } catch(const cv::Exception& err) { // Importer can throw errors, we will catch them
    std::cerr << err.msg << std::endl;
  }
  //! [Create the importer of Caffe model]

  if(!importer) {
    std::cerr << "Can't load network by using the following files: " << std::endl;
    std::cerr << "prototxt:   " << modelTxt << std::endl;
    std::cerr << "caffemodel: " << modelBin << std::endl;
    exit(-1);
  }

  // Initialization of CNN net from CAFFE files
  dnn::Net net;
  importer->populateNet(net);
  importer.release();

  // Initialization of the video camera flow
  VideoCapture capture;
  capture.open(CAMERA_TO_USE);

  if(!capture.isOpened()) {
    printf("Ouverture du flux vidéo impossible !\n");
    return -1;
  }

  // Initialization of the window which will contains the picture
  namedWindow("AlexNet", WINDOW_NORMAL);

  // While the key pressed is not q (for quit)
  while(key != 'q' && key != 'Q') {

    // Get the next frame received
    capture.read(matImg);

    // AlexNet CNN network only accepts 227x227 pictures
    resize(matImg, matImg, Size(227, 227));

    // Initialization of the data blob
    dnn::Blob inputBlob = dnn::Blob::fromImages(matImg);
    net.setBlob(".data", inputBlob);
    net.forward();                        // We launch the CNN
    dnn::Blob prob = net.getBlob("prob"); // Retrieve the final prob

    getMaxClass(prob, &classId, &classProb);
    std::vector<String> classNames = readClassNames("models/synset_words.txt");

    s1 = classNames.at(classId);
    s2 = "Probability " + std::to_string(classProb * 100) + " %";

    texte(matImg, s1, 0, 15);
    texte(matImg, s2, 0, 35);

    imshow("AlexNet", matImg);

    key = cvWaitKey(10);
  }

  cvDestroyAllWindows();

  return 0;
}
