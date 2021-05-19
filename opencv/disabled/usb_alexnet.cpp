/* [Thanh] La partie concernant OpenCV-CNN est inspire du site officiel
    Credits: http://docs.opencv.org/trunk/d5/de7/tutorial_dnn_googlenet.html
*/

#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
//using namespace cv;
//using namespace cv::dnn;

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <string>
using namespace std;

/* [T] : Voir Credits pour la source
Find best class for the blob (i. e. class with maximal probability) */
void
getMaxClass(cv::dnn::Blob& probBlob, int* classId, double* classProb) {
  cv::Mat probMat = probBlob.matRefConst(); // reshape the blob to 1x1000 matrix
  cv::Point classNumber;

  cv::minMaxLoc(probMat, NULL, classProb, NULL, &classNumber);
  *classId = classNumber.x;
}

/* [T] : Voir Credits pour la source */
std::vector<cv::String>
readClassNames(const char* filename) {
  std::vector<cv::String> classNames;
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
texte(cv::Mat& image, const cv::String& txt, int x, int y) {
  cv::putText(image, txt, cv::Point(x, y), FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(0, 0, 255), 1, 25);
}

int
main() {
  cv::Mat matImg;       // Une image matricielle
  char key;         // Un input keyboard
  int classId;      // ID classe pour le CNN
  double classProb; // Probabilite de la prediction
  string s1;
  string s2;

  /* Initialisation du CNN AlexNet */

  // [T] : Je ne sais pas comment on a installe la lib, donc je laisse ca ici
  initModule(); // Required if OpenCV is built as static libs

  cv::String modelTxt = "models/bvlc_alexnet/bvlc_alexnet.prototxt";
  cv::String modelBin = "models/bvlc_alexnet/bvlc_alexnet.caffemodel";

  //! [Create the importer of Caffe model]
  Ptr<cv::dnn::Importer> importer;
  try { // Try to import Caffe GoogleNet model
    importer = cv::dnn::createCaffeImporter(modelTxt, modelBin);
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

  // [T] : Creation du 'net' du CNN depuis les fichiers CAFFE
  cv::dnn::Net net;
  importer->populateNet(net);
  importer.release();

  // Initialisation de la capture par USB
  cv::VideoCapture cap(0);
  if(!cap.isOpened()) {
    std::cout << "Erreur video" << std::endl;
    return -1;
  }

  // Récupérer une image et l'afficher
  cv::namedWindow("AlexNet", WINDOW_NORMAL); // Créé une fenêtre

  // Affiche les images une par une
  while(key != 'q' && key != 'Q') {

    // On récupère une image
    cap.read(matImg);

    cv::resize(matImg, matImg, cv::Size(227, 227));

    cv::dnn::Blob inputBlob = cv::dnn::Blob::fromImages(matImg);
    net.setBlob(".data", inputBlob);
    net.forward();
    cv::dnn::Blob prob = net.getBlob("prob");

    getMaxClass(prob, &classId, &classProb); // Recherche de la plus forte probabilite
    std::vector<cv::String> classNames = readClassNames("models/synset_words.txt");

    s1 = classNames.at(classId);
    s2 = "Probabilite " + std::to_string(classProb * 100) + " %";

    texte(matImg, s1, 0, 15);
    texte(matImg, s2, 0, 35);

    // On affiche l'image dans une fenêtre
    cv::imshow("AlexNet", matImg);

    // On attend 10ms
    key = cvWaitKey(10);
  }

  // Ou CvDestroyWindow("Window") si on veut garder d'autres fenêtres
  cvDestroyAllWindows();

  return 0;
}
