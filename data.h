#pragma once
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std; 

#include "polygon.h"
enum SegmentationMode {
	BALLOON_MODE,
	CURV_MODE,
	ALL_MODE
};

class Data {

public: 
  Data(Mat &Image);
  Mat image; // Image originale
  Mat bwImage; // Image convertie en noir et blanc.
  Mat gradient; //Norme du gradient
  Mat gGradient; //g(Norme du gradient)
  Mat gx; // gradient en x de g
  Mat gy;  // gradient en y de g
  Polygon polygon; // polygone lié à l'image

  void drawNextStep(double step, const Mat &Image, SegmentationMode mode);
  void findContour(double step, SegmentationMode mode);
  bool isValidPoint(Point2d p);
  //Modigie le polygone en utilisant une descente de gradient et compris dans les bords de l'image. 
};