// Yannick Verdie 2010
// --- Please cv::read help() below: ---

#include <iostream>
#include <vector>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/legacy/compat.hpp>

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#if defined _MSC_VER && _MSC_VER >= 1600
#include <windows.h>
#endif
#include <GL/gl.h>
#endif

using namespace std;
//using namespace cv;

static void
help() {
  cout << "This demo demonstrates the use of the Qt enhanced version of the highgui GUI interface\n"
          "and dang if it doesn't throw in the use of of the POSIT 3D tracking algorithm too\n"
          "It works off of the video: cube4.avi\n"
          "Using OpenCV version "
       << CV_VERSION
       << "\n\n"

          " 1) This demo is mainly based on work from Javier Barandiaran Martirena\n"
          "    See this page http://code.opencv.org/projects/opencv/wiki/Posit.\n"
          " 2) This is a demo to illustrate how to use **OpenGL Callback**.\n"
          " 3) You need Qt binding to compile this sample with OpenGL support enabled.\n"
          " 4) The features' detection is very basic and could highly be improved\n"
          "    (basic thresholding tuned for the specific video) but 2).\n"
          " 5) Thanks to Google Summer of Code 2010 for supporting this work!\n"
       << endl;
}

#define FOCAL_LENGTH 600
#define CUBE_SIZE 0.5

static void
renderCube(float size) {
  glBegin(GL_QUADS);
  // Front Face
  glNormal3f(0.0f, 0.0f, 1.0f);
  glVertex3f(0.0f, 0.0f, 0.0f);
  glVertex3f(size, 0.0f, 0.0f);
  glVertex3f(size, size, 0.0f);
  glVertex3f(0.0f, size, 0.0f);
  // Back Face
  glNormal3f(0.0f, 0.0f, -1.0f);
  glVertex3f(0.0f, 0.0f, size);
  glVertex3f(0.0f, size, size);
  glVertex3f(size, size, size);
  glVertex3f(size, 0.0f, size);
  // Top Face
  glNormal3f(0.0f, 1.0f, 0.0f);
  glVertex3f(0.0f, size, 0.0f);
  glVertex3f(size, size, 0.0f);
  glVertex3f(size, size, size);
  glVertex3f(0.0f, size, size);
  // Bottom Face
  glNormal3f(0.0f, -1.0f, 0.0f);
  glVertex3f(0.0f, 0.0f, 0.0f);
  glVertex3f(0.0f, 0.0f, size);
  glVertex3f(size, 0.0f, size);
  glVertex3f(size, 0.0f, 0.0f);
  // Right face
  glNormal3f(1.0f, 0.0f, 0.0f);
  glVertex3f(size, 0.0f, 0.0f);
  glVertex3f(size, 0.0f, size);
  glVertex3f(size, size, size);
  glVertex3f(size, size, 0.0f);
  // Left Face
  glNormal3f(-1.0f, 0.0f, 0.0f);
  glVertex3f(0.0f, 0.0f, 0.0f);
  glVertex3f(0.0f, size, 0.0f);
  glVertex3f(0.0f, size, size);
  glVertex3f(0.0f, 0.0f, size);
  glEnd();
}

static void
on_opengl(void* param) {
  // Draw the object with the estimated pose
  glLoadIdentity();
  glScalef(1.0f, 1.0f, -1.0f);
  glMultcv::Matrixf((float*)param);
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE);
  renderCube(CUBE_SIZE);
  glDisable(GL_BLEND);
  glDisable(GL_LIGHTING);
}

static void
initPOSIT(std::vector<Cvcv::Point3D32f>* modelcv::Points) {
  // Create the model pointss
  modelcv::Points->push_back(cvcv::Point3D32f(0.0f, 0.0f, 0.0f)); // The first must be (0, 0, 0)
  modelcv::Points->push_back(cvcv::Point3D32f(0.0f, 0.0f, CUBE_SIZE));
  modelcv::Points->push_back(cvcv::Point3D32f(CUBE_SIZE, 0.0f, 0.0f));
  modelcv::Points->push_back(cvcv::Point3D32f(0.0f, CUBE_SIZE, 0.0f));
}

static void
foundCorners(std::vector<Cvcv::Point2D32f>* srcImagecv::Points, const cv::Mat& source, cv::Mat& grayImage) {
  cv::cvtColor(source, grayImage, cv::COLOR_RGB2GRAY);
  cv::GaussianBlur(grayImage, grayImage, cv::Size(11, 11), 0, 0);
  cv::normalize(grayImage, grayImage, 0, 255, NORM_MINMAX);
  cv::threshold(grayImage, grayImage, 26, 255, THRESH_BINARY_INV); // 25

  cv::Mat MgrayImage = grayImage;
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(MgrayImage, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

  cv::Point p;
  std::vector<Cvcv::Point2D32f> srcImagecv::Points_temp(4, cvcv::Point2D32f(0, 0));

  if(contours.size() == srcImagecv::Points_temp.size()) {
    for(size_t i = 0; i < contours.size(); i++) {
      p.x = p.y = 0;

      for(size_t j = 0; j < contours[i].size(); j++) p += contours[i][j];

      srcImagecv::Points_temp.at(i) = cvcv::Point2D32f(float(p.x) / contours[i].size(), float(p.y) / contours[i].size());
    }

    // Need to keep the same order
    // > y = 0
    // > x = 1
    // < x = 2
    // < y = 3

    // get point 0;
    size_t index = 0;
    for(size_t i = 1; i < srcImagecv::Points_temp.size(); i++)
      if(srcImagecv::Points_temp.at(i).y > srcImagecv::Points_temp.at(index).y)
        index = i;
    srcImagecv::Points->at(0) = srcImagecv::Points_temp.at(index);

    // get point 1;
    index = 0;
    for(size_t i = 1; i < srcImagecv::Points_temp.size(); i++)
      if(srcImagecv::Points_temp.at(i).x > srcImagecv::Points_temp.at(index).x)
        index = i;
    srcImagecv::Points->at(1) = srcImagecv::Points_temp.at(index);

    // get point 2;
    index = 0;
    for(size_t i = 1; i < srcImagecv::Points_temp.size(); i++)
      if(srcImagecv::Points_temp.at(i).x < srcImagecv::Points_temp.at(index).x)
        index = i;
    srcImagecv::Points->at(2) = srcImagecv::Points_temp.at(index);

    // get point 3;
    index = 0;
    for(size_t i = 1; i < srcImagecv::Points_temp.size(); i++)
      if(srcImagecv::Points_temp.at(i).y < srcImagecv::Points_temp.at(index).y)
        index = i;
    srcImagecv::Points->at(3) = srcImagecv::Points_temp.at(index);

    cv::Mat Msource = source;
    stringstream ss;
    for(size_t i = 0; i < srcImagecv::Points_temp.size(); i++) {
      ss << i;
      cv::circle(Msource, srcImagecv::Points->at(i), 5, cv::Scalar(0, 0, 255));
      cv::putText(Msource, ss.str(), srcImagecv::Points->at(i), FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255));
      ss.str("");

      // new coordinate system in the middle of the frame and reversed (camera coordinate system)
      srcImagecv::Points->at(i) = cvcv::Point2D32f(srcImagecv::Points_temp.at(i).x - source.cols / 2,
                                                   source.rows / 2 - srcImagecv::Points_temp.at(i).y);
    }
  }
}

static void
createOpenGLcv::MatrixFrom(float* posePOSIT, const Cvcv::Matr32f& rotationcv::Matrix, const CvVect32f& translationVector) {
  // coordinate system returned is relative to the first 3D input point
  for(int f = 0; f < 3; f++)
    for(int c = 0; c < 3; c++) posePOSIT[c * 4 + f] = rotationcv::Matrix[f * 3 + c]; // transposed

  posePOSIT[3] = translationVector[0];
  posePOSIT[7] = translationVector[1];
  posePOSIT[11] = translationVector[2];
  posePOSIT[12] = 0.0f;
  posePOSIT[13] = 0.0f;
  posePOSIT[14] = 0.0f;
  posePOSIT[15] = 1.0f;
}

int
main(void) {
  help();

  string fileName = "cube4.avi";
  cv::VideoCapture video(fileName);
  if(!video.isOpened()) {
    cerr << "Video file " << fileName << " could not be opened" << endl;
    return EXIT_FAILURE;
  }

  cv::Mat source, grayImage;
  video >> source;

  cv::namedWindow("Original", cv::WINDOW_AUTOSIZE | cv::WINDOW_FREERATIO);
  cv::namedWindow("POSIT", WINDOW_OPENGL | cv::WINDOW_FREERATIO);
  cv::resizeWindow("POSIT", source.cols, source.rows);

  cv::displayOverlay("POSIT",
                 "We lost the 4 corners' detection quite often (the red circles disappear).\n"
                 "This demo is only to illustrate how to use OpenGL callback.\n"
                 " -- Press ESC to exit.",
                 10000);

  float OpenGLcv::Matrix[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  setOpenGlContext("POSIT");
  setOpenGlDrawCallback("POSIT", on_opengl, OpenGLcv::Matrix);

  std::vector<Cvcv::Point3D32f> modelcv::Points;
  initPOSIT(&modelcv::Points);

  // Create the POSIT object with the model points
  CvPOSITObject* positObject = cvCreatePOSITObject(&modelcv::Points[0], (int)modelcv::Points.size());

  Cvcv::Matr32f rotation_matrix = new float[9];
  CvVect32f translation_std::vector = new float[3];
  CvTermCriteria criteria = cvTermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 100, 1e-4f);
  std::vector<Cvcv::Point2D32f> srcImagecv::Points(4, cvcv::Point2D32f(0, 0));

  while(cv::waitKey(33) != 27) {
    video >> source;
    if(source.empty())
      break;

    cv::imshow("Original", source);

    foundCorners(&srcImagecv::Points, source, grayImage);
    cvPOSIT(positObject, &srcImagecv::Points[0], FOCAL_LENGTH, criteria, rotation_matrix, translation_std::vector);
    createOpenGLcv::MatrixFrom(OpenGLcv::Matrix, rotation_matrix, translation_std::vector);

    updateWindow("POSIT");

    if(video.get(cv::CAP_PROP_POS_AVI_RATIO) > 0.99)
      video.set(cv::CAP_PROP_POS_AVI_RATIO, 0);
  }

  setOpenGlDrawCallback("POSIT", NULL, NULL);
  cv::destroyAllWindows();
  cvReleasePOSITObject(&positObject);

  delete[] rotation_matrix;
  delete[] translation_std::vector;

  return EXIT_SUCCESS;
}
