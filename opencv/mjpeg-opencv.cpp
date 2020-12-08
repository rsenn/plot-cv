
#include <fstream>
#include <iostream>
#include <vector>
#include <deque>
#include <windows.h>
#include <thread>

#include <stdio.h>
#include <string.h>
#include <ctype.h>

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <boost/thread/thread.hpp>
#include <boost/lockfree/spsc_queue.hpp>
#include <iostream>

#include <boost/atomic.hpp>
using namespace std;
using namespace cv;

VideoCapture cap;
Mat img, showimg;
HOGDescriptor hog;
deque<Mat> imgs;

size_t i, j;
boost::lockfree::spsc_queue<Mat, boost::lockfree::capacity<1024>> spsc_queue; /* spsc_queue用于图像的缓存，因为获取图像和处理图像都有较大的延时，所以采用生产者消费者模式，生产者即为一个专门处理获取图像的线程，消费者即为对图像处理的线程
                                                                               */

/*
 * ===  FUNCTION  ======================================================================
 *         Name:  producer
 *  Description:  简单的获取图像
 * =====================================================================================
 */
void
producer(void) {
  while(true) {
    cap.read(img);        /* 获取图像 */
    spsc_queue.push(img); /* 加入到缓存队列中 */
  }
}

boost::atomic<bool> done(false);

/*
 * ===  FUNCTION  ======================================================================
 *         Name:  consumer
 *  Description:  处理图像线程，计算hog和显示
 * =====================================================================================
 */
void
consumer(void) {

  while(true) {
    vector<Rect> found, found_filtered;
    spsc_queue.pop(showimg);
    hog.detectMultiScale(showimg, found, 0, Size(4, 4), Size(0, 0), 1.05, 2);
    for(i = 0; i < found.size(); i++) {
      Rect r = found[i];
      for(j = 0; j < found.size(); j++)
        if(j != i && (r & found[j]) == r)
          break;
      if(j == found.size())
        found_filtered.push_back(r);
    }
    for(i = 0; i < found_filtered.size(); i++) {
      Rect r = found_filtered[i];
      r.x += cvRound(r.width * 0.1);
      r.width = cvRound(r.width * 0.8);
      r.y += cvRound(r.height * 0.06);
      r.height = cvRound(r.height * 0.9);
      rectangle(showimg, r.tl(), r.br(), cv::Scalar(0, 255, 0), 1);
    }
    imshow("1", showimg);
    waitKey(5);
  }
}

/*
 * ===  FUNCTION  ======================================================================
 *         Name:  load_lear_model
 *  Description:  加载自己训练的hog 行人model (INRIA person detection database)
 *  参考：http://blog.youtueye.com/work/opencv-hog-peopledetector-trainning.html
 * =====================================================================================
 */
vector<float>
load_lear_model(const char* model_file) {
  vector<float> detector;
  FILE* modelfl;
  if((modelfl = fopen(model_file, "rb")) == NULL) {
    cout << "Unable to open the modelfile" << endl;
    return detector;
  }

  char version_buffer[10];
  if(!fread(&version_buffer, sizeof(char), 10, modelfl)) {
    cout << "Unable to read version" << endl;
    return detector;
  }

  if(strcmp(version_buffer, "V6.01")) {
    cout << "Version of model-file does not match version of svm_classify!" << endl;
    return detector;
  }
  // read version number
  int version = 0;
  if(!fread(&version, sizeof(int), 1, modelfl)) {
    cout << "Unable to read version number" << endl;
    return detector;
  }

  if(version < 200) {
    cout << "Does not support model file compiled for light version" << endl;
    return detector;
  }

  long kernel_type;
  fread(&(kernel_type), sizeof(long), 1, modelfl);

  {
    // ignore these
    long poly_degree;
    fread(&(poly_degree), sizeof(long), 1, modelfl);

    double rbf_gamma;
    fread(&(rbf_gamma), sizeof(double), 1, modelfl);

    double coef_lin;
    fread(&(coef_lin), sizeof(double), 1, modelfl);
    double coef_const;
    fread(&(coef_const), sizeof(double), 1, modelfl);

    long l;
    fread(&l, sizeof(long), 1, modelfl);
    char* custom = new char[l];
    fread(custom, sizeof(char), l, modelfl);
    delete[] custom;
  }

  long totwords;
  fread(&(totwords), sizeof(long), 1, modelfl);

  {
    // ignore these
    long totdoc;
    fread(&(totdoc), sizeof(long), 1, modelfl);

    long sv_num;
    fread(&(sv_num), sizeof(long), 1, modelfl);
  }

  double linearbias = 0.0;
  fread(&linearbias, sizeof(double), 1, modelfl);

  if(kernel_type == 0) { /* linear kernel */
    /* save linear wts also */
    double* linearwt = new double[totwords + 1];
    int length = totwords;
    fread(linearwt, sizeof(double), totwords + 1, modelfl);

    for(int i = 0; i < totwords; i++) {
      float term = linearwt[i];
      detector.push_back(term);
    }
    float term = -linearbias;
    detector.push_back(term);
    delete[] linearwt;

  } else {
    cout << "Only supports linear SVM model files" << endl;
  }

  fclose(modelfl);
  return detector;
}

int
main(int argc, char** argv) {
  cout << "boost::lockfree::queue is ";
  if(!spsc_queue.is_lock_free())
    cout << "not ";
  cout << "lockfree" << endl;
  vector<float> detector = load_lear_model(argv[1]); /* load model */
  hog.setSVMDetector(detector);

  cap.open("d://1.avi");
  // cap.open("http://10.104.5.192:8888/?action=stream?dummy=param.mjpg");//get image by mjpeg
  // stream
  cap.set(cv::CAP_PROP_FRAME_WIDTH, 320);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, 240);
  if(!cap.isOpened())
    return -1;

  thread mThread(producer);
  Sleep(5000); /* 让生产者先生产一会儿 */

  thread mThread2(consumer);
  mThread.join();
  mThread2.join();

  return 0;
}
