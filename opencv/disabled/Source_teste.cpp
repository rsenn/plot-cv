#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

//using namespace cv;
using namespace std;

int
main(int argc, char** argv) {

  cv::VideoCapture cap(argv[1]);
  if(!cap.isOpened()) {
    cout << "Cannot open the video file" << endl;
    return -1;
  }

  double count = cap.get(cv::CAP_PROP_FRAME_COUNT); // get the frame count
  cout << count;
  cap.set(cv::CAP_PROP_POS_FRAMES, count - 1000); // Set index to last frame
  cv::namedWindow("MyVideo", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("Teste_Gray", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("Teste_GaussianBlur", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("Teste_Canny", cv::WINDOW_AUTOSIZE);

  while(1) {

    cv::Mat frame;
    bool success = cap.read(frame);
    if(!success) {
      cout << "Cannot cv::read  frame " << endl;
      break;
    }
    cv::imshow("MyVideo", frame);

    // parte de tu a tentar um grayscale
    cv::Mat frame_gray;
    cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);
    cv::imshow("Teste_Gray", frame_gray);
    // GaussianBlur
    cv::Mat frame_gaussian;
    cv::GaussianBlur(frame_gray, frame_gaussian, cv::Size(7, 7), 1.5, 1.5);
    cv::imshow("Teste_GaussianBlur", frame_gaussian);
    // cv::Canny edge
    cv::Mat frame_canny;
    cv::Canny(frame_gaussian, frame_canny, 0, 30, 3);
    cv::imshow("Teste_Canny", frame_canny);

    // http://docs.opencv.org/2.4/doc/tutorials/introduction/load_save_image/load_save_image.html

    cv::imwrite("C:/Users/User/Documents/Visual Studio "
            "2015/Projects/experiencia_opencv/data/Canny_Image.jpg",
            frame_canny);
    //  img = cvQueryFrame(frame_canny);
    //  cvSaveImage("frame/canny.jpg", img);

    if(cv::waitKey(0) == 27)
      break;
  }
}