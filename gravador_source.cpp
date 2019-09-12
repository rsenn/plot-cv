#include <opencv2\opencv.hpp>
#include <iostream>
#include <string>

using namespace cv;
using namespace std;

int
main(int argc, char** argv) {

  VideoCapture cap(argv[1]);
  if(!cap.isOpened()) {
    cout << "Cannot open the video file" << endl;
    return -1;
  }

  double count = cap.get(CV_CAP_PROP_FRAME_COUNT); // get the frame count

  double Pos = 0; // evitar o primeiro frame
  cout << count << endl;
  cap.set(CV_CAP_PROP_POS_FRAMES, Pos); // Set index to first frame (simbolo da playstation)
  /*namedWindow("MyVideo", CV_WINDOW_AUTOSIZE);
  namedWindow("Teste_Gray", CV_WINDOW_AUTOSIZE);
  namedWindow("Teste_GaussianBlur", CV_WINDOW_AUTOSIZE);
  namedWindow("Teste_Canny", CV_WINDOW_AUTOSIZE);
  */
  while(Pos <= count) {
    cap.set(CV_CAP_PROP_POS_FRAMES, Pos); // Set index to first frame (simbolo da playstation)
    Mat frame;
    bool success = cap.read(frame);
    if(!success) {
      cout << "Cannot read  frame " << endl;
      break;
    }
    // imshow("MyVideo", frame);

    // parte de tu a tentar um grayscale
    Mat frame_gray;
    cvtColor(frame, frame_gray, CV_BGR2GRAY);
    //	imshow("Teste_Gray", frame_gray);
    // GaussianBlur
    Mat frame_gaussian;
    GaussianBlur(frame_gray, frame_gaussian, Size(7, 7), 1.5, 1.5);
    //	imshow("Teste_GaussianBlur", frame_gaussian);
    // Canny edge
    Mat frame_canny;
    Canny(frame_gaussian, frame_canny, 0, 30, 3);
    //	imshow("Teste_Canny", frame_canny);

    // http://docs.opencv.org/2.4/doc/tutorials/introduction/load_save_image/load_save_image.html

    int Pos_int = Pos;

    string varAsString = to_string(Pos_int);

    String directory = "../../data/Output/Canny_Image_Teste" + varAsString + ".jpg";
    // cout << directory;

    imwrite(directory, frame_canny);
    //	img = cvQueryFrame(frame_canny);
    //	cvSaveImage("frame/canny.jpg", img);

    if(waitKey(0) == 27)
      break; // Não está a funcionar

    Pos = Pos + 10; // para não fazer todos os frames

    // cout << Pos_int << endl;
    if(Pos_int % 100 == 0) {
      cout << "Mais 100" << endl;
    }
  }
  cout << "Feito" << endl;
}