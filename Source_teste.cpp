#include<opencv2\opencv.hpp>
#include<iostream>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{

	VideoCapture cap(argv[1]);
	if (!cap.isOpened()) {
		cout << "Cannot open the video file" << endl;
		return -1;
	}

	double count = cap.get(CV_CAP_PROP_FRAME_COUNT); //get the frame count
	cout << count;
	cap.set(CV_CAP_PROP_POS_FRAMES, count - 1000); //Set index to last frame
	namedWindow("MyVideo", CV_WINDOW_AUTOSIZE);
	namedWindow("Teste_Gray", CV_WINDOW_AUTOSIZE);
	namedWindow("Teste_GaussianBlur", CV_WINDOW_AUTOSIZE);
	namedWindow("Teste_Canny", CV_WINDOW_AUTOSIZE);

	while (1)
	{

		Mat frame;
		bool success = cap.read(frame);
		if (!success) {
			cout << "Cannot read  frame " << endl;
			break;
		}
		imshow("MyVideo", frame);

		// parte de tu a tentar um grayscale
		Mat frame_gray;
		cvtColor(frame, frame_gray, CV_BGR2GRAY);
		imshow("Teste_Gray", frame_gray);
		//GaussianBlur
		Mat frame_gaussian;
		GaussianBlur(frame_gray, frame_gaussian, Size(7, 7), 1.5, 1.5);
		imshow("Teste_GaussianBlur", frame_gaussian);
		//Canny edge
		Mat frame_canny;
		Canny(frame_gaussian, frame_canny, 0, 30, 3);
		imshow("Teste_Canny", frame_canny);


		// http://docs.opencv.org/2.4/doc/tutorials/introduction/load_save_image/load_save_image.html

		imwrite("C:/Users/User/Documents/Visual Studio 2015/Projects/experiencia_opencv/data/Canny_Image.jpg", frame_canny);
	//	img = cvQueryFrame(frame_canny);
	//	cvSaveImage("frame/canny.jpg", img);

		if (waitKey(0) == 27) break;

	}

}