#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>
#include <opencv2\features2d\features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>

using namespace cv;
using namespace std;


Mat imageToBinary(Mat start) {

	Mat gray_image, thresh_image;
	cvtColor(start, gray_image, CV_BGR2GRAY);
	threshold(gray_image, thresh_image, 100, 255, THRESH_BINARY);

	medianBlur(thresh_image, thresh_image, 5);

	return thresh_image;
}

vector<vector<Point>> getContours(Mat start){
	
	Mat dst = Mat::zeros(start.rows, start.cols, CV_8UC3);
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	start = start > 1;

	findContours(start, contours, hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

	return contours;
}





vector<Point2f> getMassCenters(vector<vector<Point>> contours) {

	vector<Moments> mu(contours.size());
	vector<Point2f> mc(contours.size());

	for (int i = 0; i < contours.size(); i++)
	{
		mu[i] = moments(contours[i], false);
	}

	for (int i = 0; i < contours.size(); i++){
		mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
	}

	return mc;
}


void printMassCenters(vector<Point2f> mc, vector<vector<Point>> contours) {
	cout << "Mass Centers:" << endl;

	for (int i = 0; i < contours.size(); i++)
	{
		cout << mc[i] << endl;
	}




}

float getStrain(float gage_length, vector<Point2f> two_points) {

	float diff = two_points[0].y-two_points[1].y;

	float answ = (diff/gage_length)-1;

	return answ;
}


Mat run(Mat image, int count, int if_first, float& gage_length) {

	vector<Point2f> mc;
	vector<vector<Point>> contours;
	Mat original = image;

	image = imageToBinary(image);

	contours = getContours(image);	// get the contours 

	mc = getMassCenters(contours);		// get the mass centers
	
	cout << endl << "Frame: " << count << endl;
	printMassCenters(mc, contours);		// print mass centers

	if (if_first)
	{
		gage_length = mc[0].y - mc[1].y;
		cout << endl << endl << "GAGE LENGTH SET TO: " << gage_length << endl << endl;
	}

	float strain = getStrain(gage_length, mc);
	cout << "Strain:  " << strain << endl << endl;

	//waitKey(0);

	for (int i = 0; i < contours.size(); i++)
		circle(original, mc[i], 1, CV_RGB(255, 0, 0), 3, 8, 0);


	return original;
}


int main()
{



	

	bool if_first = true;

	VideoCapture capture("C:/Users/Jameson/Desktop/data/specvid.avi");
	int totalFrameNumber = capture.get(CV_CAP_PROP_FRAME_COUNT);
	Mat currentFrame;
	vector<Mat> frame;
	float gage_length = 0;

	capture >> currentFrame;
	run(currentFrame, 0, if_first, gage_length); // GETS THE FIRST ONE

	if_first = false;

	namedWindow("Contour", WINDOW_AUTOSIZE);

	for (int i = 0; i < totalFrameNumber-1; i++)
	{
		capture >> currentFrame;

		imshow("Contour", run(currentFrame, i, if_first, gage_length));
		waitKey(10);
	}


	
	cout << endl << endl << "total amount of frames: " << capture.get(CV_CAP_PROP_FRAME_COUNT) << endl << endl;


	return 0;
}
