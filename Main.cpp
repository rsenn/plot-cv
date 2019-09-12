#include "opencv2/imgcodecs.hpp"
#include <opencv2\opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <iostream>
#include <vector>
#include "Function.h"
#include "Reference.h"

using namespace cv;
using namespace std;

using std::vector;
using std::to_string;

Mat ShowManyImages(Mat, Mat, Mat, Mat);
Mat ErrorDisplay(String);
double Reference();
double ChangeInDistance(int, int, double);
int CurrentDistance();
Mat PutTextOnMe(Mat, String, int, int, int);
String TwoDecimalString(double);
double GetDecrementReference(double);



int main()
{
	string filename = ".\\rodVideo.wmv";
	VideoCapture capWebcam(filename);
	//double Referrence = GetInput(Reference());
	double Referrence = 1;

	Scalar color = Scalar(255, 25, 255);

	String Tracker = "TrackBarWindow";
	int threshold_value = 180;
	int threshold_type = 3;
	int const max_value = 255;
	int const max_type = 4;
	int const max_binary_value = 255;

	namedWindow(Tracker);
	createTrackbar("Threshod Type", Tracker, &threshold_type, max_type);
	createTrackbar("Threshod Value", Tracker, &threshold_value, max_value);


	Mat SRC, LineMe;

	if (capWebcam.isOpened() == false)
	{
		cout << "WebCam Cannot be read" << endl;
		return(0);
	}

	while (capWebcam.isOpened())
	{
		capWebcam.read(SRC);

		Size OrgSize = SRC.size();

		LineMe = Mat::zeros(SRC.size(), CV_8UC3);

		Point StartBox = Point((SRC.cols / 2) - 100, (SRC.rows / 2) - 400);
		Point EndBox = Point((SRC.cols / 2) + 100, (SRC.rows / 2) + 400);
		Point MidBox = Point(SRC.cols / 2, SRC.rows / 2);

		circle(LineMe, MidBox, 2, Scalar(55, 255, 255));
		rectangle(LineMe, StartBox, EndBox, cv::Scalar(0, 255, 0), 5, 8, 0);

		Mat GaussiBlur;
		GaussianBlur(SRC, GaussiBlur, Size(5, 5), 1);

		Mat blurMe;
		blur(GaussiBlur, blurMe, Size(13, 13));

		Mat mask = blurMe.clone();

		erode(mask, mask, cv::Mat());
		erode(mask, mask, cv::Mat());

		dilate(mask, mask, cv::Mat());
		dilate(mask, mask, cv::Mat());

		Mat median;
		medianBlur(mask, median, 7);

		// Floodfill from point (0, 0)
		Mat im_floodfill = median.clone();
		floodFill(im_floodfill, cv::Point(0, 0), Scalar(255));

		// Invert floodfilled image
		Mat im_floodfill_inv;
		bitwise_not(im_floodfill, im_floodfill_inv);


		Mat thresholdImageInverse;
		threshold(im_floodfill_inv, thresholdImageInverse, threshold_value, max_binary_value, threshold_type);

		Mat imageCanny;
		Canny(thresholdImageInverse, imageCanny, 100, 200, 3);

		vector<vector<Point>> contours;
		vector<Vec4i> hierarchy;

		findContours(imageCanny, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, Point(0, 0));

		int countFor = 1;
		vector<Point>TopPoint;
		vector<Point>BottomPoint;

		for (int i = 0; i < contours.size(); i++)
		{
			for (int j = 0; j < contours[i].size(); j = j + countFor)
			{
				if (contours[i][j].x > StartBox.x && contours[i][j].x > StartBox.x && contours[i][j].x < EndBox.x && contours[i][j].x < EndBox.x) {
					cv::circle(LineMe, contours[i][j], 0.5, cv::Scalar(255, 0, 0));

					if (contours[i][j].y < MidBox.y) {
						TopPoint.push_back(contours[i][j]);
					}
					else if (contours[i][j].y > MidBox.y) {
						BottomPoint.push_back(contours[i][j]);
					}
				}
				else
				{
					cv::circle(LineMe, contours[i][j], 0.5, cv::Scalar(0, 0, 255));
				}
			}
		}

		int MatchingPointsSize = 0;
		int NumOfMatch = 0;

		for (int a = 1; a < TopPoint.size(); a++) {
			circle(SRC, TopPoint[a], 1, cv::Scalar(0, 0, 255));
			for (int b = 1; b < BottomPoint.size(); b++) {
				circle(SRC, BottomPoint[b], 1, cv::Scalar(0, 255, 0));
				if (TopPoint[a].x == BottomPoint[b].x) {
					MatchingPointsSize = MatchingPointsSize + (BottomPoint[b].y - TopPoint[a].y);
					NumOfMatch = NumOfMatch + 1;
				}
			}
		}

		double AvgResult = MatchingPointsSize / NumOfMatch;
		String Result = TwoDecimalString(AvgResult);
		PutTextOnMe(SRC, Result, 1, (StartBox.x + EndBox.x) / 2, StartBox.y + 50);
		PutTextOnMe(LineMe, Result, 1, (StartBox.x + EndBox.x) / 2, StartBox.y + 50);

		PutTextOnMe(SRC, "SRC", 1, 20, 20);
		PutTextOnMe(LineMe, "LineMe", 1, 20, 20);
		PutTextOnMe(im_floodfill, "Flood Fill", 1, 20, 20);
		PutTextOnMe(thresholdImageInverse, "Threshold Inverse", 1, 20, 20);
		PutTextOnMe(im_floodfill_inv, "Flood Fill Inverse", 1, 20, 20);

		namedWindow("Tester Display", CV_WINDOW_NORMAL);
		imshow("Tester Display", ShowManyImages(im_floodfill, thresholdImageInverse, im_floodfill_inv, thresholdImageInverse));

		namedWindow("LineMe", CV_WINDOW_NORMAL);
		imshow("LineMe", LineMe);

		namedWindow("SRC", CV_WINDOW_NORMAL);
		imshow("SRC", SRC);

		cvWaitKey(21);

	}

	return(0);

}