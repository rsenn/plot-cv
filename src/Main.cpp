#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>
#include <opencv2\features2d\features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>

#include "Tensile.h"

using namespace cv;
using namespace std;


Mat run(Tensile *obj, int count) {

	vector<Point2f> mc;
	vector<vector<Point>> contours;
	Mat original = obj->getCurrentFrame();

	obj->getCurrentFrame() = obj->imageToBinary(obj->getCurrentFrame());

	contours = obj->getContours(obj->getCurrentFrame());	// get the contours 

	mc = obj->getMassCenters(contours);		// get the mass centers

	cout << endl << "Frame: " << count << endl;
	obj->printMassCenters(mc, contours);		// print mass centers


	//waitKey(0);

	for (int i = 0; i < contours.size(); i++)
		circle(original, mc[i], 1, CV_RGB(255, 0, 0), 3, 8, 0);


	return original;
}


int main()
{
	

	VideoCapture capture("C:/Users/Jameson/Desktop/data/specvid.avi");
	Tensile *object = new Tensile(capture.get(CV_CAP_PROP_FRAME_COUNT));

	Mat currentFrame;




	namedWindow("Contour", WINDOW_AUTOSIZE);

	for (int i = 0; i < object->getFrameNumber() -1; i++)
	{
		capture >> currentFrame;
		object->setCurrentFrame(currentFrame);

		imshow("Contour", run(object, i));
		waitKey(10);


	}


	
	cout << endl << endl << "total amount of frames: " << object->getFrameNumber() << endl << endl;


	return 0;
}
