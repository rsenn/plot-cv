/*
Author:			Stewart Nash
File:			img_proc_7.cpp
Description:	Morphological isolation of red objects in video with contour tracking.
Arguments:		img_proc_7.exe [input filename] [output filename]
*/
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;

/*
Class pointstack is a stack of 2D float points. It has a LIFO structure.

If one attempts to push onto a full stack, the point will be pushed on
the top, and the first point will fall off of the bottom. Every element
will move down one.

The pop(Point2f & point) returns the integer place value of where the 
point was popped from and places the point in the parameter passed by
reference. You could also say it returns one more than where the stack
pointer (current_point) points to after the pop(). This is returned
rather than the current stack location so that an empty stack that a
pop() was attempted on will return a zero. The push returns the position
where the point was pushed to to be consistent. This is one less than
the current position of the stack.
*/
class Pointstack
{
	static const int MAX_STACK_RANGE = 250;
	int current_point;
	int max_point;
	cv::Point2f point_array[MAX_STACK_RANGE];

public:
	Pointstack();
	int push(cv::Point2f point);
	int pop(cv::Point2f & point);
	cv::Point2f pop(); 
};

Pointstack::Pointstack()
{
	current_point = 0;
	max_point = MAX_STACK_RANGE - 1;
}

int Pointstack::push(cv::Point2f point)
{
	int i;

	if (current_point < max_point)
		point_array[++current_point] = point;
	else
	{
		for (i = 1; i < max_point; i++)
			point_array[i-1] = point_array[i];
		point_array[i-1] = point_array[i];
		point_array[i] = point;
		return current_point;
	}

	return current_point - 1;
}

cv::Point2f Pointstack::pop()
{
	if (current_point > 0)
		return point_array[current_point--];
	else
	{
		//std::cout << "Error: attempted Pointstack::pop() with empty stack. Returning error_point (-1, -1).\n";
		cv::Point2f error_point(-1, -1);
		return error_point;
	}
}

int Pointstack::pop(cv::Point2f & point)
{
	if (current_point > 0)
		point = point_array[current_point--];
	else
	{
		//std::cout << "Error: attempted Pointstack::pop(cv::Point2f point) with empty stack. Returning 0. Point set to error_point, (-1, -1).\n";
		cv::Point2f error_point(-1, -1);
		point = error_point;
		return 0;
	}	
	return current_point + 1;
}

Pointstack myGlobalPoints;

//Create process_image with a class parameter which encapsulates hue, saturation and value.
int process_image(Mat & inputImage, Mat & outputImage);
//Track the detected objects with a bounding box and indication of center of mass.
int add_tracking(Mat & inputImage, Mat & outputImage);

const int RANGES = 2;

/*
Variables
---------
myVideo - input video file.
myOutput - created video file to write to.
myImage - captured frame for processing.
myCOpy - copy of captured frame for processing.
myMask - single channel storage for morphologic processing.
*/
int main(int argc, char * argv[])
{
	VideoCapture myVideo;
	VideoWriter myOutput;
	Mat myImage;
	Mat myCopy;
	Mat myMask;
	int elem_width;
	int elem_height;
	int i, j;

	if (argc > 1)
		myVideo.open(argv[1]);
	else
	{
		std::cout << "No filename provided.";
		waitKey(0);
		return 0;
	}
	//myVideo.open("F:/Documents/Visual Studio 2010/Projects/img_proc_7/Debug/red_balloon.avi");
	if (!myVideo.isOpened())
	{
		std::cout << "Unable to open file.";
		waitKey(0);
		return 0;
	}
	elem_width = (int) myVideo.get(CV_CAP_PROP_FRAME_WIDTH);
	elem_height = (int) myVideo.get(CV_CAP_PROP_FRAME_HEIGHT);
	Size mySize = Size(2 * elem_width, elem_height);

	if (argc > 2)
		myOutput.open(argv[2], CV_FOURCC('M','J','P','G'), myVideo.get(CV_CAP_PROP_FPS), mySize);
	else
		myOutput.open("output.avi", CV_FOURCC('M','J','P','G'), myVideo.get(CV_CAP_PROP_FPS), mySize);
	//myOutput.open("F:/Documents/Visual Studio 2010/Projects/img_proc_7/Debug/img_proc_7_out.avi", CV_FOURCC('M','J','P','G'), myVideo.get(CV_CAP_PROP_FPS), mySize);
	if (!myOutput.isOpened())
	{
		std::cout << "Unable to create file.";
		waitKey(0);
		return 0;
	}
	//Left and right halves of stitched video frame given a ROI.
	Mat myStitch(elem_height, 2 * elem_width, CV_8UC3);
	Mat myLeft(myStitch, Rect(0, 0, elem_width, elem_height));
	Mat myRight(myStitch, Rect(elem_width, 0, elem_width, elem_height));

	j = (int) myVideo.get(CV_CAP_PROP_FRAME_COUNT);
	//j = 30000;
	for (i = 0; i < j; i++)
	{
		std::cout << "frame " << i << " of " << j << std::endl;
		myVideo >> myImage;
		process_image(myImage, myMask);
		cvtColor(myMask, myCopy, CV_GRAY2BGR);
		add_tracking(myMask, myCopy);
		myImage.copyTo(myLeft);
		myCopy.copyTo(myRight);
		myOutput << myStitch;
	}

	myVideo.release();

	return 0;
}

int process_image(Mat & inputImage, Mat & outputImage)
{
	Mat myCopy;
	Mat myMask[RANGES];
	Mat myKernel;

	int lowHue[RANGES], highHue[RANGES], lowSat[RANGES], highSat[RANGES], lowVal[RANGES], highVal[RANGES];
	int elem_size_x;
	int elem_size_y;

	int i;

	elem_size_x = 7;
	elem_size_y = 7;

	lowHue[0] = 165;
	highHue[0] = 179;
	lowSat[0] = 230;
	highSat[0] = 255;
	lowVal[0] = 64;
	highVal[0] = 255;

	lowHue[1] = 0;
	highHue[1] = 30;
	lowSat[1] = 230;
	highSat[1] = 255;
	lowVal[1] = 64;
	highVal[1] = 255;

	myKernel = getStructuringElement(MORPH_RECT, Size(elem_size_x, elem_size_y));
	cvtColor(inputImage, myCopy, COLOR_BGR2HSV);

	for (i = 0; i < RANGES; i++)
		inRange(myCopy, Scalar(lowHue[i], lowSat[i], lowVal[i]), Scalar(highHue[i], highSat[i], highVal[i]), myMask[i]);

	myMask[0].copyTo(outputImage);
	outputImage = Scalar(0);

	//Combine masks with logical or.
	for (i = 0; i < RANGES; i++)
		bitwise_or(outputImage, myMask[i], outputImage);
	//Perform morphological processing.
	erode(outputImage, outputImage, myKernel);
	dilate(outputImage, outputImage, myKernel);

	dilate(outputImage, outputImage, myKernel);
	erode(outputImage, outputImage, myKernel);

	return 0;
}

int add_tracking(Mat & inputImage, Mat & outputImage)
{
	vector <vector <Point>> vecContours;
	vector <Vec4i> vecHeirarchy;
	int contour_size;
	int i;
	int circ_radius;
	float minimum_area;
	Scalar myColor1(0, 255, 0);
	Scalar myColor2(0, 0, 255);
	Point2f temp_point(0, 0);
	Pointstack myCopy;

	minimum_area = 20 * 20;
	circ_radius = 2;

	findContours(inputImage, vecContours, vecHeirarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);

	contour_size = vecContours.size();
	vector <int> vecContId(contour_size);
	vector <float> vecContArea(contour_size);
	vector <Moments> vecMoment(contour_size);
	vector <Point2f> vecMassCenter(contour_size);

	for (i = 0; i < contour_size; i++)
	{
		//Order objects by appearance and find areas.
		vecContId[i] = i;
		vecContArea[i] = contourArea(vecContours[i]);

		//Find center of mass.
		vecMoment[i] = moments(vecContours[i]);
		vecMassCenter[i] = Point2f(vecMoment[i].m10 / vecMoment[i].m00, vecMoment[i].m01 / vecMoment[i].m00);

		//Draw bounding rectangle and push mass centers of larger objects on stack.
		if (vecContArea[i] >= minimum_area)
		{
			rectangle(outputImage, boundingRect(vecContours[i]), myColor1);
			myGlobalPoints.push(vecMassCenter[i]);
		}
	}
	//Display center of mass of all points on stack.
	while (i = myGlobalPoints.pop(temp_point))
	{
		circle(outputImage, temp_point, circ_radius, myColor2);
		myCopy.push(temp_point);
	}
	while (i = myCopy.pop(temp_point))
		myGlobalPoints.push(temp_point);

	return 0;
}