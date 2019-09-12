#include <iostream>
#include <string>
#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace std;
using namespace cv;

int Threshold = 0;
double ThresholdD = 0.0;
int LowThreshold = 80; // 130 for web cam
int HighThreshold = 127; // 165 for web cam

void on_Detect_thresh_change(int x,void *p)
{
    Threshold = x;
    ThresholdD = (double) x/100;
}
void on_Canny_low_change(int x,void *p)
{
    LowThreshold = x;
}
void on_Canny_high_change(int x,void *p)
{
    HighThreshold = x;
}

int main(int argc, char** argv)
{
    //"C:\Users\User\Documents\GitHub\EdgeDetector\EdgeD\test_sphere_1.mp4"

    VideoCapture cap(argv[1]); // open the video file for reading
    VideoCapture webcam(1); // open web cam

    namedWindow("Window",CV_WINDOW_AUTOSIZE);
    createTrackbar( "Detect_thresh" ,"Window" ,&Threshold        ,100 ,on_Detect_thresh_change   );
    createTrackbar( "Canny_low"     ,"Window" ,&LowThreshold     ,255 ,on_Canny_low_change       );
    createTrackbar( "Canny_high"    ,"Window" ,&HighThreshold    ,255 ,on_Canny_high_change      );

    Mat frame;
    Mat gray;
    bool bSuccess = true;
    bSuccess = cap.read(frame);
    if (!bSuccess) { // setting the default for the web cam
        LowThreshold = 140;
        HighThreshold = 165;
    }
    double noFrames = cap.get(7);
    double currentFrame = cap.get(1);

    while(1) {
        if (bSuccess) { // read a new frame from video
            cap.read(frame);
            currentFrame = cap.get(1);
            if (currentFrame>=(noFrames-2.0)){
                cap.set(2,0); // replaying the video
                cout << "replay the video" << endl;
            }
        }
        else { // read a new frame from webcam
            webcam.read(frame);
        }

                // === First the circle detection ===

        cvtColor(frame, gray, CV_BGR2GRAY);
        // smooth it, otherwise a lot of false circles may be detected
        blur( gray, gray, Size(3,3) );
        vector<Vec3f> circles;
        HoughCircles(gray, circles, CV_HOUGH_GRADIENT,
                     2, 100, HighThreshold, LowThreshold, 10, 3000 );

                // === Then the canny edge ditection ===

        // Reduce noise with a kernel 3x3
        blur( frame, frame, Size(3,3) );
        // Canny detector
        Canny( frame, frame, LowThreshold, HighThreshold, 3 );

                // === Plotting the circles on the canny ===

        for( int i = 0; i < circles.size(); i++ ) {
            Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);
            // draw the circle outline
            circle( frame, center, radius, Scalar(255,255,255), 3, 8, 0 );
        }

        // dysplay the frame
        imshow("Window", frame );

                // === Key commands ===

        int k = waitKey(30);
        if(k == 32) { //wait for 'space' key press for 30 ms. Then pauses
            while(1) {
                k = waitKey();
                if (k == 32) {
                    break;
                }
                else if(k != -1) { // exits on any other key
                    return -1;
                }
            }
        }
        else if(k != -1) { // exits on any other key
            return -1;
        }
    }
}
