#include <sstream>
#include "objecttracking.h"

using namespace std;
using namespace cv;
TCHAR* myChar = "COM7";
tstring commPortName(myChar);
Serial serial(commPortName, 9600);

// initial min and max HSV filter values.
// these will be changed using trackbars
int H_MIN = 0;
int H_MAX = 256;
int S_MIN = 0;
int S_MAX = 256;
int V_MIN = 0;
int V_MAX = 256;
// default capture width and height
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;
// max number of objects to be detected in frame
const int MAX_NUM_OBJECTS = 50;
// minimum and maximum object area
const int MIN_OBJECT_AREA = 20 * 20;
const int MAX_OBJECT_AREA = FRAME_HEIGHT * FRAME_WIDTH / 1.5;
// names that will appear at the top of each window
const string windowName = "Original Image";
const string windowName1 = "HSV Image";
const string windowName2 = "Thresholded Image";
const string windowName3 = "After Morphological Operations";
const string trackbarWindowName = "Trackbars";
bool case1(Entity, Entity);
bool case2(Entity, Entity);
bool case3(Entity, Entity);
bool case4(Entity, Entity);

void
myFunc(cv::Mat& cameraFeed, Entity& playerfront, Entity& playerrear, Entity& ball, Entity& goal);
void movePlayer(string balld,
                Entity& playerfront,
                Entity& playerrear,
                Entity& ball,
                Entity& goal,
                Serial& serial);

void on_trackbar(int, void*) { // This function gets called whenever a
  // trackbar position is changed
}
string
intToString(int number) {

  std::stringstream ss;
  ss << number;
  return ss.str();
}
void
createTrackbars() {
  // create window for trackbars

  namedWindow(trackbarWindowName, 0);
  // create memory to store trackbar name on window
  char TrackbarName[50];
  sprintf_s(TrackbarName, "H_MIN", H_MIN);
  sprintf_s(TrackbarName, "H_MAX", H_MAX);
  sprintf_s(TrackbarName, "S_MIN", S_MIN);
  sprintf_s(TrackbarName, "S_MAX", S_MAX);
  sprintf_s(TrackbarName, "V_MIN", V_MIN);
  sprintf_s(TrackbarName, "V_MAX", V_MAX);
  // create trackbars and insert them into window
  // 3 parameters are: the address of the variable that is changing when the trackbar is
  // moved(eg.H_LOW), the max value the trackbar can move (eg. H_HIGH), and the function that is
  // called whenever the trackbar is moved(eg. on_trackbar)
  //                                  ---->    ---->     ---->
  createTrackbar("H_MIN", trackbarWindowName, &H_MIN, H_MAX, on_trackbar);
  createTrackbar("H_MAX", trackbarWindowName, &H_MAX, H_MAX, on_trackbar);
  createTrackbar("S_MIN", trackbarWindowName, &S_MIN, S_MAX, on_trackbar);
  createTrackbar("S_MAX", trackbarWindowName, &S_MAX, S_MAX, on_trackbar);
  createTrackbar("V_MIN", trackbarWindowName, &V_MIN, V_MAX, on_trackbar);
  createTrackbar("V_MAX", trackbarWindowName, &V_MAX, V_MAX, on_trackbar);
}
// void drawObject(std::vector<Entity> theEntities, cv::Mat &frame) {
//
//	for (int i = 0; i < theEntities.size(); i++) {
//		cv::circle(frame, cv::Point(theEntities.at(i).getXPos(), theEntities.at(i).getYPos()), 10,
//cv::Scalar(0, 0,
// 255)); 		cv::putText(frame, intToString(theEntities.at(i).getXPos()) + " , " +
// intToString(theEntities.at(i).getXPos()), cv::Point(theEntities.at(i).getXPos(),
// theEntities.at(i).getYPos() + 20), 1,  1, Scalar(0, 255, 0)); 		cv::putText(frame,
// theEntities.at(i).getType(), cv::Point(theEntities.at(i).getXPos(),  theEntities.at(i).getYPos()
// - 30), 1, 2, theEntities.at(i).getColor());
//	}
//}
void
drawObject(Entity anEntity, cv::Mat& frame) {
  cv::circle(frame, cv::Point(anEntity.getXPos(), anEntity.getYPos()), 10, cv::Scalar(0, 0, 255));
  cv::putText(frame,
              intToString(anEntity.getXPos()) + " , " + intToString(anEntity.getYPos()),
              cv::Point(anEntity.getXPos(), anEntity.getYPos() + 20),
              1,
              1,
              Scalar(0, 255, 0));
  cv::putText(frame,
              anEntity.getType(),
              cv::Point(anEntity.getXPos(), anEntity.getYPos() - 30),
              1,
              2,
              anEntity.getColor());
}
void
morphOps(cv::Mat& thresh) {

  // create structuring element that will be used to "dilate" and "erode" image.
  // the element chosen here is a 3px by 3px rectangle

  cv::Mat erodeElement = getStructuringElement(MORPH_RECT, Size(3, 3));
  // dilate with larger element so make sure object is nicely visible
  cv::Mat dilateElement = getStructuringElement(MORPH_RECT, Size(8, 8));

  erode(thresh, thresh, erodeElement);
  erode(thresh, thresh, erodeElement);

  dilate(thresh, thresh, dilateElement);
  dilate(thresh, thresh, dilateElement);
}
// void trackFilteredObject(cv::Mat threshold, cv::Mat HSV, cv::Mat &cameraFeed) {
//	std::vector <Entity> entities;
//
//	cv::Mat temp;
//	threshold.copyTo(temp);
//	//these two std::vectors needed for output of findContours
//	std::vector<std::vector<cv::Point>> contours;
//	std::vector<Vec4i> hierarchy;
//	//find contours of filtered image using openCV findContours function
//	findContours(temp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
//	//use moments method to find our filtered object
//	double refArea = 0;
//	bool objectFound = false;
//	if (hierarchy.size() > 0) {
//		int numObjects = hierarchy.size();
//		//if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
//		if (numObjects<MAX_NUM_OBJECTS) {
//			for (int index = 0; index >= 0; index = hierarchy[index][0]) {
//
//				Moments moment = moments((cv::Mat)contours[index]);
//				double area = moment.m00;
//
//				//if the area is less than 20 px by 20px then it is probably just noise
//				//if the area is the same as the 3/2 of the image size, probably just a bad filter
//				//we only want the object with the largest area so we safe a reference area each
//				//iteration and compare it to the area in the next iteration.
//				if (area>MIN_OBJECT_AREA) {
//
//					Entity entity1;
//
//					entity1.setXPos(moment.m10 / area);
//					entity1.setYPos(moment.m01 / area);
//
//					entities.push_back(entity1);//whenever an object is detected it is pushed to the
//std::vector
//
//					objectFound = true;
//				}
//				else objectFound = false;
//
//
//			}
//			//let user know you found an object
//			if (objectFound == true) {
//				//draw object location on screen
//				drawObject(anEntity, cameraFeed);
//			}
//
//		}
//		else putText(cameraFeed, "TOO MUCH NOISE! ADJUST FILTER", cv::Point(0, 50), 1, 2, Scalar(0, 0,
//255), 2);
//	}
//}

void
trackFilteredObject(Entity& anEntity, cv::Mat threshold, cv::Mat HSV, cv::Mat& cameraFeed) {
  /*std::vector <Entity> entities;*/ // std::vector of entities that are passed

  cv::Mat temp;
  threshold.copyTo(temp);
  // these two std::vectors needed for output of findContours
  std::vector<std::vector<cv::Point>> contours;
  std::vector<Vec4i> hierarchy;
  // find contours of filtered image using openCV findContours function
  findContours(temp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
  // use moments method to find our filtered object
  double refArea = 0;
  bool objectFound = false;
  if(hierarchy.size() > 0) {
    int numObjects = hierarchy.size();
    // if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
    if(numObjects < MAX_NUM_OBJECTS) {
      for(int index = 0; index >= 0; index = hierarchy[index][0]) {

        Moments moment = moments((cv::Mat)contours[index]);
        double area = moment.m00;

        // if the area is less than 20 px by 20px then it is probably just noise
        // if the area is the same as the 3/2 of the image size, probably just a bad filter
        // we only want the object with the largest area so we safe a reference area each
        // iteration and compare it to the area in the next iteration.
        if(area > MIN_OBJECT_AREA) {

          anEntity.setXPos(moment.m10 / area);
          anEntity.setYPos(moment.m01 / area);
          // anEntity.setType(anEntity.getType());//get type from the passed entity
          // anEntity.setColor(anEntity.getColor());//get color from the passed entity

          /*entities.push_back(anEntity);*/ // whenever an object is detected it is pushed to the
                                            // std::vector

          objectFound = true;
        } else
          objectFound = false;
      }
      // let user know you found an object
      if(objectFound == true) {
        // draw object location on screen
        drawObject(anEntity, cameraFeed);
      }

    } else
      putText(cameraFeed,
              "TOO MUCH NOISE! ADJUST FILTER",
              cv::Point(0, 50),
              1,
              2,
              Scalar(0, 0, 255),
              2);
  }
}

int
main(int argc, char* argv[]) {

  // if we would like to calibrate our filter values, set to true.
  bool calibrationMode = false;

  // cv::Matrix to store each frame of the webcam feed
  cv::Mat cameraFeed;
  cv::Mat threshold;
  cv::Mat HSV;

  if(calibrationMode) {
    // create slider bars for HSV filtering
    createTrackbars();
  }
  VideoCapture capture;
  // video capture object to acquire webcam feed
  capture.open(1);

  // open capture object at location zero (default location for webcam)

  // set height and width of capture frame
  capture.set(CV_CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
  capture.set(CV_CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
  // start an infinite loop where webcam feed is copied to cameraFeed matrix
  // all of our operations will be performed within this loop

  while(1) {
    // store image to matrix
    // convert frame from BGR to HSV colorspace
    /*cvtColor(cameraFeed, HSV, COLOR_BGR2HSV);*/
    capture.read(cameraFeed);
    if(calibrationMode == true) {
    }
    //	//if in calibration mode, we track objects based on the HSV slider values.
    //	cvtColor(cameraFeed, HSV, COLOR_BGR2HSV);
    //	inRange(HSV, Scalar(H_MIN, S_MIN, V_MIN), Scalar(H_MAX, S_MAX, V_MAX), threshold);
    //	morphOps(threshold);
    //	/*imshow(windowName2, threshold);*/
    //	trackFilteredObject(threshold, HSV, cameraFeed);
    //}
    else {
      Entity playerfront("playerfront"), playerrear("playerrear"), ball("ball"), goal("goal");
      while(waitKey(33) != 27) {
        myFunc(cameraFeed, playerfront, playerrear, ball, goal);
        capture.read(cameraFeed);
        imshow(windowName, cameraFeed);
      }
      cout << "Ball X: " << ball.getXPos() << " Ball Y: " << ball.getYPos()
           << " Player X: " << playerfront.getXPos() << " Player Y: " << playerfront.getYPos()
           << endl;
      string balld = findDirection(playerrear, ball);
      int x, y;
      int mySentinel = true;
      // if direction is northwest x and y both are decreased until they match
      if(balld == "NORTHWEST") {
        cout << balld << endl;
        while(playerrear.getXPos() != playerfront.getXPos() &&
              playerrear.getYPos() < playerfront.getYPos()) {
          if(case1(playerfront, playerrear) || case2(playerfront, playerrear)) {
            myData("L", serial);
            capture.read(cameraFeed);
            myFunc(cameraFeed, playerfront, playerrear, ball, goal);
            imshow(windowName, cameraFeed);
          } else {
            myData("R", serial);
            capture.read(cameraFeed);
            myFunc(cameraFeed, playerfront, playerrear, ball, goal);
            imshow(windowName, cameraFeed);
          }
          waitKey(33);
        }
        while(playerfront.getYPos() > ball.getYPos()) {
          myData("F", serial);
          capture.read(cameraFeed);
          myFunc(cameraFeed, playerfront, playerrear, ball, goal);
          imshow(windowName, cameraFeed);
          waitKey(33);
        }
        while(playerfront.getYPos() != playerrear.getYPos() &&
              playerfront.getXPos() > playerrear.getXPos()) {
          waitKey(33);
          if(case1(playerfront, playerrear) || case4(playerfront, playerrear)) {
            myData("R", serial);
            capture.read(cameraFeed);
            myFunc(cameraFeed, playerfront, playerrear, ball, goal);
            imshow(windowName, cameraFeed);
          } else {
            myData("L", serial);
            capture.read(cameraFeed);
            myFunc(cameraFeed, playerfront, playerrear, ball, goal);
            imshow(windowName, cameraFeed);
          }
        }
        while(playerfront.getXPos() > ball.getXPos()) {
          waitKey(33);
          myData("F", serial);
          capture.read(cameraFeed);
          myFunc(cameraFeed, playerfront, playerrear, ball, goal);
          imshow(windowName, cameraFeed);
        }
      }

      // if direction is southwest x is decreased and y is increased until they match
      else if(balld == "SOUTHWEST") {
        cout << balld << endl;
        // turn left until car rear x - car front x >= whatever distance is between them && car's
        // front y == car's rear y  go forward until ball's x coordinate == car's x coordinate  turn
        // left until car's front y - car rear y >= whatever distance between them && car's front x
        // == car's rear x  go forward until ball's y coordinate == car's y coordinate  stop
        {
          while(playerrear.getXPos() != playerfront.getXPos() &&
                playerrear.getYPos() > playerfront.getYPos()) {
            waitKey(33);
            if(case1(playerfront, playerrear) || case2(playerfront, playerrear)) {
              myData("R", serial);
              capture.read(cameraFeed);
              myFunc(cameraFeed, playerfront, playerrear, ball, goal);
              imshow(windowName, cameraFeed);
            } else {
              myData("L", serial);
              capture.read(cameraFeed);
              myFunc(cameraFeed, playerfront, playerrear, ball, goal);
              imshow(windowName, cameraFeed);
            }
          }
        }
        while(playerfront.getYPos() < ball.getYPos()) {
          waitKey(33);
          myData("F", serial);
          capture.read(cameraFeed);
          myFunc(cameraFeed, playerfront, playerrear, ball, goal);
          imshow(windowName, cameraFeed);
        }
        while(playerfront.getYPos() != playerrear.getYPos() &&
              playerfront.getXPos() > playerrear.getXPos()) {
          waitKey(33);
          if(case1(playerfront, playerrear) || case4(playerfront, playerrear)) {
            myData("R", serial);
            capture.read(cameraFeed);
            myFunc(cameraFeed, playerfront, playerrear, ball, goal);
            imshow(windowName, cameraFeed);
          } else {
            myData("L", serial);
            capture.read(cameraFeed);
            myFunc(cameraFeed, playerfront, playerrear, ball, goal);
            imshow(windowName, cameraFeed);
          }
        }
        while(playerfront.getXPos() > ball.getXPos()) {
          waitKey(33);
          myData("F", serial);
          capture.read(cameraFeed);
          myFunc(cameraFeed, playerfront, playerrear, ball, goal);
          imshow(windowName, cameraFeed);
        }
      }
      // if direction is southeast x and y both are increased until they match
      else if(balld == "SOUTHEAST") {
        cout << balld << endl;
        // turn right until car front x - car rear x >= whatever distance is between them && car's
        // front y == car's rear y  go forward until ball's x coordinate == car's x coordinate  turn
        // right until car's front y - car rear y >= whatever distance between them && car's front x
        // == car's rear x  go forward until ball's y coordinate == car's y coordinate  stop
        {
          while(playerrear.getXPos() != playerfront.getXPos() &&
                playerrear.getYPos() > playerfront.getYPos()) {
            waitKey(33);
            if(case1(playerfront, playerrear) || case2(playerfront, playerrear)) {
              myData("R", serial);
              capture.read(cameraFeed);
              myFunc(cameraFeed, playerfront, playerrear, ball, goal);
              imshow(windowName, cameraFeed);
            } else {
              myData("L", serial);
              capture.read(cameraFeed);
              myFunc(cameraFeed, playerfront, playerrear, ball, goal);
              imshow(windowName, cameraFeed);
            }
          }
          while(playerfront.getYPos() < ball.getYPos()) {
            waitKey(33);
            myData("F", serial);
            capture.read(cameraFeed);
            myFunc(cameraFeed, playerfront, playerrear, ball, goal);
            imshow(windowName, cameraFeed);
          }
          while(playerfront.getYPos() != playerrear.getYPos() &&
                playerfront.getXPos() < playerrear.getXPos()) {
            waitKey(33);
            if(case1(playerfront, playerrear) || case4(playerfront, playerrear)) {
              myData("L", serial);
              capture.read(cameraFeed);
              myFunc(cameraFeed, playerfront, playerrear, ball, goal);
              imshow(windowName, cameraFeed);
            } else {
              myData("R", serial);
              capture.read(cameraFeed);
              myFunc(cameraFeed, playerfront, playerrear, ball, goal);
              imshow(windowName, cameraFeed);
            }
          }
          while(playerfront.getXPos() < ball.getXPos()) {
            waitKey(33);
            myData("F", serial);
            capture.read(cameraFeed);
            myFunc(cameraFeed, playerfront, playerrear, ball, goal);
            imshow(windowName, cameraFeed);
          }
        }
      }
      // if direction is northeast x is increased and y is decreased until they match
      else if(balld == "NORTHEAST") {
        cout << balld << endl;
        // turn right until car front x - car rear x >= whatever distance is between them && car's
        // front y == car's rear y  go forward until ball's x coordinate == car's x coordinate  turn
        // left until car's rear y - car front y >= whatever distance between them && car's front x
        // == car's rear x  go forward until ball's y coordinate == car's y coordinate  stop
        {
          while(playerrear.getXPos() != playerfront.getXPos() &&
                playerrear.getYPos() < playerfront.getYPos()) {
            waitKey(33);
            if(case1(playerfront, playerrear) || case2(playerfront, playerrear)) {
              myData("L", serial);
              capture.read(cameraFeed);
              myFunc(cameraFeed, playerfront, playerrear, ball, goal);
              imshow(windowName, cameraFeed);
            } else {
              myData("R", serial);
              capture.read(cameraFeed);
              myFunc(cameraFeed, playerfront, playerrear, ball, goal);
              imshow(windowName, cameraFeed);
            }
          }
          while(playerfront.getYPos() > ball.getYPos()) {
            waitKey(33);
            myData("F", serial);
            capture.read(cameraFeed);
            myFunc(cameraFeed, playerfront, playerrear, ball, goal);
            imshow(windowName, cameraFeed);
          }
          while(playerfront.getYPos() != playerrear.getYPos() &&
                playerfront.getXPos() < playerrear.getXPos()) {
            waitKey(33);
            if(case1(playerfront, playerrear) || case2(playerfront, playerrear)) {
              myData("L", serial);
              capture.read(cameraFeed);
              myFunc(cameraFeed, playerfront, playerrear, ball, goal);
              imshow(windowName, cameraFeed);
            } else {
              myData("R", serial);
              capture.read(cameraFeed);
              myFunc(cameraFeed, playerfront, playerrear, ball, goal);
              imshow(windowName, cameraFeed);
            }
          }
          while(playerfront.getXPos() < ball.getXPos()) {
            waitKey(33);
            myData("F", serial);
            capture.read(cameraFeed);
            myFunc(cameraFeed, playerfront, playerrear, ball, goal);
            imshow(windowName, cameraFeed);
          }
        }
      }
      imshow(windowName, cameraFeed);
    }

    // show frames
    // imshow(windowName2,threshold);

    // imshow(windowName1,HSV);

    // delay 30ms so that screen can refresh.
    // image will not appear without this waitKey() command
    if(waitKey(30) == 27)
      break;
  }
  myData("E", serial);
  return 0;
}

void
myFunc(cv::Mat& cameraFeed, Entity& playerfront, Entity& playerrear, Entity& ball, Entity& goal) {
  cv::Mat HSV, threshold;
  // name the entity
  // playerfront
  cvtColor(cameraFeed, HSV, COLOR_BGR2HSV);
  inRange(HSV,
          playerfront.getHSVmin(),
          playerfront.getHSVmax(),
          threshold); // enter the range of hsv for the object
  morphOps(threshold);
  trackFilteredObject(playerfront,
                      threshold,
                      HSV,
                      cameraFeed); // passes the object to the tracking function //playerrear
  cvtColor(cameraFeed, HSV, COLOR_BGR2HSV);
  inRange(HSV,
          playerrear.getHSVmin(),
          playerrear.getHSVmax(),
          threshold); // enter the range of hsv for the object
  morphOps(threshold);
  trackFilteredObject(playerrear,
                      threshold,
                      HSV,
                      cameraFeed); // passes the object to the tracking function
  // ball
  cvtColor(cameraFeed, HSV, COLOR_BGR2HSV);
  inRange(HSV,
          goal.getHSVmin(),
          goal.getHSVmax(),
          threshold); // enter the range of hsv for the object
  morphOps(threshold);
  trackFilteredObject(goal,
                      threshold,
                      HSV,
                      cameraFeed); // passes the object to the tracking function
  // goal
  cvtColor(cameraFeed, HSV, COLOR_BGR2HSV);
  inRange(HSV,
          ball.getHSVmin(),
          ball.getHSVmax(),
          threshold); // enter the range of hsv for the object
  morphOps(threshold);
  trackFilteredObject(ball,
                      threshold,
                      HSV,
                      cameraFeed); // passes the object to the tracking function
}
bool
case1(Entity PF, Entity PR) {
  if(PF.getXPos() > PR.getXPos() && PF.getYPos() > PR.getYPos())
    return 1;
}
bool
case2(Entity PF, Entity PR) {
  if(PF.getXPos() > PR.getXPos() && PF.getYPos() < PR.getYPos())
    return 1;
}
bool
case3(Entity PF, Entity PR) {
  if(PF.getXPos() < PR.getXPos() && PF.getYPos() > PR.getYPos())
    return 1;
}
bool
case4(Entity PF, Entity PR) {
  if(PF.getXPos() < PR.getXPos() && PF.getYPos() < PR.getYPos())
    return 1;
}

void
movePlayer(string balld,
           Entity& playerfront,
           Entity& playerrear,
           Entity& ball,
           Entity& goal,
           Serial& serial) {}

// void movePlayer(string balld, Entity &playerfront, Entity &playerrear, Entity &ball,Entity &goal,
// Serial &serial) {
//	int x, y;
//	cv::Mat cameraFeed;
//	capture.open(1);
//
//	//if direction is northwest x and y both are decreased until they match
//	if (balld == "NORTHWEST")
//	{
//		cout << balld << " " << playerrear.getXPos() << " " <<playerrear.getYPos() << " "
//<<ball.getXPos()<<"
//"<<ball.getYPos()<<endl; 		while (!((playerrear.getXPos() - playerfront.getXPos() > 0) &&
//(playerfront.getYPos() -
// playerrear.getYPos()<10)))
//		{
//			myData("L", serial);
//			capture.read(cameraFeed);
//			myFunc(&cameraFeed, playerfront, playerrear, ball, goal);
//			imshow(windowName2, cameraFeed);
//		}
//		while (!(playerrear.getXPos() == ball.getXPos()))
//		{
//			myData("F", serial);
//			capture.read(cameraFeed);
//			myFunc(&cameraFeed,  playerfront, playerrear, ball, goal);
//			imshow(windowName2, cameraFeed);
//		}
//		while (!((playerrear.getYPos() - playerfront.getYPos() > 0) && (playerfront.getXPos() -
// playerrear.getXPos()<10)))
//		{
//			myData("R", serial);
//			capture.read(cameraFeed);
//			myFunc(&cameraFeed,  playerfront, playerrear, ball, goal);
//			imshow(windowName2, cameraFeed);
//		}
//		while (!(playerfront.getYPos() == ball.getYPos() - 5))
//		{
//			myData("F", serial);
//			capture.read(cameraFeed);
//			myFunc(&cameraFeed,  playerfront, playerrear, ball, goal);
//			imshow(windowName2, cameraFeed);
//
//		}
//	}
//	//if direction is southwest x is decreased and y is increased until they match
//	if (balld == "SOUTHWEST") {
//		cout << balld << " " << playerrear.getXPos() << " " << playerrear.getYPos() << " " <<
//ball.getXPos() << " " <<
// ball.getYPos() << endl;
//		//turn left until car rear x - car front x >= whatever distance is between them && car's front
//y == car's rear y
//		//go forward until ball's x coordinate == car's x coordinate
//		//turn left until car's front y - car rear y >= whatever distance between them && car's front
//x == car's rear x
//		//go forward until ball's y coordinate == car's y coordinate
//		//stop
//		if (!((playerrear.getXPos() - playerfront.getXPos() > 0) && (playerfront.getYPos() -
//playerrear.getYPos()<10))) { 			myData("L", serial);
//			//capture.read(cameraFeed);
//			//myFunc(&cameraFeed,  playerfront, playerrear, ball, goal);
//
//		}
//		while (!(playerrear.getXPos() == ball.getXPos())) {
//			myData("F", serial);
//			capture.read(cameraFeed);
//			myFunc(&cameraFeed,  playerfront, playerrear, ball, goal);
//			imshow(windowName2, cameraFeed);
//		}
//		while (!((playerfront.getYPos() - playerrear.getYPos() > 0) && (playerfront.getXPos() -
// playerrear.getXPos()<10))) { 			myData("L", serial); 			capture.read(cameraFeed);
// myFunc(&cameraFeed,  playerfront,  playerrear, ball, goal); 			imshow(windowName2, cameraFeed);
//		}
//		while (!(playerfront.getYPos() == ball.getYPos() - 5)) {
//			myData("F", serial);
//			capture.read(cameraFeed);
//			myFunc(&cameraFeed,  playerfront, playerrear, ball, goal);
//			imshow(windowName2, cameraFeed);
//		}
//	}
//	//if direction is southeast x and y both are increased until they match
//	if (balld == "SOUTHEAST") {
//		cout << balld << " " << playerrear.getXPos() << " " << playerrear.getYPos() << " " <<
//ball.getXPos() << " " <<
// ball.getYPos() << endl;
//		//turn right until car front x - car rear x >= whatever distance is between them && car's
//front y == car's rear
// y
//		//go forward until ball's x coordinate == car's x coordinate
//		//turn right until car's front y - car rear y >= whatever distance between them && car's front
//x == car's rear x
//		//go forward until ball's y coordinate == car's y coordinate
//		//stop
//		while (!((playerfront.getXPos() - playerrear.getXPos() > 0) && (playerfront.getYPos() ==
//playerrear.getYPos()))) { 			myData("R", serial); 			capture.read(cameraFeed);
//myFunc(&cameraFeed,  playerfront,
// playerrear, ball, goal); 			imshow(windowName2, cameraFeed);
//		}
//		while (!(playerrear.getXPos() == ball.getXPos())) {
//			myData("F", serial);
//			capture.read(cameraFeed);
//			myFunc(&cameraFeed,  playerfront, playerrear, ball, goal);
//			imshow(windowName2, cameraFeed);
//		}
//		while (!((playerfront.getYPos() - playerrear.getYPos() > 0) && (playerfront.getXPos() ==
//playerrear.getXPos()))) { 			myData("R", serial); 			capture.read(cameraFeed);
//myFunc(&cameraFeed,  playerfront,
// playerrear, ball, goal); 			imshow(windowName2, cameraFeed);
//		}
//		while (!(playerfront.getYPos() == ball.getYPos() - 5)) {
//			myData("F", serial);
//			capture.read(cameraFeed);
//			myFunc(&cameraFeed,  playerfront, playerrear, ball, goal);
//			imshow(windowName2, cameraFeed);
//		}
//	}
//	//if direction is northeast x is increased and y is decreased until they match
//	if (balld == "NORTHEAST") {
//		cout << balld << " " << playerrear.getXPos() << " " << playerrear.getYPos() << " " <<
//ball.getXPos() << " " <<
// ball.getYPos() << endl;
//		//turn right until car front x - car rear x >= whatever distance is between them && car's
//front y == car's rear
// y
//		//go forward until ball's x coordinate == car's x coordinate
//		//turn left until car's rear y - car front y >= whatever distance between them && car's front
//x == car's rear x
//		//go forward until ball's y coordinate == car's y coordinate
//		//stop
//		while (!((playerfront.getXPos() - playerrear.getXPos() > 0) && (playerfront.getYPos() ==
//playerrear.getYPos()))) { 			myData("R", serial); 			capture.read(cameraFeed);
//myFunc(&cameraFeed,  playerfront,
// playerrear, ball, goal); 			imshow(windowName2, cameraFeed);
//		}
//		while (!(playerrear.getXPos() == ball.getXPos())) {
//			myData("F", serial);
//			capture.read(cameraFeed);
//			myFunc(&cameraFeed,  playerfront, playerrear, ball, goal);
//			imshow(windowName2, cameraFeed);
//		}
//		while (!((playerrear.getYPos() - playerfront.getYPos() > 0) && (playerfront.getXPos() ==
//playerrear.getXPos()))) { 			myData("L", serial); 			capture.read(cameraFeed);
//myFunc(&cameraFeed,  playerfront,
// playerrear, ball, goal); 			imshow(windowName2, cameraFeed);
//		}
//		while (!(playerfront.getYPos() == ball.getYPos() - 5)) {
//			myData("F", serial);
//			capture.read(cameraFeed);
//			myFunc(&cameraFeed,  playerfront, playerrear, ball, goal);
//			imshow(windowName2, cameraFeed);
//		}
//	}
//}