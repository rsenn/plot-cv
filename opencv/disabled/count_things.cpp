/* count_things.cpp
 * a program to count stuff (people or objects) moving through a video
 * hopefully un live stream too
 *
 * Brian J Gravelle
 * ix.cs.uoregon.edu/~gravelle
 * gravelle@cs.uoregon.edu

 * Much of this code is based on helpful tutorials available here:
 * http://docs.opencv.org/3.1.0/d5/d07/tutorial_multitracker.html#gsc.tab=0
 * https://www.youtube.com/user/khounslow/featured

 * Apparently this is somthing I'm supposed to do:
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, cv::merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.

 * If by some miricale you find this software useful, thanks are accepted in
 * the form of chocolate or introductions to potential employers.

 */

/*TODO partial list
- flag to supress display
- getting objects from movement
- figure out tracking alg options
*/

#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>
#include <iostream>
#include <cstring>
#include <ctime>

using namespace std;
// using namespace cv;

int count_exit_1;
int count_exit_2;
int count_exit_3;
int count_exit_4;

//@wait for user to select objects to track
//@pre
//@post objects has one or more objecst in it or the program exits
void get_usr_defined_objects(cv::Mat& frame, cv::VideoCapture& cap, vector<Rect2d>& objects);

//@
void find_new_objects(cv::Mat& frame, cv::VideoCapture& cap, vector<Rect2d>& objects);

//@
void count_objects();

//@displays help message to stdout and exits
//@post program terminates
void show_help();

int
main(int argc, char** argv) {

  if(argc < 2)
    show_help();

  std::string trackingAlg = "KCF"; // default
  if(argc > 2)
    trackingAlg = argv[2]; // user

  cv::MultiTracker trackers(trackingAlg); // tracker
  vector<Rect2d> objects;                 // container for obj being tracked

  // get video
  // TODO make it video or camera
  std::string video = argv[1];
  cv::VideoCapture cap(video);

  cv::Mat frame;

  get_usr_defined_objects(frame, cap, objects);
  // initialize the tracker
  trackers.cv::add(frame, objects);

  count_exit_1 = 0;
  count_exit_2 = 0;
  count_exit_3 = 0;
  count_exit_4 = 0;

  // do the tracking
  printf("Start the tracking process, press ESC to quit.\n");
  for(;;) {
    // TODO get new objects during loop
    // get frame from the video
    cap >> frame;

    // stop the program if no more images
    if(frame.rows == 0 || frame.cols == 0)
      break;

    // update the tracking result
    trackers.update(frame);

    // TODO somthing more interesting
    //  - establish lcation - remove obj if off screen
    //  - count direction
    for(unsigned i = 0; i < trackers.objects.size(); i++) {
      cv::rectangle(frame, trackers.objects[i], cv::Scalar(255, 0, 0), 2, 1); // draw cv::rectangle around object
      int mid_x = trackers.objects[i].x + (trackers.objects[i].width / 2);
      int mid_y = trackers.objects[i].y - (trackers.objects[i].height / 2);

      // remove objects beyond bounds
      if(mid_x > frame.rows) {
        trackers.objects.erase(trackers.objects.begin() + i);
        count_exit_1++;
      } else if(mid_x < 0) {
        trackers.objects.erase(trackers.objects.begin() + i);
        count_exit_2++;
      } else if(mid_y > frame.cols) {
        trackers.objects.erase(trackers.objects.begin() + i);
        count_exit_3++;
      } else if(mid_y < 0) {
        trackers.objects.erase(trackers.objects.begin() + i);
        count_exit_4++;
      }
    }

    // show image with the tracked object
    cv::imshow("tracker", frame);

    // quit on ESC button
    if((cv::waitKey(1) == 27) || (trackers.objects.size() < 1))
      break;
  }
}

void
find_new_objects(cv::Mat& frame, cv::VideoCapture& cap, vector<Rect2d>& objects) {
}

void
get_usr_defined_objects(cv::Mat& frame, cv::VideoCapture& cap, vector<Rect2d>& objects) {
  cap >> frame;
  cv::selectROI("tracker", frame, objects);
  if(objects.size() < 1)
    exit(1);
}

void
show_help() {
  cout << endl
       << " Usage: ./count_things.out <video_name> [algorithm]\n"
          " examples:\n"
          " ./count_things.out /home/pi/videos/my_vid.h264\n"
          " ./count_things.out /home/pi/videos/my_vid.h264 MEDIANFLOW\n"
       << endl
       << endl;
  exit(1);
}