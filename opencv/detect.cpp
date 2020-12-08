#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cstdio>

using namespace cv;

/** Function Headers */
void detectAndDisplay(Mat frame);

/** Global variables */
constexpr auto face_cascade_name = "haarcascade_frontalface_alt.xml";
constexpr auto window_name = "Capture - Face detection";
CascadeClassifier face_cascade;

/** @function main */
int
main(void) {
  Mat frame;

  //-- 1. Load the cascades

  if(!face_cascade.load(face_cascade_name)) {
    printf("--(!)Error loading face cascade\n");
    return -1;
  };

  //-- 2. Read the video stream

  while(1) {

    // Workaround
    // OpenCV's VideoCapture works for USB cameras but Raspberry Pi CSI Camera Interface
    // still finding better solution ...
    system("/opt/vc/bin/raspistill -w 400 -h 300 --quality 50 --timeout 10 --output /tmp/result.jpg");
    frame = imread("/tmp/result.jpg");

    if(frame.empty()) {
      printf(" --(!) No captured frame -- Break!");
      break;
    }

    //-- 3. Apply the classifier to the frame

    detectAndDisplay(frame);

    int c = waitKey(10);

    if((char)c == 27) {
      break;
    } // escape
  }

  return 0;
}

/** @function detectAndDisplay */
void
detectAndDisplay(Mat frame) {
  std::vector<Rect> faces;
  Mat frame_gray;

  cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
  equalizeHist(frame_gray, frame_gray);

  //-- Detect faces
  face_cascade.detectMultiScale(frame_gray, faces, 1.1, 2, 0 | CASCADE_SCALE_IMAGE, Size(30, 30));

  for(size_t i = 0; i < faces.size(); i++) {
    const Point center(faces[i].x + faces[i].width / 2, faces[i].y + faces[i].height / 2);

    ellipse(frame, // image
            center,
            Size(faces[i].width / 2, faces[i].height / 2), // axes
            0,                                             // angle
            0,
            360,                 // startAngle, endAngle
            Scalar(255, 0, 255), // color
            4,
            8,
            0); // thickness, lineType, shift

    Mat faceROI = frame_gray(faces[i]);
  }
}
