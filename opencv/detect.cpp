#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cstdio>

//using namespace cv;

/** Function Headers */
void detectAndDisplay(cv::Mat frame);

/** Global variables */
constexpr auto face_cascade_name = "haarcascade_frontalface_alt.xml";
constexpr auto window_name = "Capture - Face detection";
cv::CascadeClassifier face_cascade;

/** @function main */
int
main(void) {
  cv::Mat frame;

  //-- 1. Load the cascades

  if(!face_cascade.load(face_cascade_name)) {
    printf("--(!)cv::Error loading cv::face cascade\n");
    return -1;
  };

  //-- 2. Read the video stream

  while(1) {

    // Workaround
    // OpenCV's cv::VideoCapture works for USB cameras but Raspberry Pi CSI Camera Interface
    // still finding better solution ...
    system("/opt/vc/bin/raspistill -w 400 -h 300 --quality 50 --timeout 10 --output /tmp/result.jpg");
    frame = cv::imread("/tmp/result.jpg");

    if(frame.empty()) {
      printf(" --(!) No captured frame -- Break!");
      break;
    }

    //-- 3. Apply the classifier to the frame

    detectAndDisplay(frame);

    int c = cv::waitKey(10);

    if((char)c == 27) {
      break;
    } // escape
  }

  return 0;
}

/** @function detectAndDisplay */
void
detectAndDisplay(cv::Mat frame) {
  std::vector<cv::Rect> faces;
  cv::Mat frame_gray;

  cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);
  cv::equalizeHist(frame_gray, frame_gray);

  //-- Detect faces
  face_cascade.detectMultiScale(frame_gray, faces, 1.1, 2, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30));

  for(size_t i = 0; i < faces.size(); i++) {
    const cv::Point center(faces[i].x + faces[i].width / 2, faces[i].y + faces[i].height / 2);

    cv::ellipse(frame, // image
            center,
            cv::Size(faces[i].width / 2, faces[i].height / 2), // axes
            0,                                             // angle
            0,
            360,                 // startAngle, endAngle
            cv::Scalar(255, 0, 255), // color
            4,
            8,
            0); // thickness, lineType, shift

    cv::Mat faceROI = frame_gray(faces[i]);
  }
}
