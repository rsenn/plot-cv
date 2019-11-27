#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/videoio.hpp>

//#include <opencv2/ovis.hpp>
#include <opencv2/aruco.hpp>

#include <iostream>
#include <sstream>

#define KEY_ESCAPE 27

using namespace cv;

int
main() {
  Mat img;
  std::vector<std::vector<Point2f>> corners;
  std::vector<int> ids;
  std::vector<Vec3d> rvecs;
  std::vector<Vec3d> tvecs;

  const Size2i imsize(800, 600);
  const double focal_length = 800.0;

  // aruco
  Ptr<aruco::Dictionary> adict = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
  Mat marker_img(cv::Size(100,100), CV_8U);
  Mat out_img(cv::Size(780, 780), CV_8U);

  out_img ^= 0xff;

  for(size_t i = 0; i < 49; i++) {
    std::ostringstream os;
    aruco::drawMarker(adict, i, 100, marker_img);
    os << "marker-" << i << ".png";
    //imshow("marker", out_img);
    // imwrite(os.str(), out_img);
    int row = i / 7;
    int col = i % 7;
    std::cout << "type: " << marker_img.type() << std::endl;
    marker_img.copyTo(out_img(cv::Rect(row * 110 + 10, col * 110 + 10, 100, 100)));
  }
  imwrite("marker.png", out_img);

  // random calibration data, your mileage may vary
  Mat1d cm = Mat1d::zeros(3, 3);      // init empty matrix
  cm.at<double>(0, 0) = focal_length; // f_x
  cm.at<double>(1, 1) = focal_length; // f_y
  cm.at<double>(2, 2) = 1;            // f_z
  Mat K = getDefaultNewCameraMatrix(cm, imsize, true);

  // AR scene
  //  ovis::addResourceLocation("packs/Sinbad.zip"); // shipped with Ogre
  //
  //  Ptr<ovis::WindowScene> win = ovis::createWindow(String("arucoAR"), imsize, ovis::SCENE_INTERACTIVE | ovis::SCENE_AA);
  //  win->setCameraIntrinsics(K, imsize);
  //  win->createEntity("sinbad", "Sinbad.mesh", Vec3i(0, 0, 5), Vec3f(1.57, 0.0, 0.0));
  //  win->createLightEntity("sun", Vec3i(0, 0, 100));

  // video capture
  VideoCapture cap{0};
  cap.set(CAP_PROP_FRAME_WIDTH, imsize.width);
  cap.set(CAP_PROP_FRAME_HEIGHT, imsize.height);

  std::cout << "Press ESCAPE to exit demo" << std::endl;
  while(cv::waitKey(1) != KEY_ESCAPE) {
    cap.read(img);
    // win->setBackground(img);
    aruco::detectMarkers(img, adict, corners, ids);

    waitKey(1);

    if(ids.size() == 0)
      continue;

    aruco::estimatePoseSingleMarkers(corners, 5, K, noArray(), rvecs, tvecs);
    // win->setCameraPose(tvecs.at(0), rvecs.at(0), true);
  }

  return 0;
}
