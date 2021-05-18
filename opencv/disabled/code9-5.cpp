// Program illustrate stereo camera rectification
// Author: Samarth Manoj Brahmbhatt, University of Pennsylvania

#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <boost/filesystem.hpp>
#include "Config.h"

//using namespace cv;
using namespace std;
using namespace boost::filesystem3;

class calibrator {
private:
  string l_path, r_path;
  vector<cv::Mat> l_images, r_images;
  cv::Mat l_cameraMatrix, l_distCoeffs, r_cameraMatrix, r_distCoeffs;
  bool show_chess_corners;
  float side_length;
  int width, height;
  vector<vector<cv::Point2f>> l_image_points, r_image_points;
  vector<vector<Point3f>> object_points;
  cv::Mat R, T, E, F;

public:
  calibrator(string, string, float, int, int);
  void calc_image_points(bool);
  bool calibrate();
  void save_info(string);
  cv::Size get_image_size();
};

class rectifier {
private:
  cv::Mat map_l1, map_l2, map_r1, map_r2; // pixel maps for rectification
  string path;

public:
  rectifier(string, cv::Size);   // constructor
  void show_rectified(cv::Size); // function to show live rectified feed from stereo camera
};

calibrator::calibrator(string _l_path, string _r_path, float _side_length, int _width, int _height) {
  side_length = _side_length;
  width = _width;
  height = _height;

  l_path = _l_path;
  r_path = _r_path;

  // Read images
  for(directory_iterator i(l_path), end_iter; i != end_iter; i++) {
    string im_name = i->path().filename().string();
    string l_filename = l_path + im_name;
    im_name.replace(im_name.begin(), im_name.begin() + 4, string("right"));
    string r_filename = r_path + im_name;
    cv::Mat lim = cv::imread(l_filename), rim = cv::imread(r_filename);
    if(!lim.empty() && !rim.empty()) {
      l_images.push_back(lim);
      r_images.push_back(rim);
    }
  }
}

void
calibrator::calc_image_points(bool show) {
  // Calculate the object points in the object co-ordinate system (origin at top left corner)
  vector<Point3f> ob_p;
  for(int i = 0; i < height; i++) {
    for(int j = 0; j < width; j++) { ob_p.push_back(Point3f(j * side_length, i * side_length, 0.f)); }
  }

  if(show) {
    cv::namedWindow("Left Chessboard corners");
    cv::namedWindow("Right Chessboard corners");
  }

  for(int i = 0; i < l_images.size(); i++) {
    cv::Mat lim = l_images[i], rim = r_images[i];
    vector<cv::Point2f> l_im_p, r_im_p;
    bool l_pattern_found = cv::findChessboardCorners(lim,
                                                 cv::Size(width, height),
                                                 l_im_p,
                                                 CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
    bool r_pattern_found = cv::findChessboardCorners(rim,
                                                 cv::Size(width, height),
                                                 r_im_p,
                                                 CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
    if(l_pattern_found && r_pattern_found) {
      object_points.push_back(ob_p);
      cv::Mat gray;
      cv::cvtColor(lim, gray, cv::COLOR_BGR2GRAY);
      cv::cornerSubPix(gray, l_im_p, cv::Size(5, 5), cv::Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
      cv::cvtColor(rim, gray, cv::COLOR_BGR2GRAY);
      cv::cornerSubPix(gray, r_im_p, cv::Size(5, 5), cv::Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
      l_image_points.push_back(l_im_p);
      r_image_points.push_back(r_im_p);
      if(show) {
        cv::Mat im_show = lim.clone();
        cv::drawChessboardCorners(im_show, cv::Size(width, height), l_im_p, true);
        cv::imshow("Left Chessboard corners", im_show);
        im_show = rim.clone();
        cv::drawChessboardCorners(im_show, cv::Size(width, height), r_im_p, true);
        cv::imshow("Right Chessboard corners", im_show);
        while(char(cv::waitKey(1)) != ' ') {}
      }
    } else {
      l_images.erase(l_images.begin() + i);
      r_images.erase(r_images.begin() + i);
    }
  }
}

bool
calibrator::calibrate() {
  string filename = DATA_FOLDER + string("left_cam_calib.xml");
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  fs["cameraMatrix"] >> l_cameraMatrix;
  fs["distCoeffs"] >> l_distCoeffs;
  fs.release();

  filename = DATA_FOLDER + string("right_cam_calib.xml");
  fs.open(filename, cv::FileStorage::READ);
  fs["cameraMatrix"] >> r_cameraMatrix;
  fs["distCoeffs"] >> r_distCoeffs;
  fs.release();

  if(!l_cameraMatrix.empty() && !l_distCoeffs.empty() && !r_cameraMatrix.empty() && !r_distCoeffs.empty()) {
    double rms = cv::stereoCalibrate(object_points,
                                 l_image_points,
                                 r_image_points,
                                 l_cameraMatrix,
                                 l_distCoeffs,
                                 r_cameraMatrix,
                                 r_distCoeffs,
                                 l_images[0].size(),
                                 R,
                                 T,
                                 E,
                                 F);
    cout << "Calibrated stereo camera with a RMS cv::error of " << rms << endl;
    return true;
  } else
    return false;
}

void
calibrator::save_info(string filename) {
  cv::FileStorage fs(filename, cv::FileStorage::WRITE);
  fs << "l_cameraMatrix" << l_cameraMatrix;
  fs << "r_cameraMatrix" << r_cameraMatrix;
  fs << "l_distCoeffs" << l_distCoeffs;
  fs << "r_distCoeffs" << r_distCoeffs;
  fs << "R" << R;
  fs << "T" << T;
  fs << "E" << E;
  fs << "F" << F;
  fs.release();
  cout << "Calibration parameters saved to " << filename << endl;
}

Size
calibrator::get_image_size() {
  return l_images[0].size();
}

rectifier::rectifier(string filename, cv::Size image_size) {
  // Read individal camera calibration information from saved XML file
  cv::Mat l_cameraMatrix, l_distCoeffs, r_cameraMatrix, r_distCoeffs, R, T;
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  fs["l_cameraMatrix"] >> l_cameraMatrix;
  fs["l_distCoeffs"] >> l_distCoeffs;
  fs["r_cameraMatrix"] >> r_cameraMatrix;
  fs["r_distCoeffs"] >> r_distCoeffs;
  fs["R"] >> R;
  fs["T"] >> T;
  fs.release();

  if(l_cameraMatrix.empty() || r_cameraMatrix.empty() || l_distCoeffs.empty() || r_distCoeffs.empty() || R.empty() || T.empty())
    cout << "Rectifier: Loading of files not successful" << endl;

  // Calculate transforms for rectifying images
  cv::Mat Rl, Rr, Pl, Pr, Q;
  stereoRectify(l_cameraMatrix, l_distCoeffs, r_cameraMatrix, r_distCoeffs, image_size, R, T, Rl, Rr, Pl, Pr, Q);

  // Calculate pixel maps for efficient rectification of images via lookup tables
  cv::initUndistortRectifyMap(l_cameraMatrix, l_distCoeffs, Rl, Pl, image_size, CV_16SC2, map_l1, map_l2);
  cv::initUndistortRectifyMap(r_cameraMatrix, r_distCoeffs, Rr, Pr, image_size, CV_16SC2, map_r1, map_r2);

  fs.open(filename, cv::FileStorage::APPEND);
  fs << "Rl" << Rl;
  fs << "Rr" << Rr;
  fs << "Pl" << Pl;
  fs << "Pr" << Pr;
  fs << "Q" << Q;
  fs << "map_l1" << map_l1;
  fs << "map_l2" << map_l2;
  fs << "map_r1" << map_r1;
  fs << "map_r2" << map_r2;
  fs.release();
}

void
rectifier::show_rectified(cv::Size image_size) {
  cv::VideoCapture capr(1), capl(2);
  // reduce frame size
  capl.set(cv::CAP_PROP_FRAME_HEIGHT, image_size.height);
  capl.set(cv::CAP_PROP_FRAME_WIDTH, image_size.width);
  capr.set(cv::CAP_PROP_FRAME_HEIGHT, image_size.height);
  capr.set(cv::CAP_PROP_FRAME_WIDTH, image_size.width);

  cv::destroyAllWindows();
  cv::namedWindow("Combo");
  while(char(cv::waitKey(1)) != 'q') {
    // grab raw frames first
    capl.grab();
    capr.grab();
    // decode later so the grabbed frames are less apart in time
    cv::Mat framel, framel_rect, framer, framer_rect;
    capl.retrieve(framel);
    capr.retrieve(framer);

    if(framel.empty() || framer.empty())
      break;

    // Remap images by pixel maps to rectify
    cv::remap(framel, framel_rect, map_l1, map_l2, INTER_LINEAR);
    cv::remap(framer, framer_rect, map_r1, map_r2, INTER_LINEAR);

    // Make a larger image containing the left and right rectified images side-by-side
    cv::Mat combo(image_size.height, 2 * image_size.width, CV_8UC3);
    framel_rect.copyTo(combo(Range::all(), Range(0, image_size.width)));
    framer_rect.copyTo(combo(Range::all(), Range(image_size.width, 2 * image_size.width)));

    // Draw horizontal red lines in the combo image to make comparison easier
    for(int y = 0; y < combo.rows; y += 20) cv::line(combo, cv::Point(0, y), cv::Point(combo.cols, y), cv::Scalar(0, 0, 255));

    cv::imshow("Combo", combo);
  }
  capl.release();
  capr.release();
}

int
main() {
  string filename = DATA_FOLDER + string("stereo_calib.xml");
  /*
  calibrator calib(LEFT_FOLDER, RIGHT_FOLDER, 25.f, 5, 4);
  calib.calc_image_points(true);
  bool done = calib.calibrate();
  if(!done) cout << "Stereo Calibration not successful because individial calibration matrices could
  not be cv::read" << endl; calib.save_info(filename); cv::Size image_size = calib.get_image_size();
  */

  cv::Size image_size(320, 240);
  rectifier rec(filename, image_size);
  rec.show_rectified(image_size);

  return 0;
}
