#include <camera_c.h>
#include <mask_c.h>
#include <diff_c.h>
#include <util.h>
#include <libv4l2.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <qelapsedtimer.h>

//using namespace cv;

QElapsedTimer telapsed;

camera_c::camera_c(QObject* parent, int width, int height, int res, int cv::threshold, int thresholdZone)
    : parent(parent), width(width), height(height), resolution(res), cv::threshold(threshold), thresholdZone(thresholdZone) {
  isLearning = false;
  enabled = false;
  connect(parent, SIGNAL(snap()), this, SLOT(snap()));
  connect(this, SIGNAL(setMarkerVisible(uint, uint, bool)), parent, SLOT(setMarkerVisible(uint, uint, bool)));

  dx = ((double)width / resolution);
  dy = ((double)height / resolution);
}

void
camera_c::init() {
  // NEEDED ON RASPI : sud
  capture = new cv::VideoCapture(0);
  // cv::VideoCapture cv::cap(0); // open the default camera
  if(!capture->isOpened()) { // check if we succeeded
    qDebug() << "camera not found";
    exit(0);
  } else
    qDebug() << "camera ok";

  capture->set(cv::CAP_PROP_FRAME_WIDTH, width);
  capture->set(cv::CAP_PROP_FRAME_HEIGHT, height);

  width = capture->get(cv::CAP_PROP_FRAME_WIDTH);
  height = capture->get(cv::CAP_PROP_FRAME_HEIGHT);

  emit setSize(width, height);

  capture->set(cv::CAP_PROP_FPS, 60);

  int fps = capture->get(cv::CAP_PROP_FPS);

  snap();

  updTimer = new QTimer();
  connect(updTimer, SIGNAL(timeout()), this, SLOT(update()));
  updTimer->start(1000 / fps); // about 24Hz
}

void
camera_c::config(void) {
  struct v4l2_capability video_cap;

  int exposition = ((MainWindow*)parent)->getExposition();
  if(exposition <= 0)
    exposition = 1;

  // open capture
  int descriptor = v4l2_open("/dev/video0", O_RDWR);

  if(ioctl(descriptor, VIDIOC_QUERYCAP, &video_cap) == -1)
    perror("cam_info: Can't get capabilities");
  else {
    QString buf, res;

    for(int i = 0; i < 32; i++) {
      if(video_cap.card[i] == 0)
        break;
      res.append((char)video_cap.card[i]);
    }
    qDebug() << res;
    //   printf("Minimum size:\t%d x %d\n", video_cap.minwidth, video_cap.minheight);
    //  printf("Maximum size:\t%d x %d\n", video_cap.maxwidth, video_cap.maxheight);
  }

  // manual exposure control
  v4l2_control c;
  c.id = V4L2_CID_EXPOSURE_AUTO;
  c.value = V4L2_EXPOSURE_MANUAL;
  v4l2_ioctl(descriptor, VIDIOC_S_CTRL, &c);

  c.id = V4L2_CID_EXPOSURE_ABSOLUTE;
  c.value = exposition;
  v4l2_ioctl(descriptor, VIDIOC_S_CTRL, &c);
}

void
camera_c::snap() {
  if(capture->grab()) {

    capture->retrieve(imageSnap);

    if(!imageSnap.data) { // Check for invalid input
      qDebug() << "Could not open or find the image";
    } else {
      // Show our image inside cv::it.
      qImageSnap = Mat2QImage(imageSnap);
      qImageSnap.save(((MainWindow*)parent)->imageFileName);

      // emit dataReady(2);
      emit sendSnap(qImageSnap);
    }
  }
}

void
camera_c::update(void) {
  if(capture->grab()) {
    capture->retrieve(image);

    if(!image.data) { // Check for invalid input
      qDebug() << "Could not open or find the image";
    } else {
      if(imageSnap.data) {
        cv::absdiff(imageSnap, image, buf);

        imageDiff = buf.clone();
        cv::threshold(imageDiff, imageDiff, cv::threshold, 255, cv::THRESH_BINARY_INV);
        cv::cvtColor(imageDiff, imageDiff, cv::COLOR_BGR2GRAY);
        checkZones();

        qImageDiff = Mat2QImage(imageDiff);
        //                pixmapImageDiff = QPixmap::fromImage(qImageDiff);

        emit sendDiff(qImageDiff);

      }

      else
        emit dataReady(1);
    }
  }
}

void
camera_c::shutdown(void) {
  qDebug() << "shuting down";

  enabled = false;

  connect(((MainWindow*)parent)->lbl_imageDiff,
          SIGNAL(destroyed(QObject*)),
          ((MainWindow*)parent)->lbl_imageSnap,
          SLOT(deleteLater()));
  // connect(((MainWindow*)parent)->lbl_imageSnap,SIGNAL(destroyed(QObject*)),this,SLOT(deleteLater()));
  connect(((MainWindow*)parent)->lbl_imageSnap, SIGNAL(destroyed(QObject*)), this->updTimer, SLOT(stop()));
  connect(((MainWindow*)parent)->lbl_imageSnap, SIGNAL(destroyed(QObject*)), this->updTimer, SLOT(deleteLater()));
  connect(this->updTimer, SIGNAL(destroyed(QObject*)), this, SLOT(deleteLater()));

  connect(this, SIGNAL(destroyed(QObject*)), ((MainWindow*)parent), SLOT(init()));
  // connect(((MainWindow*)parent)->lbl_imageSnap,SIGNAL(destroyed(QObject*)),((MainWindow*)parent),SLOT(init()));

  ((MainWindow*)parent)->lbl_imageDiff->close();
}

void
camera_c::checkZones(void) {
  if(!enabled)
    return;

  int zoneCmpt = 0;

  if(isLearning) {
    for(int xi = 0; xi < resolution; xi++)
      for(int yi = 0; yi < resolution; yi++) {
        if(((MainWindow*)parent)->lbl_imageSnap->isZoneMarked(xi, yi)) {

          if(getZoneValue(xi, yi) < 255 - thresholdZone) {

            detectedZone[xi][yi] = true;
            emit setMarkerVisible(xi, yi, true);
          }
        }
      }
  } else {
    for(int xi = 0; xi < resolution; xi++)
      for(int yi = 0; yi < resolution; yi++) {
        if(((MainWindow*)parent)->lbl_imageSnap->isZoneMarked(xi, yi)) {
          if(getZoneValue(xi, yi) < 255 - thresholdZone) {
            emit setMarkerVisible(xi, yi, true);
            zoneCmpt++;
          } else {
            emit setMarkerVisible(xi, yi, false);
          }
        }
      }

    if(zoneCmpt > 0)
      emit triggerSignal(zoneCmpt);
  }
}

int
camera_c::getZoneValue(int X, int Y) {
  int val;
  cv::Mat subDiff = imageDiff(cv::Rect(dx * X, dy * Y, dx, dy));
  cv::Scalar tempVal = cv::mean(subDiff);
  val = (int)tempVal.val[0];

  return val;
}

void
camera_c::startLearning(void) {
  isLearning = true;

  detectedZone.cv::resize(resolution, std::vector<bool>(resolution, false));
}

void
camera_c::stopLearning(void) {
  isLearning = false;
  sendZone(detectedZone);
}

void
camera_c::enable(bool status) {

  enabled = status;

  qDebug() << "enable" << status;
}

camera_c::~camera_c(void) {
  for(unsigned int xi = 0; xi < ((MainWindow*)parent)->markers.size(); xi++)
    for(unsigned int yi = 0; yi < ((MainWindow*)parent)->markers[0].size(); yi++)
      ((MainWindow*)parent)->markers[xi][yi]->deleteLater();

  ((MainWindow*)parent)->markers.clear();

  capture->release();
  qDebug() << "cam destroyed";
}
