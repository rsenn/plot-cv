#include "plot-cv.hpp"
#include "color.hpp"
#include "js.hpp"
#include "../qjs-opencv/line.hpp"
#include "../qjs-opencv/geometry.hpp"
#include "matrix.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>

#include <iostream>
#include <list>
#include <map>
#include <algorithm>
#include <string>
#include <set>
#include <chrono>
#include <thread>

static std::vector<std::string> arguments;

using std::chrono::duration;
using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::chrono::steady_clock;
using std::chrono::time_point;

std::ofstream logfile("plot-cv.log", std::ios_base::out | std::ios_base::ate);
std::map<std::string, bool> views{{"imgOriginal", true},
                                  {"imgBlurred", false},
                                  {"imgCanny", true},
                                  {"imgGrayscale", false},
                                  {"imgMorphology", false},
                                  {"imgVector", true}};

std::map<std::string, std::string> trackbars;

static int show_image, image_index, prev_image_index = -1;
static int brightness, exposure, gain;
static size_t num_images;
char keycode = 0;
int levels = 3;
int eps = 8;
int blur = 4;
cv::VideoCapture capWebcam;

void
set_image_index(int i) {
  cv::setTrackbarPos("frame", "imgCanny", i);
  std::cerr << "set_image_index(" << i << ")" << std::endl;
}

/**
 * @brief      F
 *
 * @param[in]  input  The input
 * @param      u      { parameter_description }
 */
void
trackbar(int input, void* u) {
  const char* name = reinterpret_cast<const char*>(const_cast<const void*>(u));
  static int epsilon, blur;
  if(!strcmp(name, "eps"))
    epsilon = input;
  if(!strcmp(name, "blur"))
    blur = input;
};

void
trackbar_prop(int input, void* u) {
  int propId = (int)(ptrdiff_t)(u);
  double value = input;

  capWebcam.set(propId, input / 255);
};

int
create_trackbar(const char* name,
                const std::string& window,
                int* value,
                int count,
                cv::TrackbarCallback onChange = 0,
                const char* caption = nullptr) {
  std::string barName = name;
  std::string windowName = views[window] ? window : "imgOriginal";
  if(caption == nullptr)
    caption = name;
  trackbars[barName] = caption;
  return cv::createTrackbar(caption,
                            windowName,
                            value,
                            count,
                            onChange,
                            const_cast<void*>(static_cast<const void*>(name)));
}

int
create_trackbar(const char* name,
                const std::string& window,
                int* value,
                int count,
                cv::TrackbarCallback onChange,
                int propId) {
  std::string barName = name;
  std::string windowName = views[window] ? window : "imgOriginal";
  trackbars[barName] = name;
  int ret =
      cv::createTrackbar(name, windowName, value, count, onChange, (void*)(ptrdiff_t)propId);
  cv::setTrackbarPos(name, windowName, capWebcam.get(propId) * 255);
  return ret;
}

void
display_image(std::string str, image_type* m) {

  if(!views[str])
    return;

  // std::cout << "Displaying '" << str << "' " << m->cols << "x" << m->rows << std::endl;

  cv::imshow(str.c_str(), *m);
  /* if(m == nullptr)
     return;

   image_type out(cvSize(m->cols, m->rows), m->type());
   if(dptr != nullptr && show_diagnostics) {
     cv::rectangle(*dptr,
                   point_type<int>(50, 50),
                   point_type<int>(out.cols - 50, out.rows - 50),
                   color_type(255, 0, 255, 255));
     cv::rectangle(*dptr,
                   point_type<int>(51, 51),
                   point_type<int>(out.cols - 51, out.rows - 51),
                   color_type(255, 0, 255, 255));
   }
   image_type mask = image_type::zeros(m->size(), CV_8UC1);
   image_type alpha = get_alpha_channel(*dptr);
   dptr->copyTo(out);
   cv::bitwise_and(*m, *m, out, alpha);
   int baseLine;

   if(show_diagnostics)
     std::cerr << "show_image: " << show_image << std::endl;

   if(show_image != -1) {
     cv::Size textSize =
         cv::getTextSize(image_names[show_image], cv::FONT_HERSHEY_PLAIN, 1.5, 2, &baseLine);

     point_type<int> origin(out.cols - 20 - textSize.width, out.rows - 20 - textSize.height +
   baseLine);

     cv::putText(out,
                 image_names[show_image],
                 origin,

                 cv::FONT_HERSHEY_PLAIN,
                 1.5,
                 color_type(0, 255, 0, 255),
                 2,
                 cv::LINE_AA);
   }

   cv::imshow(str.c_str(), out);*/
}

static std::string&
implode(std::vector<std::string>&& elems, const std::string& delim, std::string& s) {
  for(std::vector<std::string>::const_iterator ii = elems.begin(); ii != elems.end(); ++ii) {
    s += (*ii);
    if(ii + 1 != elems.end()) {
      s += delim;
    }
  }

  return s;
}

int
main(int argc, char* argv[]) {
  using std::back_inserter;
  using std::distance;
  using std::endl;
  using std::for_each;
  using std::iterator_traits;
  using std::transform;

  std::map<const char*, const char*> props = {{"name", "test"}, {"length", "4"}};
  arguments = std::vector<std::string>(argv + 1, argv + argc);

  unsigned int ret;
  float wantFPS = 2;
  show_image = ORIGINAL;
  size_t end = arguments.size() > 10 ? 10 : arguments.size();
  std::string range;

  std::cerr << "num files: " << arguments.size() << std::endl;
  std::cerr << "files 0.." << end << ": "
            << implode(std::vector<std::string>(arguments.cbegin(), arguments.cbegin() + end),
                       ", ",
                       range)
            << std::endl;

  num_images = arguments.size();

  std::string filename;
  cv::CommandLineParser parser(argc,
                               argv,
                               "{help h usage ? |      | print this message   }"
                               "{@input         |      | camera number or image file }"
                               "{@width         |    -1| frame width }"
                               "{@height        |    -1| frame height }"
                               "{si save-images |      | save each frame as image }");

  js_init(argc, argv);

  if(ret < 0)
    return ret;

  parser.about("plot-cv v1.0");
  if(parser.has("help")) {
    parser.printMessage();
    return 0;
  }
  bool saveImages = parser.has("save-images");
  std::string input = parser.get<cv::String>(0);
  int width = parser.get<int>(1);
  int height = parser.get<int>(2);
  int camID = isdigit(input.c_str()[0]) ? std::stoi(input, nullptr, 10) : -1;

  if(camID == -1)
    filename = input;

  capWebcam.open(camID, cv::CAP_V4L);

  image_type imgInput;
  if(camID >= 0) {
    // capWebcam.open((int)camID, (int)cv::CAP_V4L2); // declare a VideoCapture
    // object and associate to webcam, 0 => use 1st webcam

    if(capWebcam.isOpened() == false) {
      logfile << "error: capWebcam not accessed successfully\n\n";
      getchar();
      return (0);
    }

    if(width != -1 && height != -1) {
      capWebcam.set(cv::CAP_PROP_FRAME_WIDTH, width);
      capWebcam.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    }

    capWebcam.set(cv::CAP_PROP_FPS, wantFPS);
    double fps = capWebcam.get(cv::CAP_PROP_FPS);
    std::cout << "Frames per second: " << fps << std::endl;
    int width = capWebcam.get(cv::CAP_PROP_FRAME_WIDTH);
    int height = capWebcam.get(cv::CAP_PROP_FRAME_HEIGHT);
    std::cout << "Resolution: " << width << "x" << height << std::endl;
  } else {
    imgInput = cv::imread(filename.empty() ? "input.png" : filename);
    imgInput.copyTo(imgRaw);
    //   num_images = argc - 2;
  }

  int startTime = -1;

  std::vector<std::string> visible;

  std::for_each(views.cbegin(),
                views.cend(),
                [&visible](const std::map<std::string, bool>::value_type& pair) {
                  if(pair.second)
                    visible.push_back(pair.first);
                });

  for(const auto& window : visible) {
    std::cout << "Create window '" << window << "'" << std::endl;
    cv::namedWindow(window, cv::WINDOW_NORMAL);
  }

  create_trackbar("brightness",
                  "imgOriginal",
                  &brightness,
                  255,
                  trackbar_prop,
                  (int)cv::CAP_PROP_BRIGHTNESS);
  create_trackbar(
      "exposure", "imgOriginal", &exposure, 255, trackbar_prop, (int)cv::CAP_PROP_EXPOSURE);
  create_trackbar("gain", "imgOriginal", &gain, 255, trackbar_prop, (int)cv::CAP_PROP_GAIN);
  create_trackbar("frame", "imgCanny", &image_index, num_images - 1, trackbar, "frame");
  create_trackbar("threshold2", "imgCanny", &thresh2, 255, trackbar, "thres2");
  create_trackbar("morphology_kernel_size",
                  "imgMorphology",
                  &config.morphology_kernel_size,
                  2,
                  trackbar,
                  "Morphology kernel size");
  create_trackbar("morphology_enable",
                  "imgMorphology",
                  &morphology_enable,
                  1,
                  trackbar,
                  "Morphology enable");
  create_trackbar("morphology_operator",
                  "imgMorphology",
                  &config.morphology_operator,
                  3,
                  trackbar,
                  "Morphology operator");
  create_trackbar("blur_sigma", "imgBlurred", &config.blur_sigma, 300, trackbar, "blur sigma");
  create_trackbar("blur_kernel_size",
                  "imgBlurred",
                  &config.blur_kernel_size,
                  2,
                  trackbar,
                  "Blur kernel size");
  create_trackbar("hough_rho", "imgVector", &config.hough_rho, 10, trackbar, "Hough rho");
  create_trackbar("hough_theta", "imgCanny", &config.hough_theta, 360, trackbar, "Hough theta");
  create_trackbar("hough_threshold",
                  "imgVector",
                  &config.hough_threshold,
                  1000,
                  trackbar,
                  "Hough threshold");
  create_trackbar("hough_minlinelen",
                  "imgVector",
                  &config.hough_minlinelen,
                  1000,
                  trackbar,
                  "Hough minLineLen");
  create_trackbar("hough_maxlinega",
                  "imgVector",
                  &config.hough_maxlinegap,
                  1000,
                  trackbar,
                  "Hough maxLineGap");

  time_point<steady_clock> start = steady_clock::now();
  int frames = 0;
  image_index = 0;

  while(keycode != 27) { // until the Esc key is pressed or webcam connection is lost

    switch(keycode) {
      case 27: {
        return 0;
      }
      case 28: // right arrow
      case 'd':
      case 'D': set_image_index((image_index + 1) % (num_images)); break;
      case 29: // left arrow
      case 'a':
      case 'A': set_image_index(image_index > 0 ? image_index - 1 : num_images - 1); break;
      case 30: // up arrow
      case 'w':
      case 'W': set_image_index(0); break;
      case 31:
      case 's':
      case 'S': // down arrow
        set_image_index(num_images - 1);
        break;

      case 'c':
      case 'C': show_image = CANNY; break;
      case 'm': morphology_enable = !morphology_enable; break;
      case 'M': config.morphology_operator = !config.morphology_operator; break;
      case 'x':
      case 'X': show_diagnostics = !show_diagnostics; break;
      case 'p':
      case 'P': show_image = MORPHOLOGY; break;
      case 'o':
      case 'O': show_image = ORIGINAL; break;
      case 'g':
      case 'G': show_image = GRAYSCALE; break;
      case 'v':
      case 'V':
        show_image--;
        show_image &= 0b11;
        break;
      case 81:
      case -104:
        show_image++;
        show_image &= 0b11;
        break;
      case -1: break;
      default:
        if(show_diagnostics)
          std::cerr << "Unknown keycode: '" << (int)(char)keycode << "'" << std::endl;
        break;
    }
    time_point<steady_clock> now = steady_clock::now();
    auto diff = now - start;
    auto end = now + milliseconds((int)(1000.0 / wantFPS));

    frames++;

    if(diff >= seconds(1)) {
      start = now;
#ifdef DEBUG_OUTPUT_

      std::cout << "FPS: " << frames << std::endl;
#endif
      frames = 0;
    }

    bool blnFrameReadSuccessfully = false;
    if(capWebcam.isOpened()) {
      blnFrameReadSuccessfully = capWebcam.grab();
      capWebcam.retrieve(imgRaw);
    } else {
      if(image_index != prev_image_index) {
        filename = arguments[image_index];
        std::cout << "Read image '" << filename << "' ..." << std::endl;
        imgInput = cv::imread(filename);
      }
      if(!imgInput.empty() && imgInput.cols > 0 && imgInput.rows > 0) {
        imgInput.copyTo(imgRaw);
        prev_image_index = image_index;
        blnFrameReadSuccessfully = true;
      }
    }

    if(!blnFrameReadSuccessfully) {                     // if frame not read successfully
      logfile << "error: frame not read from webcam\n"; // print error message
                                                        // to std out
      break;                                            // and jump out of while loop
    }

    int frame = capWebcam.get(cv::CAP_PROP_POS_FRAMES);
    int msec = capWebcam.get(cv::CAP_PROP_POS_MSEC);

    if(startTime == -1)
      startTime = msec;

#ifdef DEBUG_OUTPUT_
    std::cout << "Video frame#" << frame << " pos " << (msec - startTime) << "ms" << std::endl;
#endif
    image_type imgOutput;

    imgRaw.copyTo(imgOutput);
    imgRaw.copyTo(imgOriginal);

    if(saveImages)
      write_image(imgOutput);

    if(dptr == nullptr) {
      imgVector =
          image_type::zeros(cvSize(imgOriginal.cols, imgOriginal.rows), imgOriginal.type());
      dptr = &imgVector;
    }
    process_image(std::function<void(std::string, cv::Mat*)>(&display_image), show_image);

    duration<int, std::milli> sleep = duration_cast<milliseconds>(end - steady_clock::now());
    int millis = sleep.count(); // > 0 ? sleep.count() : 0;

#ifdef DEBUG_OUTPUT_
    std::cout << "Sleep " << millis << "ms" << std::endl;
#endif
    keycode = cv::waitKey(millis > 0 ? millis : 1);

    // std::this_thread::sleep_until(end);

  } // end while

  return (0);
}
