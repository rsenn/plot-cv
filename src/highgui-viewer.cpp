
#include "plot-cv.h"
#include "color.h"
#include "js.h"
#include "jsbindings.h"
#include "line.h"
#include "geometry.h"
#include "matrix.h"
#include <opencv2/core.hpp>

#include <opencv2/imgproc.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <iostream>
#include <list>
#include <map>
#include <algorithm>
#include <string>

extern "C" cv::Mat* dptr;

std::ofstream logfile("plot-cv.log", std::ios_base::out | std::ios_base::ate);

static int show_image;

char keycode = 0;
int levels = 3;
int eps = 8;
int blur = 4;

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
display_image(std::string str, image_type* m) {

  cv::imshow(str.c_str(), *m);
  /* if(m == nullptr)
     return;

   image_type out(cvSize(m->cols, m->rows), m->type());
   if(dptr != nullptr && show_diagnostics) {
     cv::rectangle(*dptr,
                   point2i_type(50, 50),
                   point2i_type(out.cols - 50, out.rows - 50),
                   color_type(255, 0, 255, 255));
     cv::rectangle(*dptr,
                   point2i_type(51, 51),
                   point2i_type(out.cols - 51, out.rows - 51),
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

     point2i_type origin(out.cols - 20 - textSize.width, out.rows - 20 - textSize.height +
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

///////////////////////////////////////////////////////////////////////////////////////////////////
int
main(int argc, char* argv[]) {
  using std::back_inserter;
  using std::distance;
  using std::endl;
  using std::for_each;
  using std::iterator_traits;
  using std::transform;

  std::map<const char*, const char*> props = {{"name", "test"}, {"length", "4"}};

  unsigned int ret;
  show_image = 0;

  int camID = (argc > 1 && isdigit(argv[1][0])) ? strtol(argv[1], nullptr, 10) : -1;
  std::string filename;

  js_init(argc, argv);

  if(ret < 0)
    return ret;

  if(camID == -1 && argc > 1)
    filename = argv[1];
  cv::VideoCapture capWebcam(camID);
  image_type imgInput;
  if(camID >= 0) {
    // capWebcam.open((int)camID, (int)cv::CAP_V4L2); // declare a VideoCapture
    // object and associate to webcam, 0 => use 1st webcam

    if(capWebcam.isOpened() == false) { // check if VideoCapture object was
                                        // associated to webcam successfully
      logfile << "error: capWebcam not accessed successfully\n\n"; // if not, print
                                                                   // error message
                                                                   // to std out
      getchar();  // may have to modify this line if not using Windows
      return (0); // and exit program
    }
  } else {
    imgInput = cv::imread(filename.empty() ? "input.png" : filename);
  }

  cv::namedWindow("imgBlurred", CV_WINDOW_AUTOSIZE);
  cv::namedWindow("imgMorphology", CV_WINDOW_AUTOSIZE);
  cv::namedWindow("imgCanny", CV_WINDOW_AUTOSIZE);
  cv::createTrackbar("threshold", "imgCanny", &thresh, 255, trackbar, (void*)"thres");
  cv::createTrackbar("threshold2", "imgCanny", &thresh2, 255, trackbar, (void*)"thres2");
  cv::createTrackbar("blur", "imgBlurred", &blur, 7, trackbar, (void*)"blur");
  // cv::createTrackbar("Image", "img", &show_image, 4, trackbar, (void*)"Image Index");
  cv::createTrackbar("morphology_kernel_size",
                     "imgMorphology",
                     &config.morphology_kernel_size,
                     2,
                     trackbar,
                     (void*)"Morphology kernel size");
  cv::createTrackbar("morphology_enable",
                     "imgMorphology",
                     &morphology_enable,
                     2,
                     trackbar,
                     (void*)"Morphology enable");
  cv::createTrackbar("morphology_operator",
                     "imgMorphology",
                     &morphology_operator,
                     3,
                     trackbar,
                     (void*)"Morphology operator");
  cv::createTrackbar("blur_kernel_size",
                     "imgBlurred",
                     &config.blur_kernel_size,
                     1,
                     trackbar,
                     (void*)"Blur kernel size");

  cv::createTrackbar("hough_rho", "imgCanny", &config.hough_rho, 10, trackbar, (void*)"Hough rho");
  cv::createTrackbar(
      "hough_theta", "imgCanny", &config.hough_theta, 360, trackbar, (void*)"Hough theta");
  cv::createTrackbar("hough_threshold",
                     "imgCanny",
                     &config.hough_threshold,
                     1000,
                     trackbar,
                     (void*)"Hough threshold");
  cv::createTrackbar("hough_minlinelen",
                     "imgCanny",
                     &config.hough_minlinelen,
                     1000,
                     trackbar,
                     (void*)"Hough minLineLen");
  cv::createTrackbar("hough_maxlinega",
                     "imgCanny",
                     &config.hough_maxlinegap,
                     1000,
                     trackbar,
                     (void*)"Hough maxLineGap");
  mptr = &imgOriginal;

  while(keycode != 27) { // until the Esc key is pressed or webcam connection is lost

    switch(keycode) {
      case 27: return 0;
      case 'c':
      case 'C': show_image = CANNY; break;
      case 'm': morphology_enable = !morphology_enable; break;
      case 'M': morphology_operator = !morphology_operator; break;
      case 'd':
      case 'D': show_diagnostics = !show_diagnostics; break;
      case 'p':
      case 'P': show_image = MORPHOLOGY; break;
      case 'o':
      case 'O': show_image = ORIGINAL; break;
      case 'g':
      case 'G': show_image = GRAYSCALE; break;
      case 83:
      case -106:
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

    // std::cerr << "Show image: " << show_image << std::endl;

    bool blnFrameReadSuccessfully = false;
    if(capWebcam.isOpened()) {
      blnFrameReadSuccessfully = capWebcam.read(imgRaw); // get next frame
                                                         //
    } else {
      imgInput.copyTo(imgRaw);
      blnFrameReadSuccessfully = imgRaw.cols > 0 && imgRaw.rows > 0;
    }

    if(!blnFrameReadSuccessfully || imgRaw.empty()) {   // if frame not read successfully
      logfile << "error: frame not read from webcam\n"; // print error message
                                                        // to std out
      break;                                            // and jump out of while loop
    }

    image_type imgOutput;

    imgRaw.copyTo(imgOutput);
    imgRaw.copyTo(imgOriginal);

    write_image(imgOutput);

    if(dptr == nullptr) {
      imgVector = image_type::zeros(cvSize(imgOriginal.cols, imgOriginal.rows), imgOriginal.type());
      dptr = &imgVector;
    }
    process_image(std::function<void(std::string, cv::Mat*)>(&display_image), show_image);
    keycode = cv::waitKey(200);

    /*
          if(show_image == ORIGINAL)
            display_image(imgOriginal);
          else if(show_image == GRAYSCALE)
            display_image(imgGrayscale);*/

    //  keycode = cv::waitKey(1); // delay (in ms) and get key press, if any
  } // end while

  return (0);
}
