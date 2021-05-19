#include <iostream> // for standard I/O
#include <string>   // for strings
#include <iomanip>  // for controlling float print precision
#include <sstream>  // string to number conversion

#include <opencv2/core/core.hpp>       // Basic OpenCV structures (Mat, Scalar)
#include <opencv2/imgproc/imgproc.hpp> // Gaussian Blur
#include <opencv2/highgui/highgui.hpp> // OpenCV window I/O
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/videoio.hpp>

using namespace std;
// using namespace cv;

double getPSNR(const cv::Mat& I1, const cv::Mat& I2);
cv::Scalar getMSSIM(const cv::Mat& I1, const cv::Mat& I2);

static void
help() {
  cout << "------------------------------------------------------------------------------" << endl
       << "This program shows how to cv::read a video file with OpenCV. In addition, it "
       << "tests the similarity of two input videos first with PSNR, and for the frames "
       << "below a PSNR trigger value, also with MSSIM." << endl
       << "Usage:" << endl
       << "./video-source referenceVideo useCaseTestVideo PSNR_Trigger_Value Wait_Between_Frames " << endl
       << "--------------------------------------------------------------------------" << endl
       << endl;
}

int
main(int argc, char* argv[]) {
  help();

  if(argc != 5) {
    cout << "Not enough parameters" << endl;
    return -1;
  }

  stringstream conv;

  const string sourceReference = argv[1], sourceCompareWith = argv[2];
  int psnrTriggerValue, delay;
  conv << argv[3] << endl << argv[4]; // put in the strings
  conv >> psnrTriggerValue >> delay;  // take out the numbers

  char c;
  int frameNum = -1; // Frame counter

  cv::VideoCapture captRefrnc(sourceReference), captUndTst(sourceCompareWith);

  if(!captRefrnc.isOpened()) {
    cout << "Could not open reference " << sourceReference << endl;
    return -1;
  }

  if(!captUndTst.isOpened()) {
    cout << "Could not open case test " << sourceCompareWith << endl;
    return -1;
  }

  cv::Size refS = cv::Size((int)captRefrnc.get(cv::CAP_PROP_FRAME_WIDTH), (int)captRefrnc.get(cv::CAP_PROP_FRAME_HEIGHT)),
           uTSi = cv::Size((int)captUndTst.get(cv::CAP_PROP_FRAME_WIDTH), (int)captUndTst.get(cv::CAP_PROP_FRAME_HEIGHT));

  if(refS != uTSi) {
    cout << "Inputs have different size!!! Closing." << endl;
    return -1;
  }

  const char* WIN_UT = "Under Test";
  const char* WIN_RF = "Reference";

  // Windows
  cv::namedWindow(WIN_RF, cv::WINDOW_AUTOSIZE);
  cv::namedWindow(WIN_UT, cv::WINDOW_AUTOSIZE);
  cvMoveWindow(WIN_RF, 400, 0);        // 750,  2 (bernat =0)
  cvMoveWindow(WIN_UT, refS.width, 0); // 1500, 2

  cout << "Reference frame resolution: Width=" << refS.width << "  Height=" << refS.height
       << " of nr#: " << captRefrnc.get(cv::CAP_PROP_FRAME_COUNT) << endl;

  cout << "PSNR trigger value " << setiosflags(ios::fixed) << setprecision(3) << psnrTriggerValue << endl;

  cv::Mat frameReference, frameUnderTest;
  double psnrV;
  cv::Scalar mssimV;

  for(;;) { // Show the image captured in the window and repeat
    captRefrnc >> frameReference;
    captUndTst >> frameUnderTest;

    if(frameReference.empty() || frameUnderTest.empty()) {
      cout << " < < <  Game over!  > > > ";
      break;
    }

    ++frameNum;
    cout << "Frame: " << frameNum << "# ";

    ///////////////////////////////// PSNR ////////////////////////////////////////////////////
    psnrV = getPSNR(frameReference, frameUnderTest);
    cout << setiosflags(ios::fixed) << setprecision(3) << psnrV << "dB";

    //////////////////////////////////// MSSIM /////////////////////////////////////////////////
    if(psnrV < psnrTriggerValue && psnrV) {
      mssimV = getMSSIM(frameReference, frameUnderTest);

      cout << " MSSIM: "
           << " R " << setiosflags(ios::fixed) << setprecision(2) << mssimV.val[2] * 100 << "%"
           << " G " << setiosflags(ios::fixed) << setprecision(2) << mssimV.val[1] * 100 << "%"
           << " B " << setiosflags(ios::fixed) << setprecision(2) << mssimV.val[0] * 100 << "%";
    }

    cout << endl;

    ////////////////////////////////// Show Image /////////////////////////////////////////////
    cv::imshow(WIN_RF, frameReference);
    cv::imshow(WIN_UT, frameUnderTest);

    c = (char)cv::waitKey(delay);
    if(c == 27)
      break;
  }

  return 0;
}

double
getPSNR(const cv::Mat& I1, const cv::Mat& I2) {
  cv::Mat s1;
  cv::absdiff(I1, I2, s1);  // |I1 - I2|
  s1.convertTo(s1, CV_32F); // cannot make a square on 8 bits
  s1 = s1.mul(s1);          // |I1 - I2|^2

  cv::Scalar s = cv::sum(s1); // cv::sum elements per channel

  double sse = s.val[0] + s.val[1] + s.val[2]; // cv::sum channels

  if(sse <= 1e-10) // for small values return zero
    return 0;
  else {
    double mse = sse / (double)(I1.channels() * I1.total());
    double psnr = 10.0 * log10((255 * 255) / mse);
    return psnr;
  }
}

cv::Scalar
getMSSIM(const cv::Mat& i1, const cv::Mat& i2) {
  const double C1 = 6.5025, C2 = 58.5225;
  /***************************** INITS **********************************/
  int d = CV_32F;

  cv::Mat I1, I2;
  i1.convertTo(I1, d); // cannot calculate on one byte large values
  i2.convertTo(I2, d);

  cv::Mat I2_2 = I2.mul(I2);  // I2^2
  cv::Mat I1_2 = I1.mul(I1);  // I1^2
  cv::Mat I1_I2 = I1.mul(I2); // I1 * I2

  /*************************** END INITS **********************************/

  cv::Mat mu1, mu2; // PRELIMINARY COMPUTING
  cv::GaussianBlur(I1, mu1, cv::Size(11, 11), 1.5);
  cv::GaussianBlur(I2, mu2, cv::Size(11, 11), 1.5);

  cv::Mat mu1_2 = mu1.mul(mu1);
  cv::Mat mu2_2 = mu2.mul(mu2);
  cv::Mat mu1_mu2 = mu1.mul(mu2);

  cv::Mat sigma1_2, sigma2_2, sigma12;

  cv::GaussianBlur(I1_2, sigma1_2, cv::Size(11, 11), 1.5);
  sigma1_2 -= mu1_2;

  cv::GaussianBlur(I2_2, sigma2_2, cv::Size(11, 11), 1.5);
  sigma2_2 -= mu2_2;

  cv::GaussianBlur(I1_I2, sigma12, cv::Size(11, 11), 1.5);
  sigma12 -= mu1_mu2;

  ///////////////////////////////// FORMULA ////////////////////////////////
  cv::Mat t1, t2, t3;

  t1 = 2 * mu1_mu2 + C1;
  t2 = 2 * sigma12 + C2;
  t3 = t1.mul(t2); // t3 = ((2*mu1_mu2 + C1).*(2*sigma12 + C2))

  t1 = mu1_2 + mu2_2 + C1;
  t2 = sigma1_2 + sigma2_2 + C2;
  t1 = t1.mul(t2); // t1 =((mu1_2 + mu2_2 + C1).*(sigma1_2 + sigma2_2 + C2))

  cv::Mat ssim_map;
  cv::divide(t3, t1, ssim_map); // ssim_map =  t3./t1;

  cv::Scalar mssim = cv::mean(ssim_map); // mssim = average of ssim map
  return mssim;
}
