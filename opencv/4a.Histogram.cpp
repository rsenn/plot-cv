#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace std;

int
main() {
  VideoCapture cap(0);
  if(!cap.isOpened()) {
    printf("ERROR: khong the mo camera\r\n");
    return -1;
  }
  namedWindow("Camera", WINDOW_NORMAL);
  resizeWindow("Camera", 300, 300);
  namedWindow("Histogram", WINDOW_NORMAL);
  resizeWindow("Histogram", 300, 300);

  while(1) {
    Mat frame;
    cap >> frame;
    if(frame.empty()) {
      printf("ERROR: khong the bat hinh\r\n");
      exit(-1); // Thoat khoi chuong trinh
    }
    // Separate the source image in its three R,G and B planes
    vector<Mat> bgr_planes;
    split(frame, bgr_planes);
    // Output matrixes
    Mat b_hist, g_hist, r_hist;
    // our bins to have the same size (uniform) and to clear the histograms in the beginning
    bool uniform = true, accumulate = false;
    // Set the range of values (between 0 and 255 )
    int histSize = 256;
    float range[] = {0, 256}; // the upper boundary is exclusive
    const float* histRange = {range};
    // calculate the histograms
    calcHist(&bgr_planes[0], // source array
             1,              // number of source arrays
             0,              // channel
             Mat(),          // A mask to be used on the source array
             b_hist,         // output
             1,              // dimensionality
             &histSize,      // The number of bins per each used dimension
             &histRange,     // The range of values to be measured per each dimension
             uniform,        // Flag indicating whether the histogram is uniform or not
             accumulate      // Accumulation flag. If it is set, the histogram is not cleared in the
                             // beginning when it is allocated.
    );
    calcHist(&bgr_planes[1], 1, 0, Mat(), g_hist, 1, &histSize, &histRange, uniform, accumulate);
    calcHist(&bgr_planes[2], 1, 0, Mat(), r_hist, 1, &histSize, &histRange, uniform, accumulate);
    // Create an image to display the histograms
    int hist_w = 512, hist_h = 400;
    int bin_w = cvRound((double)hist_w / histSize);
    Mat histImage(hist_h, hist_w, CV_8UC3, Scalar(0, 0, 0));
    // Normalize the histogram so its values fall in the range of histImage size
    normalize(b_hist,         // Input array
              b_hist,         // Output array
              0,              // lower limit
              histImage.rows, // upper limit, height of histImage
              NORM_MINMAX,    // type of normalization
              -1,             // output normalized array will be the same type as the input
              Mat()           // Optional mask
    );
    normalize(g_hist, g_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat());
    normalize(r_hist, r_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat());
    // Draw lines of histogram
    for(int i = 1; i < histSize; i++) {
      line(histImage, Point(bin_w * (i - 1), hist_h - cvRound(b_hist.at<float>(i - 1))), Point(bin_w * (i), hist_h - cvRound(b_hist.at<float>(i))), Scalar(255, 0, 0), 2, 8, 0);
      line(histImage, Point(bin_w * (i - 1), hist_h - cvRound(g_hist.at<float>(i - 1))), Point(bin_w * (i), hist_h - cvRound(g_hist.at<float>(i))), Scalar(0, 255, 0), 2, 8, 0);
      line(histImage, Point(bin_w * (i - 1), hist_h - cvRound(r_hist.at<float>(i - 1))), Point(bin_w * (i), hist_h - cvRound(r_hist.at<float>(i))), Scalar(0, 0, 255), 2, 8, 0);
    }
    /*-----------------------------------------------*/
    imshow("Camera", frame);
    imshow("Histogram", histImage);
    waitKey(1);
  }
  printf("Tat camera\r\n");
  cap.release();
  destroyAllWindows();
  return 0;
}
