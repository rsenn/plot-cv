#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/flann.hpp>
#include <opencv2/xfeatures2d.hpp>

//using namespace cv;
//using namespace cv::xfeatures2d;

////////////////////////////////////////////////////
// This program demonstrates the GMS matching strategy.
int
main(int argc, char* argv[]) {
  const char* keys = "{ h help        |                  | print help message  }"
                     "{ l left        |                  | specify left (reference) image  }"
                     "{ r right       |                  | specify right (query) image }"
                     "{ camera        | 0                | specify the camera device number }"
                     "{ nfeatures     | 10000            | specify the maximum number of ORB features }"
                     "{ fastThreshold | 20               | specify the FAST cv::threshold }"
                     "{ drawSimple    | true             | do not draw not matched keypoints }"
                     "{ withRotation  | false            | take rotation into account }"
                     "{ withScale     | false            | take scale into account }";

  cv::CommandLineParser cmd(argc, argv, keys);
  if(cmd.has("help")) {
    std::cout << "Usage: gms_matcher [options]" << std::endl;
    std::cout << "Available options:" << std::endl;
    cmd.printMessage();
    return EXIT_SUCCESS;
  }

  Ptr<Feature2D> orb = ORB::create(cmd.get<int>("nfeatures"));
  orb.dynamicCast<cv::ORB>()->setFastThreshold(cmd.get<int>("fastThreshold"));
  Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

  if(!cmd.get<cv::String>("left").empty() && !cmd.get<String>("right").empty()) {
    cv::Mat imgL = cv::imread(cmd.get<cv::String>("left"));
    cv::Mat imgR = cv::imread(cmd.get<cv::String>("right"));

    std::vector<KeyPoint> kpRef, kpCur;
    cv::Mat descRef, descCur;
    orb->detectAndCompute(imgL, cv::noArray(), kpRef, descRef);
    orb->detectAndCompute(imgR, cv::noArray(), kpCur, descCur);

    std::vector<DMatch> matchesAll, matchesGMS;
    matcher->match(descCur, descRef, matchesAll);

    matchGMS(imgR.size(),
             imgL.size(),
             kpCur,
             kpRef,
             matchesAll,
             matchesGMS,
             cmd.get<bool>("withRotation"),
             cmd.get<bool>("withScale"));
    std::cout << "matchesGMS: " << matchesGMS.size() << std::endl;

    cv::Mat frameMatches;
    if(cmd.get<bool>("drawSimple"))
      drawMatches(imgR,
                  kpCur,
                  imgL,
                  kpRef,
                  matchesGMS,
                  frameMatches,
                  cv::Scalar::all(-1),
                  cv::Scalar::all(-1),
                  std::vector<char>(),
                  DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    else
      drawMatches(imgR, kpCur, imgL, kpRef, matchesGMS, frameMatches);
    cv::imshow("Matches GMS", frameMatches);
    cv::waitKey();
  } else {
    std::vector<KeyPoint> kpRef;
    cv::Mat descRef;

    cv::VideoCapture capture(cmd.get<int>("camera"));
    // Camera warm-up
    for(int i = 0; i < 10; i++) {
      cv::Mat frame;
      capture >> frame;
    }

    cv::Mat frameRef;
    for(;;) {
      cv::Mat frame;
      capture >> frame;

      if(frameRef.empty()) {
        frame.copyTo(frameRef);
        orb->detectAndCompute(frameRef, cv::noArray(), kpRef, descRef);
      }

      TickMeter tm;
      tm.start();
      std::vector<KeyPoint> kp;
      cv::Mat desc;
      orb->detectAndCompute(frame, cv::noArray(), kp, desc);
      tm.stop();
      double t_orb = tm.getTimeMilli();

      tm.reset();
      tm.start();
      std::vector<DMatch> matchesAll, matchesGMS;
      matcher->match(desc, descRef, matchesAll);
      tm.stop();
      double t_match = tm.getTimeMilli();

      matchGMS(frame.size(),
               frameRef.size(),
               kp,
               kpRef,
               matchesAll,
               matchesGMS,
               cmd.get<bool>("withRotation"),
               cmd.get<bool>("withScale"));
      tm.stop();
      cv::Mat frameMatches;
      if(cmd.get<bool>("drawSimple"))
        drawMatches(frame,
                    kp,
                    frameRef,
                    kpRef,
                    matchesGMS,
                    frameMatches,
                    cv::Scalar::all(-1),
                    cv::Scalar::all(-1),
                    std::vector<char>(),
                    DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
      else
        drawMatches(frame, kp, frameRef, kpRef, matchesGMS, frameMatches);

      cv::String label = cv::format("ORB: %.2f ms", t_orb);
      cv::putText(frameMatches, label, cv::Point(20, 20), FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));
      label = cv::format("Matching: %.2f ms", t_match);
      cv::putText(frameMatches, label, cv::Point(20, 40), FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));
      label = cv::format("GMS matching: %.2f ms", tm.getTimeMilli());
      cv::putText(frameMatches, label, cv::Point(20, 60), FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));
      cv::putText(frameMatches,
              "Press r to reinitialize the reference image.",
              cv::Point(frameMatches.cols - 380, 20),
              FONT_HERSHEY_SIMPLEX,
              0.5,
              cv::Scalar(0, 0, 255));
      cv::putText(
          frameMatches, "Press esc to quit.", cv::Point(frameMatches.cols - 180, 40), FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));

      cv::imshow("Matches GMS", frameMatches);
      int c = cv::waitKey(30);
      if(c == 27)
        break;
      else if(c == 'r') {
        frame.copyTo(frameRef);
        orb->detectAndCompute(frameRef, cv::noArray(), kpRef, descRef);
      }
    }
  }

  return EXIT_SUCCESS;
}
