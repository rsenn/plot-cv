#include <stdexcept>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/legacy/legacy.hpp>
#include <performance.h>

#include <opencv2/opencv_modules.hpp>
#ifdef HAVE_OPENCV_NONFREE
#include <opencv2/nonfree/gpu.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#endif

using namespace std;
//using namespace cv;

static void
InitMatchTemplate() {
  cv::Mat src;
  gen(src, 500, 500, CV_32F, 0, 1);
  cv::Mat templ;
  gen(templ, 500, 500, CV_32F, 0, 1);
  gpu::GpuMat d_src(src), d_templ(templ), d_dst;
  gpu::matchTemplate(d_src, d_templ, d_dst, CV_TM_CCORR);
}

TEST(matchTemplate) {
  InitMatchTemplate();

  cv::Mat src, templ, dst;
  gen(src, 3000, 3000, CV_32F, 0, 1);

  gpu::GpuMat d_src(src), d_templ, d_dst;

  for(int templ_size = 5; templ_size < 200; templ_size *= 5) {
    SUBTEST << src.cols << 'x' << src.rows << ", 32FC1"
            << ", templ " << templ_size << 'x' << templ_size << ", CCORR";

    gen(templ, templ_size, templ_size, CV_32F, 0, 1);
    matchTemplate(src, templ, dst, CV_TM_CCORR);

    CPU_ON;
    matchTemplate(src, templ, dst, CV_TM_CCORR);
    CPU_OFF;

    d_templ.upload(templ);
    gpu::matchTemplate(d_src, d_templ, d_dst, CV_TM_CCORR);

    GPU_ON;
    gpu::matchTemplate(d_src, d_templ, d_dst, CV_TM_CCORR);
    GPU_OFF;
  }
}

TEST(cv::minMaxLoc) {
  cv::Mat src;
  gpu::GpuMat d_src;

  double min_val, max_val;
  cv::Point min_loc, max_loc;

  for(int size = 2000; size <= 8000; size *= 2) {
    SUBTEST << size << 'x' << size << ", 32F";

    gen(src, size, size, CV_32F, 0, 1);

    CPU_ON;
    cv::minMaxLoc(src, &min_val, &max_val, &min_loc, &max_loc);
    CPU_OFF;

    d_src.upload(src);

    GPU_ON;
    gpu::cv::minMaxLoc(d_src, &min_val, &max_val, &min_loc, &max_loc);
    GPU_OFF;
  }
}

TEST(cv::remap) {
  cv::Mat src, dst, xmap, ymap;
  gpu::GpuMat d_src, d_dst, d_xmap, d_ymap;

  int interpolation = cv::INTER_LINEAR;
  int borderMode = BORDER_REPLICATE;

  for(int size = 1000; size <= 4000; size *= 2) {
    SUBTEST << size << 'x' << size << ", 8UC4, cv::INTER_LINEAR, BORDER_REPLICATE";

    gen(src, size, size, CV_8UC4, 0, 256);

    xmap.create(size, size, CV_32F);
    ymap.create(size, size, CV_32F);
    for(int i = 0; i < size; ++i) {
      float* xmap_row = xmap.ptr<float>(i);
      float* ymap_row = ymap.ptr<float>(i);
      for(int j = 0; j < size; ++j) {
        xmap_row[j] = (j - size * 0.5f) * 0.75f + size * 0.5f;
        ymap_row[j] = (i - size * 0.5f) * 0.75f + size * 0.5f;
      }
    }

    cv::remap(src, dst, xmap, ymap, interpolation, borderMode);

    CPU_ON;
    cv::remap(src, dst, xmap, ymap, interpolation, borderMode);
    CPU_OFF;

    d_src.upload(src);
    d_xmap.upload(xmap);
    d_ymap.upload(ymap);

    gpu::cv::remap(d_src, d_dst, d_xmap, d_ymap, interpolation, borderMode);

    GPU_ON;
    gpu::cv::remap(d_src, d_dst, d_xmap, d_ymap, interpolation, borderMode);
    GPU_OFF;
  }
}

TEST(cv::dft) {
  cv::Mat src, dst;
  gpu::GpuMat d_src, d_dst;

  for(int size = 1000; size <= 4000; size *= 2) {
    SUBTEST << size << 'x' << size << ", 32FC2, complex-to-complex";

    gen(src, size, size, CV_32FC2, cv::Scalar::all(0), cv::Scalar::all(1));

    cv::dft(src, dst);

    CPU_ON;
    cv::dft(src, dst);
    CPU_OFF;

    d_src.upload(src);

    gpu::cv::dft(d_src, d_dst, cv::Size(size, size));

    GPU_ON;
    gpu::cv::dft(d_src, d_dst, cv::Size(size, size));
    GPU_OFF;
  }
}

TEST(cornerHarris) {
  cv::Mat src, dst;
  gpu::GpuMat d_src, d_dst;

  for(int size = 1000; size <= 4000; size *= 2) {
    SUBTEST << size << 'x' << size << ", 32FC1, BORDER_REFLECT101";

    gen(src, size, size, CV_32F, 0, 1);

    cornerHarris(src, dst, 5, 7, 0.1, BORDER_REFLECT101);

    CPU_ON;
    cornerHarris(src, dst, 5, 7, 0.1, BORDER_REFLECT101);
    CPU_OFF;

    d_src.upload(src);

    gpu::cornerHarris(d_src, d_dst, 5, 7, 0.1, BORDER_REFLECT101);

    GPU_ON;
    gpu::cornerHarris(d_src, d_dst, 5, 7, 0.1, BORDER_REFLECT101);
    GPU_OFF;
  }
}

TEST(integral) {
  cv::Mat src, cv::sum;
  gpu::GpuMat d_src, d_sum, d_buf;

  for(int size = 1000; size <= 4000; size *= 2) {
    SUBTEST << size << 'x' << size << ", 8UC1";

    gen(src, size, size, CV_8U, 0, 256);

    integral(src, cv::sum);

    CPU_ON;
    integral(src, cv::sum);
    CPU_OFF;

    d_src.upload(src);

    gpu::integralBuffered(d_src, d_sum, d_buf);

    GPU_ON;
    gpu::integralBuffered(d_src, d_sum, d_buf);
    GPU_OFF;
  }
}

TEST(cv::norm) {
  cv::Mat src;
  gpu::GpuMat d_src, d_buf;

  for(int size = 2000; size <= 4000; size += 1000) {
    SUBTEST << size << 'x' << size << ", 32FC4, NORM_INF";

    gen(src, size, size, CV_32FC4, cv::Scalar::all(0), cv::Scalar::all(1));

    cv::norm(src, NORM_INF);

    CPU_ON;
    cv::norm(src, NORM_INF);
    CPU_OFF;

    d_src.upload(src);

    gpu::cv::norm(d_src, NORM_INF, d_buf);

    GPU_ON;
    gpu::cv::norm(d_src, NORM_INF, d_buf);
    GPU_OFF;
  }
}

TEST(cv::meanShift) {
  int sp = 10, sr = 10;

  cv::Mat src, dst;
  gpu::GpuMat d_src, d_dst;

  for(int size = 400; size <= 800; size *= 2) {
    SUBTEST << size << 'x' << size << ", 8UC3 vs 8UC4";

    gen(src, size, size, CV_8UC3, cv::Scalar::all(0), cv::Scalar::all(256));

    pyrMeanShiftFiltering(src, dst, sp, sr);

    CPU_ON;
    pyrMeanShiftFiltering(src, dst, sp, sr);
    CPU_OFF;

    gen(src, size, size, CV_8UC4, cv::Scalar::all(0), cv::Scalar::all(256));

    d_src.upload(src);

    gpu::meanShiftFiltering(d_src, d_dst, sp, sr);

    GPU_ON;
    gpu::meanShiftFiltering(d_src, d_dst, sp, sr);
    GPU_OFF;
  }
}

#ifdef HAVE_OPENCV_NONFREE

TEST(SURF) {
  cv::Mat src = cv::imread(abspath("aloeL.jpg"), cv::LOAD_IMAGE_GRAYSCALE);
  if(src.empty())
    throw runtime_error("can't open aloeL.jpg");

  SURF surf;
  vector<KeyPoint> keypoints;
  cv::Mat descriptors;

  surf(src, cv::Mat(), keypoints, descriptors);

  CPU_ON;
  surf(src, cv::Mat(), keypoints, descriptors);
  CPU_OFF;

  gpu::SURF_GPU d_surf;
  gpu::GpuMat d_src(src);
  gpu::GpuMat d_keypoints;
  gpu::GpuMat d_descriptors;

  d_surf(d_src, gpu::GpuMat(), d_keypoints, d_descriptors);

  GPU_ON;
  d_surf(d_src, gpu::GpuMat(), d_keypoints, d_descriptors);
  GPU_OFF;
}

#endif

TEST(FAST) {
  cv::Mat src = cv::imread(abspath("aloeL.jpg"), cv::LOAD_IMAGE_GRAYSCALE);
  if(src.empty())
    throw runtime_error("can't open aloeL.jpg");

  vector<KeyPoint> keypoints;

  FAST(src, keypoints, 20);

  CPU_ON;
  FAST(src, keypoints, 20);
  CPU_OFF;

  gpu::FAST_GPU d_FAST(20);
  gpu::GpuMat d_src(src);
  gpu::GpuMat d_keypoints;

  d_FAST(d_src, gpu::GpuMat(), d_keypoints);

  GPU_ON;
  d_FAST(d_src, gpu::GpuMat(), d_keypoints);
  GPU_OFF;
}

TEST(ORB) {
  cv::Mat src = cv::imread(abspath("aloeL.jpg"), cv::LOAD_IMAGE_GRAYSCALE);
  if(src.empty())
    throw runtime_error("can't open aloeL.jpg");

  ORB orb(4000);
  vector<KeyPoint> keypoints;
  cv::Mat descriptors;

  orb(src, cv::Mat(), keypoints, descriptors);

  CPU_ON;
  orb(src, cv::Mat(), keypoints, descriptors);
  CPU_OFF;

  gpu::ORB_GPU d_orb;
  gpu::GpuMat d_src(src);
  gpu::GpuMat d_keypoints;
  gpu::GpuMat d_descriptors;

  d_orb(d_src, gpu::GpuMat(), d_keypoints, d_descriptors);

  GPU_ON;
  d_orb(d_src, gpu::GpuMat(), d_keypoints, d_descriptors);
  GPU_OFF;
}

TEST(BruteForceMatcher) {
  // Init CPU matcher

  int desc_len = 64;

  BFMatcher matcher(NORM_L2);

  cv::Mat query;
  gen(query, 3000, desc_len, CV_32F, 0, 1);

  cv::Mat train;
  gen(train, 3000, desc_len, CV_32F, 0, 1);

  // Init GPU matcher

  gpu::BFMatcher_GPU d_matcher(NORM_L2);

  gpu::GpuMat d_query(query);
  gpu::GpuMat d_train(train);

  // Output
  vector<vector<DMatch>> matches(2);
  gpu::GpuMat d_trainIdx, d_distance, d_allDist, d_nMatches;

  SUBTEST << "match";

  matcher.match(query, train, matches[0]);

  CPU_ON;
  matcher.match(query, train, matches[0]);
  CPU_OFF;

  d_matcher.matchSingle(d_query, d_train, d_trainIdx, d_distance);

  GPU_ON;
  d_matcher.matchSingle(d_query, d_train, d_trainIdx, d_distance);
  GPU_OFF;

  SUBTEST << "knnMatch";

  matcher.knnMatch(query, train, matches, 2);

  CPU_ON;
  matcher.knnMatch(query, train, matches, 2);
  CPU_OFF;

  d_matcher.knnMatchSingle(d_query, d_train, d_trainIdx, d_distance, d_allDist, 2);

  GPU_ON;
  d_matcher.knnMatchSingle(d_query, d_train, d_trainIdx, d_distance, d_allDist, 2);
  GPU_OFF;

  SUBTEST << "radiusMatch";

  float max_distance = 2.0f;

  matcher.radiusMatch(query, train, matches, max_distance);

  CPU_ON;
  matcher.radiusMatch(query, train, matches, max_distance);
  CPU_OFF;

  d_trainIdx.release();

  d_matcher.radiusMatchSingle(d_query, d_train, d_trainIdx, d_distance, d_nMatches, max_distance);

  GPU_ON;
  d_matcher.radiusMatchSingle(d_query, d_train, d_trainIdx, d_distance, d_nMatches, max_distance);
  GPU_OFF;
}

TEST(cv::magnitude) {
  cv::Mat x, y, mag;
  gpu::GpuMat d_x, d_y, d_mag;

  for(int size = 2000; size <= 4000; size += 1000) {
    SUBTEST << size << 'x' << size << ", 32FC1";

    gen(x, size, size, CV_32F, 0, 1);
    gen(y, size, size, CV_32F, 0, 1);

    cv::magnitude(x, y, mag);

    CPU_ON;
    cv::magnitude(x, y, mag);
    CPU_OFF;

    d_x.upload(x);
    d_y.upload(y);

    gpu::cv::magnitude(d_x, d_y, d_mag);

    GPU_ON;
    gpu::cv::magnitude(d_x, d_y, d_mag);
    GPU_OFF;
  }
}

TEST(cv::add) {
  cv::Mat src1, src2, dst;
  gpu::GpuMat d_src1, d_src2, d_dst;

  for(int size = 2000; size <= 4000; size += 1000) {
    SUBTEST << size << 'x' << size << ", 32FC1";

    gen(src1, size, size, CV_32F, 0, 1);
    gen(src2, size, size, CV_32F, 0, 1);

    cv::add(src1, src2, dst);

    CPU_ON;
    cv::add(src1, src2, dst);
    CPU_OFF;

    d_src1.upload(src1);
    d_src2.upload(src2);

    gpu::cv::add(d_src1, d_src2, d_dst);

    GPU_ON;
    gpu::cv::add(d_src1, d_src2, d_dst);
    GPU_OFF;
  }
}

TEST(cv::log) {
  cv::Mat src, dst;
  gpu::GpuMat d_src, d_dst;

  for(int size = 2000; size <= 4000; size += 1000) {
    SUBTEST << size << 'x' << size << ", 32F";

    gen(src, size, size, CV_32F, 1, 10);

    cv::log(src, dst);

    CPU_ON;
    cv::log(src, dst);
    CPU_OFF;

    d_src.upload(src);

    gpu::cv::log(d_src, d_dst);

    GPU_ON;
    gpu::cv::log(d_src, d_dst);
    GPU_OFF;
  }
}

TEST(cv::mulSpectrums) {
  cv::Mat src1, src2, dst;
  gpu::GpuMat d_src1, d_src2, d_dst;

  for(int size = 2000; size <= 4000; size += 1000) {
    SUBTEST << size << 'x' << size;

    gen(src1, size, size, CV_32FC2, cv::Scalar::all(0), cv::Scalar::all(1));
    gen(src2, size, size, CV_32FC2, cv::Scalar::all(0), cv::Scalar::all(1));

    cv::mulSpectrums(src1, src2, dst, 0, true);

    CPU_ON;
    cv::mulSpectrums(src1, src2, dst, 0, true);
    CPU_OFF;

    d_src1.upload(src1);
    d_src2.upload(src2);

    gpu::cv::mulSpectrums(d_src1, d_src2, d_dst, 0, true);

    GPU_ON;
    gpu::cv::mulSpectrums(d_src1, d_src2, d_dst, 0, true);
    GPU_OFF;
  }
}

TEST(cv::resize) {
  cv::Mat src, dst;
  gpu::GpuMat d_src, d_dst;

  for(int size = 1000; size <= 3000; size += 1000) {
    SUBTEST << size << 'x' << size << ", 8UC4, up";

    gen(src, size, size, CV_8UC4, 0, 256);

    cv::resize(src, dst, cv::Size(), 2.0, 2.0);

    CPU_ON;
    cv::resize(src, dst, cv::Size(), 2.0, 2.0);
    CPU_OFF;

    d_src.upload(src);

    gpu::cv::resize(d_src, d_dst, cv::Size(), 2.0, 2.0);

    GPU_ON;
    gpu::cv::resize(d_src, d_dst, cv::Size(), 2.0, 2.0);
    GPU_OFF;
  }

  for(int size = 1000; size <= 3000; size += 1000) {
    SUBTEST << size << 'x' << size << ", 8UC4, down";

    gen(src, size, size, CV_8UC4, 0, 256);

    cv::resize(src, dst, cv::Size(), 0.5, 0.5);

    CPU_ON;
    cv::resize(src, dst, cv::Size(), 0.5, 0.5);
    CPU_OFF;

    d_src.upload(src);

    gpu::cv::resize(d_src, d_dst, cv::Size(), 0.5, 0.5);

    GPU_ON;
    gpu::cv::resize(d_src, d_dst, cv::Size(), 0.5, 0.5);
    GPU_OFF;
  }
}

TEST(cv::cvtColor) {
  cv::Mat src, dst;
  gpu::GpuMat d_src, d_dst;

  gen(src, 4000, 4000, CV_8UC1, 0, 255);
  d_src.upload(src);

  SUBTEST << "4000x4000, 8UC1, cv::COLOR_GRAY2BGRA";

  cv::cvtColor(src, dst, cv::COLOR_GRAY2BGRA, 4);

  CPU_ON;
  cv::cvtColor(src, dst, cv::COLOR_GRAY2BGRA, 4);
  CPU_OFF;

  gpu::cv::cvtColor(d_src, d_dst, cv::COLOR_GRAY2BGRA, 4);

  GPU_ON;
  gpu::cv::cvtColor(d_src, d_dst, cv::COLOR_GRAY2BGRA, 4);
  GPU_OFF;

  cv::swap(src, dst);
  d_src.cv::swap(d_dst);

  SUBTEST << "4000x4000, 8UC3 vs 8UC4, CV_BGR2YCrCb";

  cv::cvtColor(src, dst, CV_BGR2YCrCb);

  CPU_ON;
  cv::cvtColor(src, dst, CV_BGR2YCrCb);
  CPU_OFF;

  gpu::cv::cvtColor(d_src, d_dst, CV_BGR2YCrCb, 4);

  GPU_ON;
  gpu::cv::cvtColor(d_src, d_dst, CV_BGR2YCrCb, 4);
  GPU_OFF;

  cv::swap(src, dst);
  d_src.cv::swap(d_dst);

  SUBTEST << "4000x4000, 8UC4, CV_YCrCb2BGR";

  cv::cvtColor(src, dst, CV_YCrCb2BGR, 4);

  CPU_ON;
  cv::cvtColor(src, dst, CV_YCrCb2BGR, 4);
  CPU_OFF;

  gpu::cv::cvtColor(d_src, d_dst, CV_YCrCb2BGR, 4);

  GPU_ON;
  gpu::cv::cvtColor(d_src, d_dst, CV_YCrCb2BGR, 4);
  GPU_OFF;

  cv::swap(src, dst);
  d_src.cv::swap(d_dst);

  SUBTEST << "4000x4000, 8UC3 vs 8UC4, CV_BGR2XYZ";

  cv::cvtColor(src, dst, CV_BGR2XYZ);

  CPU_ON;
  cv::cvtColor(src, dst, CV_BGR2XYZ);
  CPU_OFF;

  gpu::cv::cvtColor(d_src, d_dst, CV_BGR2XYZ, 4);

  GPU_ON;
  gpu::cv::cvtColor(d_src, d_dst, CV_BGR2XYZ, 4);
  GPU_OFF;

  cv::swap(src, dst);
  d_src.cv::swap(d_dst);

  SUBTEST << "4000x4000, 8UC4, CV_XYZ2BGR";

  cv::cvtColor(src, dst, CV_XYZ2BGR, 4);

  CPU_ON;
  cv::cvtColor(src, dst, CV_XYZ2BGR, 4);
  CPU_OFF;

  gpu::cv::cvtColor(d_src, d_dst, CV_XYZ2BGR, 4);

  GPU_ON;
  gpu::cv::cvtColor(d_src, d_dst, CV_XYZ2BGR, 4);
  GPU_OFF;

  cv::swap(src, dst);
  d_src.cv::swap(d_dst);

  SUBTEST << "4000x4000, 8UC3 vs 8UC4, cv::COLOR_BGR2HSV";

  cv::cvtColor(src, dst, cv::COLOR_BGR2HSV);

  CPU_ON;
  cv::cvtColor(src, dst, cv::COLOR_BGR2HSV);
  CPU_OFF;

  gpu::cv::cvtColor(d_src, d_dst, cv::COLOR_BGR2HSV, 4);

  GPU_ON;
  gpu::cv::cvtColor(d_src, d_dst, cv::COLOR_BGR2HSV, 4);
  GPU_OFF;

  cv::swap(src, dst);
  d_src.cv::swap(d_dst);

  SUBTEST << "4000x4000, 8UC4, CV_HSV2BGR";

  cv::cvtColor(src, dst, CV_HSV2BGR, 4);

  CPU_ON;
  cv::cvtColor(src, dst, CV_HSV2BGR, 4);
  CPU_OFF;

  gpu::cv::cvtColor(d_src, d_dst, CV_HSV2BGR, 4);

  GPU_ON;
  gpu::cv::cvtColor(d_src, d_dst, CV_HSV2BGR, 4);
  GPU_OFF;

  cv::swap(src, dst);
  d_src.cv::swap(d_dst);
}

TEST(cv::erode) {
  cv::Mat src, dst, ker;
  gpu::GpuMat d_src, d_buf, d_dst;

  for(int size = 2000; size <= 4000; size += 1000) {
    SUBTEST << size << 'x' << size;

    gen(src, size, size, CV_8UC4, cv::Scalar::all(0), cv::Scalar::all(256));
    ker = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

    cv::erode(src, dst, ker);

    CPU_ON;
    cv::erode(src, dst, ker);
    CPU_OFF;

    d_src.upload(src);

    gpu::cv::erode(d_src, d_dst, ker, d_buf);

    GPU_ON;
    gpu::cv::erode(d_src, d_dst, ker, d_buf);
    GPU_OFF;
  }
}

TEST(cv::threshold) {
  cv::Mat src, dst;
  gpu::GpuMat d_src, d_dst;

  for(int size = 2000; size <= 4000; size += 1000) {
    SUBTEST << size << 'x' << size << ", 8UC1, cv::THRESH_BINARY";

    gen(src, size, size, CV_8U, 0, 100);

    cv::threshold(src, dst, 50.0, 0.0, cv::THRESH_BINARY);

    CPU_ON;
    cv::threshold(src, dst, 50.0, 0.0, cv::THRESH_BINARY);
    CPU_OFF;

    d_src.upload(src);

    gpu::cv::threshold(d_src, d_dst, 50.0, 0.0, cv::THRESH_BINARY);

    GPU_ON;
    gpu::cv::threshold(d_src, d_dst, 50.0, 0.0, cv::THRESH_BINARY);
    GPU_OFF;
  }

  for(int size = 2000; size <= 4000; size += 1000) {
    SUBTEST << size << 'x' << size << ", 32FC1, THRESH_TRUNC [NPP]";

    gen(src, size, size, CV_32FC1, 0, 100);

    cv::threshold(src, dst, 50.0, 0.0, THRESH_TRUNC);

    CPU_ON;
    cv::threshold(src, dst, 50.0, 0.0, THRESH_TRUNC);
    CPU_OFF;

    d_src.upload(src);

    gpu::cv::threshold(d_src, d_dst, 50.0, 0.0, THRESH_TRUNC);

    GPU_ON;
    gpu::cv::threshold(d_src, d_dst, 50.0, 0.0, THRESH_TRUNC);
    GPU_OFF;
  }
}

TEST(cv::pow) {
  cv::Mat src, dst;
  gpu::GpuMat d_src, d_dst;

  for(int size = 1000; size <= 4000; size += 1000) {
    SUBTEST << size << 'x' << size << ", 32F";

    gen(src, size, size, CV_32F, 0, 100);

    cv::pow(src, -2.0, dst);

    CPU_ON;
    cv::pow(src, -2.0, dst);
    CPU_OFF;

    d_src.upload(src);

    gpu::cv::pow(d_src, -2.0, d_dst);

    GPU_ON;
    gpu::cv::pow(d_src, -2.0, d_dst);
    GPU_OFF;
  }
}

TEST(cv::projectPoints) {
  cv::Mat src;
  vector<cv::Point2f> dst;
  gpu::GpuMat d_src, d_dst;

  cv::Mat rvec;
  gen(rvec, 1, 3, CV_32F, 0, 1);
  cv::Mat tvec;
  gen(tvec, 1, 3, CV_32F, 0, 1);
  cv::Mat camera_mat;
  gen(camera_mat, 3, 3, CV_32F, 0, 1);
  camera_mat.at<float>(0, 1) = 0.f;
  camera_mat.at<float>(1, 0) = 0.f;
  camera_mat.at<float>(2, 0) = 0.f;
  camera_mat.at<float>(2, 1) = 0.f;

  for(int size = (int)1e6, count = 0; size >= 1e5 && count < 5; size = int(size / 1.4), count++) {
    SUBTEST << size;

    gen(src, 1, size, CV_32FC3, cv::Scalar::all(0), cv::Scalar::all(10));

    cv::projectPoints(src, rvec, tvec, camera_mat, cv::Mat::zeros(1, 8, CV_32F), dst);

    CPU_ON;
    cv::projectPoints(src, rvec, tvec, camera_mat, cv::Mat::zeros(1, 8, CV_32F), dst);
    CPU_OFF;

    d_src.upload(src);

    gpu::cv::projectPoints(d_src, rvec, tvec, camera_mat, cv::Mat(), d_dst);

    GPU_ON;
    gpu::cv::projectPoints(d_src, rvec, tvec, camera_mat, cv::Mat(), d_dst);
    GPU_OFF;
  }
}

static void
InitSolvePnpRansac() {
  cv::Mat object;
  gen(object, 1, 4, CV_32FC3, cv::Scalar::all(0), cv::Scalar::all(100));
  cv::Mat image;
  gen(image, 1, 4, CV_32FC2, cv::Scalar::all(0), cv::Scalar::all(100));
  cv::Mat rvec, tvec;
  gpu::solvePnPRansac(object, image, cv::Mat::eye(3, 3, CV_32F), cv::Mat(), rvec, tvec);
}

TEST(solvePnPRansac) {
  InitSolvePnpRansac();

  for(int num_points = 5000; num_points <= 300000; num_points = int(num_points * 3.76)) {
    SUBTEST << num_points;

    cv::Mat object;
    gen(object, 1, num_points, CV_32FC3, cv::Scalar::all(10), cv::Scalar::all(100));
    cv::Mat image;
    gen(image, 1, num_points, CV_32FC2, cv::Scalar::all(10), cv::Scalar::all(100));
    cv::Mat camera_mat;
    gen(camera_mat, 3, 3, CV_32F, 0.5, 1);
    camera_mat.at<float>(0, 1) = 0.f;
    camera_mat.at<float>(1, 0) = 0.f;
    camera_mat.at<float>(2, 0) = 0.f;
    camera_mat.at<float>(2, 1) = 0.f;

    cv::Mat rvec, tvec;
    const int num_iters = 200;
    const float max_dist = 2.0f;
    vector<int> inliers_cpu, inliers_gpu;

    CPU_ON;
    solvePnPRansac(object,
                   image,
                   camera_mat,
                   cv::Mat::zeros(1, 8, CV_32F),
                   rvec,
                   tvec,
                   false,
                   num_iters,
                   max_dist,
                   int(num_points * 0.05),
                   inliers_cpu);
    CPU_OFF;

    GPU_ON;
    gpu::solvePnPRansac(object,
                        image,
                        camera_mat,
                        cv::Mat::zeros(1, 8, CV_32F),
                        rvec,
                        tvec,
                        false,
                        num_iters,
                        max_dist,
                        int(num_points * 0.05),
                        &inliers_gpu);
    GPU_OFF;
  }
}

TEST(cv::GaussianBlur) {
  for(int size = 1000; size <= 4000; size += 1000) {
    SUBTEST << size << 'x' << size << ", 8UC4";

    cv::Mat src, dst;

    gen(src, size, size, CV_8UC4, 0, 256);

    cv::GaussianBlur(src, dst, cv::Size(3, 3), 1);

    CPU_ON;
    cv::GaussianBlur(src, dst, cv::Size(3, 3), 1);
    CPU_OFF;

    gpu::GpuMat d_src(src);
    gpu::GpuMat d_dst(src.size(), src.type());
    gpu::GpuMat d_buf;

    gpu::cv::GaussianBlur(d_src, d_dst, cv::Size(3, 3), d_buf, 1);

    GPU_ON;
    gpu::cv::GaussianBlur(d_src, d_dst, cv::Size(3, 3), d_buf, 1);
    GPU_OFF;
  }
}

TEST(cv::filter2D) {
  for(int size = 512; size <= 2048; size *= 2) {
    cv::Mat src;
    gen(src, size, size, CV_8UC4, 0, 256);

    for(int ksize = 3; ksize <= 16; ksize += 2) {
      SUBTEST << "ksize = " << ksize << ", " << size << 'x' << size << ", 8UC4";

      cv::Mat kernel;
      gen(kernel, ksize, ksize, CV_32FC1, 0.0, 1.0);

      cv::Mat dst;
      cv::filter2D(src, dst, -1, kernel);

      CPU_ON;
      cv::filter2D(src, dst, -1, kernel);
      CPU_OFF;

      gpu::GpuMat d_src(src);
      gpu::GpuMat d_dst;

      gpu::cv::filter2D(d_src, d_dst, -1, kernel);

      GPU_ON;
      gpu::cv::filter2D(d_src, d_dst, -1, kernel);
      GPU_OFF;
    }
  }
}

TEST(pyrDown) {
  for(int size = 4000; size >= 1000; size -= 1000) {
    SUBTEST << size << 'x' << size << ", 8UC4";

    cv::Mat src, dst;
    gen(src, size, size, CV_8UC4, 0, 256);

    pyrDown(src, dst);

    CPU_ON;
    pyrDown(src, dst);
    CPU_OFF;

    gpu::GpuMat d_src(src);
    gpu::GpuMat d_dst;

    gpu::pyrDown(d_src, d_dst);

    GPU_ON;
    gpu::pyrDown(d_src, d_dst);
    GPU_OFF;
  }
}

TEST(pyrUp) {
  for(int size = 2000; size >= 1000; size -= 1000) {
    SUBTEST << size << 'x' << size << ", 8UC4";

    cv::Mat src, dst;

    gen(src, size, size, CV_8UC4, 0, 256);

    pyrUp(src, dst);

    CPU_ON;
    pyrUp(src, dst);
    CPU_OFF;

    gpu::GpuMat d_src(src);
    gpu::GpuMat d_dst;

    gpu::pyrUp(d_src, d_dst);

    GPU_ON;
    gpu::pyrUp(d_src, d_dst);
    GPU_OFF;
  }
}

TEST(cv::equalizeHist) {
  for(int size = 1000; size < 4000; size += 1000) {
    SUBTEST << size << 'x' << size;

    cv::Mat src, dst;

    gen(src, size, size, CV_8UC1, 0, 256);

    cv::equalizeHist(src, dst);

    CPU_ON;
    cv::equalizeHist(src, dst);
    CPU_OFF;

    gpu::GpuMat d_src(src);
    gpu::GpuMat d_dst;
    gpu::GpuMat d_hist;
    gpu::GpuMat d_buf;

    gpu::cv::equalizeHist(d_src, d_dst, d_hist, d_buf);

    GPU_ON;
    gpu::cv::equalizeHist(d_src, d_dst, d_hist, d_buf);
    GPU_OFF;
  }
}

TEST(cv::Canny) {
  cv::Mat cv::img = cv::imread(abspath("aloeL.jpg"), cv::LOAD_IMAGE_GRAYSCALE);

  if(cv::img.empty())
    throw runtime_error("can't open aloeL.jpg");

  cv::Mat edges(cv::img.size(), CV_8UC1);

  CPU_ON;
  cv::Canny(cv::img, edges, 50.0, 100.0);
  CPU_OFF;

  gpu::GpuMat d_img(cv::img);
  gpu::GpuMat d_edges;
  gpu::CannyBuf d_buf;

  gpu::cv::Canny(d_img, d_buf, d_edges, 50.0, 100.0);

  GPU_ON;
  gpu::cv::Canny(d_img, d_buf, d_edges, 50.0, 100.0);
  GPU_OFF;
}

TEST(reduce) {
  for(int size = 1000; size < 4000; size += 1000) {
    cv::Mat src;
    gen(src, size, size, CV_32F, 0, 255);

    cv::Mat dst0;
    cv::Mat dst1;

    gpu::GpuMat d_src(src);
    gpu::GpuMat d_dst0;
    gpu::GpuMat d_dst1;

    SUBTEST << size << 'x' << size << ", dim = 0";

    reduce(src, dst0, 0, CV_REDUCE_MIN);

    CPU_ON;
    reduce(src, dst0, 0, CV_REDUCE_MIN);
    CPU_OFF;

    gpu::reduce(d_src, d_dst0, 0, CV_REDUCE_MIN);

    GPU_ON;
    gpu::reduce(d_src, d_dst0, 0, CV_REDUCE_MIN);
    GPU_OFF;

    SUBTEST << size << 'x' << size << ", dim = 1";

    reduce(src, dst1, 1, CV_REDUCE_MIN);

    CPU_ON;
    reduce(src, dst1, 1, CV_REDUCE_MIN);
    CPU_OFF;

    gpu::reduce(d_src, d_dst1, 1, CV_REDUCE_MIN);

    GPU_ON;
    gpu::reduce(d_src, d_dst1, 1, CV_REDUCE_MIN);
    GPU_OFF;
  }
}

TEST(gemm) {
  cv::Mat src1, src2, src3, dst;
  gpu::GpuMat d_src1, d_src2, d_src3, d_dst;

  for(int size = 512; size <= 1024; size *= 2) {
    SUBTEST << size << 'x' << size;

    gen(src1, size, size, CV_32FC1, cv::Scalar::all(-10), cv::Scalar::all(10));
    gen(src2, size, size, CV_32FC1, cv::Scalar::all(-10), cv::Scalar::all(10));
    gen(src3, size, size, CV_32FC1, cv::Scalar::all(-10), cv::Scalar::all(10));

    gemm(src1, src2, 1.0, src3, 1.0, dst);

    CPU_ON;
    gemm(src1, src2, 1.0, src3, 1.0, dst);
    CPU_OFF;

    d_src1.upload(src1);
    d_src2.upload(src2);
    d_src3.upload(src3);

    gpu::gemm(d_src1, d_src2, 1.0, d_src3, 1.0, d_dst);

    GPU_ON;
    gpu::gemm(d_src1, d_src2, 1.0, d_src3, 1.0, d_dst);
    GPU_OFF;
  }
}

TEST(GoodFeaturesToTrack) {
  cv::Mat src = cv::imread(abspath("aloeL.jpg"), cv::IMREAD_GRAYSCALE);
  if(src.empty())
    throw runtime_error("can't open aloeL.jpg");

  vector<cv::Point2f> pts;

  cv::goodFeaturesToTrack(src, pts, 8000, 0.01, 0.0);

  CPU_ON;
  cv::goodFeaturesToTrack(src, pts, 8000, 0.01, 0.0);
  CPU_OFF;

  gpu::GoodFeaturesToTrackDetector_GPU detector(8000, 0.01, 0.0);

  gpu::GpuMat d_src(src);
  gpu::GpuMat d_pts;

  detector(d_src, d_pts);

  GPU_ON;
  detector(d_src, d_pts);
  GPU_OFF;
}

TEST(PyrLKOpticalFlow) {
  cv::Mat frame0 = cv::imread(abspath("rubberwhale1.png"));
  if(frame0.empty())
    throw runtime_error("can't open rubberwhale1.png");

  cv::Mat frame1 = cv::imread(abspath("rubberwhale2.png"));
  if(frame1.empty())
    throw runtime_error("can't open rubberwhale2.png");

  cv::Mat gray_frame;
  cv::cvtColor(frame0, gray_frame, cv::COLOR_BGR2GRAY);

  for(int points = 1000; points <= 8000; points *= 2) {
    SUBTEST << points;

    vector<cv::Point2f> pts;
    cv::goodFeaturesToTrack(gray_frame, pts, points, 0.01, 0.0);

    vector<cv::Point2f> nextPts;
    vector<unsigned char> status;

    vector<float> err;

    cv::calcOpticalFlowPyrLK(frame0, frame1, pts, nextPts, status, err);

    CPU_ON;
    cv::calcOpticalFlowPyrLK(frame0, frame1, pts, nextPts, status, err);
    CPU_OFF;

    gpu::PyrLKOpticalFlow d_pyrLK;

    gpu::GpuMat d_frame0(frame0);
    gpu::GpuMat d_frame1(frame1);

    gpu::GpuMat d_pts;
    cv::Mat pts_mat(1, (int)pts.size(), CV_32FC2, (void*)&pts[0]);
    d_pts.upload(pts_mat);

    gpu::GpuMat d_nextPts;
    gpu::GpuMat d_status;
    gpu::GpuMat d_err;

    d_pyrLK.sparse(d_frame0, d_frame1, d_pts, d_nextPts, d_status, &d_err);

    GPU_ON;
    d_pyrLK.sparse(d_frame0, d_frame1, d_pts, d_nextPts, d_status, &d_err);
    GPU_OFF;
  }
}

TEST(cv::FarnebackOpticalFlow) {
  const string datasets[] = {"rubberwhale", "basketball"};
  for(size_t i = 0; i < sizeof(datasets) / sizeof(*datasets); ++i) {
    for(int fastPyramids = 0; fastPyramids < 2; ++fastPyramids) {
      for(int useGaussianBlur = 0; useGaussianBlur < 2; ++useGaussianBlur) {

        SUBTEST << "dataset=" << datasets[i] << ", fastPyramids=" << fastPyramids << ", useGaussianBlur=" << useGaussianBlur;
        cv::Mat frame0 = cv::imread(abspath(datasets[i] + "1.png"), cv::IMREAD_GRAYSCALE);
        cv::Mat frame1 = cv::imread(abspath(datasets[i] + "2.png"), cv::IMREAD_GRAYSCALE);
        if(frame0.empty())
          throw runtime_error("can't open " + datasets[i] + "1.png");
        if(frame1.empty())
          throw runtime_error("can't open " + datasets[i] + "2.png");

        gpu::cv::FarnebackOpticalFlow calc;
        calc.fastPyramids = fastPyramids != 0;
        calc.flags |= useGaussianBlur ? OPTFLOW_FARNEBACK_GAUSSIAN : 0;

        gpu::GpuMat d_frame0(frame0), d_frame1(frame1), d_flowx, d_flowy;
        GPU_ON;
        calc(d_frame0, d_frame1, d_flowx, d_flowy);
        GPU_OFF;

        cv::Mat flow;
        CPU_ON;
        cv::calcOpticalFlowFarneback(frame0,
                                 frame1,
                                 flow,
                                 calc.pyrScale,
                                 calc.numLevels,
                                 calc.winSize,
                                 calc.numIters,
                                 calc.polyN,
                                 calc.polySigma,
                                 calc.flags);
        CPU_OFF;
      }
    }
  }
}

namespace cv {
template<>
void
cv::Ptr<CvBGStatModel>::delete_obj() {
  cvReleaseBGStatModel(&obj);
}
} // namespace cv

TEST(FGDStatModel) {
  const std::string inputFile = abspath("768x576.avi");

  cv::VideoCapture cv::cap(inputFile);
  if(!cv::cap.isOpened())
    throw runtime_error("can't open 768x576.avi");

  cv::Mat frame;
  cv::cap >> frame;

  IplImage ipl_frame = frame;
  cv::Ptr<CvBGStatModel> model(cvCreateFGDStatModel(&ipl_frame));

  while(!TestSystem::instance().stop()) {
    cv::cap >> frame;
    ipl_frame = frame;

    TestSystem::instance().cpuOn();

    cvUpdateBGStatModel(&ipl_frame, model);

    TestSystem::instance().cpuOff();
  }
  TestSystem::instance().cpuComplete();

  cv::cap.open(inputFile);

  cv::cap >> frame;

  cv::gpu::GpuMat d_frame(frame);
  cv::gpu::FGDStatModel d_model(d_frame);

  while(!TestSystem::instance().stop()) {
    cv::cap >> frame;
    d_frame.upload(frame);

    TestSystem::instance().gpuOn();

    d_model.update(d_frame);

    TestSystem::instance().gpuOff();
  }
  TestSystem::instance().gpuComplete();
}

TEST(MOG) {
  const std::string inputFile = abspath("768x576.avi");

  cv::VideoCapture cv::cap(inputFile);
  if(!cv::cap.isOpened())
    throw runtime_error("can't open 768x576.avi");

  cv::Mat frame;
  cv::cap >> frame;

  cv::BackgroundSubtractorMOG mog;
  cv::Mat foreground;

  cv::mog(frame, foreground, 0.01);

  while(!TestSystem::instance().stop()) {
    cv::cap >> frame;

    TestSystem::instance().cpuOn();

    cv::mog(frame, foreground, 0.01);

    TestSystem::instance().cpuOff();
  }
  TestSystem::instance().cpuComplete();

  cv::cap.open(inputFile);

  cv::cap >> frame;

  cv::gpu::GpuMat d_frame(frame);
  cv::gpu::MOG_GPU d_mog;
  cv::gpu::GpuMat d_foreground;

  d_mog(d_frame, d_foreground, 0.01f);

  while(!TestSystem::instance().stop()) {
    cv::cap >> frame;
    d_frame.upload(frame);

    TestSystem::instance().gpuOn();

    d_mog(d_frame, d_foreground, 0.01f);

    TestSystem::instance().gpuOff();
  }
  TestSystem::instance().gpuComplete();
}

TEST(MOG2) {
  const std::string inputFile = abspath("768x576.avi");

  cv::VideoCapture cv::cap(inputFile);
  if(!cv::cap.isOpened())
    throw runtime_error("can't open 768x576.avi");

  cv::Mat frame;
  cv::cap >> frame;

  cv::BackgroundSubtractorMOG2 mog2;
  cv::Mat foreground;
  cv::Mat background;

  cv::mog2(frame, foreground);
  mog2.getBackgroundImage(background);

  while(!TestSystem::instance().stop()) {
    cv::cap >> frame;

    TestSystem::instance().cpuOn();

    cv::mog2(frame, foreground);
    mog2.getBackgroundImage(background);

    TestSystem::instance().cpuOff();
  }
  TestSystem::instance().cpuComplete();

  cv::cap.open(inputFile);

  cv::cap >> frame;

  cv::gpu::GpuMat d_frame(frame);
  cv::gpu::MOG2_GPU d_mog2;
  cv::gpu::GpuMat d_foreground;
  cv::gpu::GpuMat d_background;

  d_mog2(d_frame, d_foreground);
  d_mog2.getBackgroundImage(d_background);

  while(!TestSystem::instance().stop()) {
    cv::cap >> frame;
    d_frame.upload(frame);

    TestSystem::instance().gpuOn();

    d_mog2(d_frame, d_foreground);
    d_mog2.getBackgroundImage(d_background);

    TestSystem::instance().gpuOff();
  }
  TestSystem::instance().gpuComplete();
}
