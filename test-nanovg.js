import { DrawCircle, DrawImage, GLFW, Mat2Image } from './draw-utils.js';
import { HSLA } from './lib/color.js';
import Console from 'console';
import { className } from 'util';
import * as glfw from 'glfw'
import { poll, wait, getProcAddress, getTime, setTime, getTimerValue, getTimerFrequency, getPlatform, platformSupported, getKeyName, getKeyScancode, terminate, postEmptyEvent, extensionSupported, createCursor, createStandardCursor, destroyCursor,  FOCUSED, ICONIFIED, RESIZABLE, VISIBLE, DECORATED, AUTO_ICONIFY, FLOATING, MAXIMIZED, RED_BITS, GREEN_BITS, BLUE_BITS, ALPHA_BITS, DEPTH_BITS, STENCIL_BITS, ACCUM_RED_BITS, ACCUM_GREEN_BITS, ACCUM_BLUE_BITS, ACCUM_ALPHA_BITS, AUX_BUFFERS, STEREO, SAMPLES, SRGB_CAPABLE, REFRESH_RATE, DOUBLEBUFFER, CLIENT_API, CONTEXT_VERSION_MAJOR, CONTEXT_VERSION_MINOR, CONTEXT_REVISION, CONTEXT_ROBUSTNESS, OPENGL_FORWARD_COMPAT, OPENGL_DEBUG_CONTEXT, OPENGL_PROFILE, CONTEXT_RELEASE_BEHAVIOR, CONTEXT_NO_ERROR, CONTEXT_CREATION_API, OPENGL_CORE_PROFILE, VERSION_MAJOR, VERSION_MINOR, VERSION_REVISION, ANY_PLATFORM, PLATFORM_WIN32, PLATFORM_COCOA, PLATFORM_WAYLAND, PLATFORM_X11, PLATFORM_NULL, Position, Size, GammaRamp, Image, Monitor, Scale, VideoMode, Window, WorkArea, version, rawMouseMotionSupported } from 'glfw'
import * as ImGui from 'imgui';
import * as nvg from 'nanovg';
import { imdecode, imencode, imread, imwrite, split, normalize, merge, mixChannels, addWeighted, getTickCount, getTickFrequency, getCPUTickCount, bitwise_and, bitwise_or, bitwise_xor, bitwise_not, countNonZero, findNonZero, hconcat, vconcat, CV_VERSION_STATUS, CV_PI, CV_2PI, CV_LOG2, absdiff, add, compare, divide, max, min, multiply, solve, subtract, convertFp16, convertScaleAbs, copyMakeBorder, copyTo, dct, dft, exp, extractChannel, flip, idct, idft, invert, log, mulTransposed, perspectiveTransform, reduce, rotate, sort, sortIdx, sqrt, transform, transpose, calcCovarMatrix, cartToPolar, checkRange, determinant, eigen, eigenNonSymmetric, inRange, insertChannel, LUT, magnitude, Mahalanobis, mean, meanStdDev, minMaxIdx, minMaxLoc, mulSpectrums, norm, patchNaNs, phase, polarToCart, pow, randn, randShuffle, randu, repeat, scaleAdd, setIdentity, solveCubic, solvePoly, sum, trace, CV_RGB, CV_VERSION_MAJOR, CV_VERSION_MINOR, CV_VERSION_REVISION, CV_CMP_EQ, CV_CMP_GT, CV_CMP_GE, CV_CMP_LT, CV_CMP_LE, CV_CMP_NE, CV_8U, CV_8S, CV_16U, CV_16S, CV_32S, CV_32F, CV_64F, CV_8UC1, CV_8UC2, CV_8UC3, CV_8UC4, CV_8SC1, CV_8SC2, CV_8SC3, CV_8SC4, CV_16UC1, CV_16UC2, CV_16UC3, CV_16UC4, CV_16SC1, CV_16SC2, CV_16SC3, CV_16SC4, CV_32SC1, CV_32SC2, CV_32SC3, CV_32SC4, CV_32FC1, CV_32FC2, CV_32FC3, CV_32FC4, CV_64FC1, CV_64FC2, CV_64FC3, CV_64FC4, NORM_HAMMING, NORM_HAMMING2, NORM_INF, NORM_L1, NORM_L2, NORM_L2SQR, NORM_MINMAX, NORM_RELATIVE, NORM_TYPE_MASK, COLOR_BGR2BGRA, COLOR_RGB2RGBA, COLOR_BGRA2BGR, COLOR_RGBA2RGB, COLOR_BGR2RGBA, COLOR_RGB2BGRA, COLOR_RGBA2BGR, COLOR_BGRA2RGB, COLOR_BGR2RGB, COLOR_RGB2BGR, COLOR_BGRA2RGBA, COLOR_RGBA2BGRA, COLOR_BGR2GRAY, COLOR_RGB2GRAY, COLOR_GRAY2BGR, COLOR_GRAY2RGB, COLOR_GRAY2BGRA, COLOR_GRAY2RGBA, COLOR_BGRA2GRAY, COLOR_RGBA2GRAY, COLOR_BGR2BGR565, COLOR_RGB2BGR565, COLOR_BGR5652BGR, COLOR_BGR5652RGB, COLOR_BGRA2BGR565, COLOR_RGBA2BGR565, COLOR_BGR5652BGRA, COLOR_BGR5652RGBA, COLOR_GRAY2BGR565, COLOR_BGR5652GRAY, COLOR_BGR2BGR555, COLOR_RGB2BGR555, COLOR_BGR5552BGR, COLOR_BGR5552RGB, COLOR_BGRA2BGR555, COLOR_RGBA2BGR555, COLOR_BGR5552BGRA, COLOR_BGR5552RGBA, COLOR_GRAY2BGR555, COLOR_BGR5552GRAY, COLOR_BGR2XYZ, COLOR_RGB2XYZ, COLOR_XYZ2BGR, COLOR_XYZ2RGB, COLOR_BGR2YCrCb, COLOR_RGB2YCrCb, COLOR_YCrCb2BGR, COLOR_YCrCb2RGB, COLOR_BGR2HSV, COLOR_RGB2HSV, COLOR_BGR2Lab, COLOR_RGB2Lab, COLOR_BGR2Luv, COLOR_RGB2Luv, COLOR_BGR2HLS, COLOR_RGB2HLS, COLOR_HSV2BGR, COLOR_HSV2RGB, COLOR_Lab2BGR, COLOR_Lab2RGB, COLOR_Luv2BGR, COLOR_Luv2RGB, COLOR_HLS2BGR, COLOR_HLS2RGB, COLOR_BGR2HSV_FULL, COLOR_RGB2HSV_FULL, COLOR_BGR2HLS_FULL, COLOR_RGB2HLS_FULL, COLOR_HSV2BGR_FULL, COLOR_HSV2RGB_FULL, COLOR_HLS2BGR_FULL, COLOR_HLS2RGB_FULL, COLOR_LBGR2Lab, COLOR_LRGB2Lab, COLOR_LBGR2Luv, COLOR_LRGB2Luv, COLOR_Lab2LBGR, COLOR_Lab2LRGB, COLOR_Luv2LBGR, COLOR_Luv2LRGB, COLOR_BGR2YUV, COLOR_RGB2YUV, COLOR_YUV2BGR, COLOR_YUV2RGB, COLOR_YUV2RGB_NV12, COLOR_YUV2BGR_NV12, COLOR_YUV2RGB_NV21, COLOR_YUV2BGR_NV21, COLOR_YUV420sp2RGB, COLOR_YUV420sp2BGR, COLOR_YUV2RGBA_NV12, COLOR_YUV2BGRA_NV12, COLOR_YUV2RGBA_NV21, COLOR_YUV2BGRA_NV21, COLOR_YUV420sp2RGBA, COLOR_YUV420sp2BGRA, COLOR_YUV2RGB_YV12, COLOR_YUV2BGR_YV12, COLOR_YUV2RGB_IYUV, COLOR_YUV2BGR_IYUV, COLOR_YUV2RGB_I420, COLOR_YUV2BGR_I420, COLOR_YUV420p2RGB, COLOR_YUV420p2BGR, COLOR_YUV2RGBA_YV12, COLOR_YUV2BGRA_YV12, COLOR_YUV2RGBA_IYUV, COLOR_YUV2BGRA_IYUV, COLOR_YUV2RGBA_I420, COLOR_YUV2BGRA_I420, COLOR_YUV420p2RGBA, COLOR_YUV420p2BGRA, COLOR_YUV2GRAY_420, COLOR_YUV2GRAY_NV21, COLOR_YUV2GRAY_NV12, COLOR_YUV2GRAY_YV12, COLOR_YUV2GRAY_IYUV, COLOR_YUV2GRAY_I420, COLOR_YUV420sp2GRAY, COLOR_YUV420p2GRAY, COLOR_YUV2RGB_UYVY, COLOR_YUV2BGR_UYVY, COLOR_YUV2RGB_Y422, COLOR_YUV2BGR_Y422, COLOR_YUV2RGB_UYNV, COLOR_YUV2BGR_UYNV, COLOR_YUV2RGBA_UYVY, COLOR_YUV2BGRA_UYVY, COLOR_YUV2RGBA_Y422, COLOR_YUV2BGRA_Y422, COLOR_YUV2RGBA_UYNV, COLOR_YUV2BGRA_UYNV, COLOR_YUV2RGB_YUY2, COLOR_YUV2BGR_YUY2, COLOR_YUV2RGB_YVYU, COLOR_YUV2BGR_YVYU, COLOR_YUV2RGB_YUYV, COLOR_YUV2BGR_YUYV, COLOR_YUV2RGB_YUNV, COLOR_YUV2BGR_YUNV, COLOR_YUV2RGBA_YUY2, COLOR_YUV2BGRA_YUY2, COLOR_YUV2RGBA_YVYU, COLOR_YUV2BGRA_YVYU, COLOR_YUV2RGBA_YUYV, COLOR_YUV2BGRA_YUYV, COLOR_YUV2RGBA_YUNV, COLOR_YUV2BGRA_YUNV, COLOR_YUV2GRAY_UYVY, COLOR_YUV2GRAY_YUY2, COLOR_YUV2GRAY_Y422, COLOR_YUV2GRAY_UYNV, COLOR_YUV2GRAY_YVYU, COLOR_YUV2GRAY_YUYV, COLOR_YUV2GRAY_YUNV, COLOR_RGBA2mRGBA, COLOR_mRGBA2RGBA, COLOR_RGB2YUV_I420, COLOR_BGR2YUV_I420, COLOR_RGB2YUV_IYUV, COLOR_BGR2YUV_IYUV, COLOR_RGBA2YUV_I420, COLOR_BGRA2YUV_I420, COLOR_RGBA2YUV_IYUV, COLOR_BGRA2YUV_IYUV, COLOR_RGB2YUV_YV12, COLOR_BGR2YUV_YV12, COLOR_RGBA2YUV_YV12, COLOR_BGRA2YUV_YV12, COLOR_BayerBG2BGR, COLOR_BayerGB2BGR, COLOR_BayerRG2BGR, COLOR_BayerGR2BGR, COLOR_BayerBG2RGB, COLOR_BayerGB2RGB, COLOR_BayerRG2RGB, COLOR_BayerGR2RGB, COLOR_BayerBG2GRAY, COLOR_BayerGB2GRAY, COLOR_BayerRG2GRAY, COLOR_BayerGR2GRAY, COLOR_BayerBG2BGR_VNG, COLOR_BayerGB2BGR_VNG, COLOR_BayerRG2BGR_VNG, COLOR_BayerGR2BGR_VNG, COLOR_BayerBG2RGB_VNG, COLOR_BayerGB2RGB_VNG, COLOR_BayerRG2RGB_VNG, COLOR_BayerGR2RGB_VNG, COLOR_BayerBG2BGR_EA, COLOR_BayerGB2BGR_EA, COLOR_BayerRG2BGR_EA, COLOR_BayerGR2BGR_EA, COLOR_BayerBG2RGB_EA, COLOR_BayerGB2RGB_EA, COLOR_BayerRG2RGB_EA, COLOR_BayerGR2RGB_EA, COLOR_BayerBG2BGRA, COLOR_BayerGB2BGRA, COLOR_BayerRG2BGRA, COLOR_BayerGR2BGRA, COLOR_BayerBG2RGBA, COLOR_BayerGB2RGBA, COLOR_BayerRG2RGBA, COLOR_BayerGR2RGBA, RETR_EXTERNAL, RETR_LIST, RETR_CCOMP, RETR_TREE, RETR_FLOODFILL, CHAIN_APPROX_NONE, CHAIN_APPROX_SIMPLE, CHAIN_APPROX_TC89_L1, CHAIN_APPROX_TC89_KCOS, BORDER_CONSTANT, BORDER_REPLICATE, BORDER_REFLECT, BORDER_WRAP, BORDER_REFLECT_101, BORDER_TRANSPARENT, BORDER_REFLECT101, BORDER_DEFAULT, BORDER_ISOLATED, THRESH_BINARY, THRESH_BINARY_INV, THRESH_TRUNC, THRESH_TOZERO, THRESH_TOZERO_INV, THRESH_MASK, THRESH_OTSU, THRESH_TRIANGLE, MORPH_RECT, MORPH_CROSS, MORPH_ELLIPSE, CAP_ANY, CAP_VFW, CAP_V4L, CAP_V4L2, CAP_FIREWIRE, CAP_FIREWARE, CAP_IEEE1394, CAP_DC1394, CAP_CMU1394, CAP_QT, CAP_UNICAP, CAP_DSHOW, CAP_PVAPI, CAP_OPENNI, CAP_OPENNI_ASUS, CAP_ANDROID, CAP_XIAPI, CAP_AVFOUNDATION, CAP_GIGANETIX, CAP_MSMF, CAP_WINRT, CAP_INTELPERC, CAP_OPENNI2, CAP_OPENNI2_ASUS, CAP_GPHOTO2, CAP_GSTREAMER, CAP_FFMPEG, CAP_IMAGES, CAP_ARAVIS, CAP_OPENCV_MJPEG, CAP_INTEL_MFX, CAP_XINE, CAP_PROP_POS_MSEC, CAP_PROP_POS_FRAMES, CAP_PROP_POS_AVI_RATIO, CAP_PROP_FRAME_WIDTH, CAP_PROP_FRAME_HEIGHT, CAP_PROP_FPS, CAP_PROP_FOURCC, CAP_PROP_FRAME_COUNT, CAP_PROP_FORMAT, CAP_PROP_MODE, CAP_PROP_BRIGHTNESS, CAP_PROP_CONTRAST, CAP_PROP_SATURATION, CAP_PROP_HUE, CAP_PROP_GAIN, CAP_PROP_EXPOSURE, CAP_PROP_CONVERT_RGB, CAP_PROP_WHITE_BALANCE_BLUE_U, CAP_PROP_RECTIFICATION, CAP_PROP_MONOCHROME, CAP_PROP_SHARPNESS, CAP_PROP_AUTO_EXPOSURE, CAP_PROP_GAMMA, CAP_PROP_TEMPERATURE, CAP_PROP_TRIGGER, CAP_PROP_TRIGGER_DELAY, CAP_PROP_WHITE_BALANCE_RED_V, CAP_PROP_ZOOM, CAP_PROP_FOCUS, CAP_PROP_GUID, CAP_PROP_ISO_SPEED, CAP_PROP_BACKLIGHT, CAP_PROP_PAN, CAP_PROP_TILT, CAP_PROP_ROLL, CAP_PROP_IRIS, CAP_PROP_SETTINGS, CAP_PROP_BUFFERSIZE, CAP_PROP_AUTOFOCUS, CAP_PROP_SAR_NUM, CAP_PROP_SAR_DEN, CAP_PROP_BACKEND, CAP_PROP_CHANNEL, CAP_PROP_AUTO_WB, CAP_PROP_WB_TEMPERATURE, CAP_PROP_CODEC_PIXEL_FORMAT, VIDEOWRITER_PROP_QUALITY, VIDEOWRITER_PROP_FRAMEBYTES, VIDEOWRITER_PROP_NSTRIPES, FONT_HERSHEY_SIMPLEX, FONT_HERSHEY_PLAIN, FONT_HERSHEY_DUPLEX, FONT_HERSHEY_COMPLEX, FONT_HERSHEY_TRIPLEX, FONT_HERSHEY_COMPLEX_SMALL, FONT_HERSHEY_SCRIPT_SIMPLEX, FONT_HERSHEY_SCRIPT_COMPLEX, FONT_ITALIC, HIER_NEXT, HIER_PREV, HIER_CHILD, HIER_PARENT, HOUGH_STANDARD, HOUGH_PROBABILISTIC, HOUGH_MULTI_SCALE, HOUGH_GRADIENT, INTER_NEAREST, INTER_LINEAR, INTER_CUBIC, INTER_AREA, INTER_LANCZOS4, INTER_LINEAR_EXACT, INTER_MAX, CONTOURS_MATCH_I1, CONTOURS_MATCH_I2, CONTOURS_MATCH_I3, ACCESS_READ, ACCESS_WRITE, ACCESS_RW, ACCESS_MASK, ACCESS_FAST, USAGE_DEFAULT, USAGE_ALLOCATE_HOST_MEMORY, USAGE_ALLOCATE_DEVICE_MEMORY, USAGE_ALLOCATE_SHARED_MEMORY, IMREAD_UNCHANGED, IMREAD_GRAYSCALE, IMREAD_COLOR, IMREAD_ANYDEPTH, IMREAD_ANYCOLOR, IMREAD_LOAD_GDAL, IMREAD_REDUCED_GRAYSCALE_2, IMREAD_REDUCED_COLOR_2, IMREAD_REDUCED_GRAYSCALE_4, IMREAD_REDUCED_COLOR_4, IMREAD_REDUCED_GRAYSCALE_8, IMREAD_REDUCED_COLOR_8, IMREAD_IGNORE_ORIENTATION, IMWRITE_JPEG_QUALITY, IMWRITE_JPEG_PROGRESSIVE, IMWRITE_JPEG_OPTIMIZE, IMWRITE_JPEG_RST_INTERVAL, IMWRITE_JPEG_LUMA_QUALITY, IMWRITE_JPEG_CHROMA_QUALITY, IMWRITE_PNG_COMPRESSION, IMWRITE_PNG_STRATEGY, IMWRITE_PNG_BILEVEL, IMWRITE_PXM_BINARY, IMWRITE_EXR_TYPE, IMWRITE_WEBP_QUALITY, IMWRITE_PAM_TUPLETYPE, IMWRITE_TIFF_RESUNIT, IMWRITE_TIFF_XDPI, IMWRITE_TIFF_YDPI, IMWRITE_TIFF_COMPRESSION, IMWRITE_JPEG2000_COMPRESSION_X1000, IMWRITE_EXR_TYPE_HALF, IMWRITE_EXR_TYPE_FLOAT, IMWRITE_PNG_STRATEGY_DEFAULT, IMWRITE_PNG_STRATEGY_FILTERED, IMWRITE_PNG_STRATEGY_HUFFMAN_ONLY, IMWRITE_PNG_STRATEGY_RLE, IMWRITE_PNG_STRATEGY_FIXED, IMWRITE_PAM_FORMAT_NULL, IMWRITE_PAM_FORMAT_BLACKANDWHITE, IMWRITE_PAM_FORMAT_GRAYSCALE, IMWRITE_PAM_FORMAT_GRAYSCALE_ALPHA, IMWRITE_PAM_FORMAT_RGB, IMWRITE_PAM_FORMAT_RGB_ALPHA, FILLED, LINE_8, LINE_AA, LSD_REFINE_NONE, LSD_REFINE_STD, LSD_REFINE_ADV, COLORMAP_AUTUMN, COLORMAP_BONE, COLORMAP_JET, COLORMAP_WINTER, COLORMAP_RAINBOW, COLORMAP_OCEAN, COLORMAP_SUMMER, COLORMAP_SPRING, COLORMAP_COOL, COLORMAP_HSV, COLORMAP_PINK, COLORMAP_HOT, COLORMAP_PARULA, COLORMAP_MAGMA, COLORMAP_INFERNO, COLORMAP_PLASMA, COLORMAP_VIRIDIS, COLORMAP_CIVIDIS, COLORMAP_TWILIGHT, COLORMAP_TWILIGHT_SHIFTED, COLORMAP_TURBO, DEFAULT, DRAW_OVER_OUTIMG, NOT_DRAW_SINGLE_POINTS, DRAW_RICH_KEYPOINTS, ALIGN_CENTER, ALIGN_LEFT, ALIGN_RIGHT, ALIGN_HORIZONTAL, ALIGN_MIDDLE, ALIGN_TOP, ALIGN_BOTTOM, ALIGN_VERTICAL, default, imshow, namedWindow, moveWindow, resizeWindow, destroyWindow, getWindowImageRect, getWindowProperty, setWindowProperty, setWindowTitle, createTrackbar, createButton, getTrackbarPos, setTrackbarPos, setTrackbarMin, setTrackbarMax, getMouseWheelDelta, setMouseCallback, pollKey, waitKey, waitKeyEx, getScreenResolution, selectROI, selectROIs, displayOverlay, WINDOW_NORMAL, WINDOW_AUTOSIZE, WINDOW_OPENGL, WINDOW_FULLSCREEN, WINDOW_FREERATIO, WINDOW_KEEPRATIO, WINDOW_GUI_EXPANDED, WINDOW_GUI_NORMAL, WND_PROP_FULLSCREEN, WND_PROP_AUTOSIZE, WND_PROP_ASPECT_RATIO, WND_PROP_OPENGL, WND_PROP_VISIBLE, WND_PROP_TOPMOST, EVENT_MOUSEMOVE, EVENT_LBUTTONDOWN, EVENT_RBUTTONDOWN, EVENT_MBUTTONDOWN, EVENT_LBUTTONUP, EVENT_RBUTTONUP, EVENT_MBUTTONUP, EVENT_LBUTTONDBLCLK, EVENT_RBUTTONDBLCLK, EVENT_MBUTTONDBLCLK, EVENT_MOUSEWHEEL, EVENT_MOUSEHWHEEL, EVENT_FLAG_LBUTTON, EVENT_FLAG_RBUTTON, EVENT_FLAG_MBUTTON, EVENT_FLAG_CTRLKEY, EVENT_FLAG_SHIFTKEY, EVENT_FLAG_ALTKEY, blur, boundingRect, GaussianBlur, HoughLines, HoughLinesP, HoughCircles, Canny, goodFeaturesToTrack, cvtColor, equalizeHist, threshold, bilateralFilter, findContours, pointPolygonTest, cornerHarris, calcHist, dilate, erode, morphologyEx, medianBlur, skeletonization, pixelNeighborhood, pixelNeighborhoodCross, pixelFindValue, lineSegmentDetector, traceSkeleton, paletteGenerate, paletteApply, paletteMatch, accumulate, accumulateProduct, accumulateSquare, accumulateWeighted, createHanningWindow, phaseCorrelate, adaptiveThreshold, blendLinear, distanceTransform, floodFill, grabCut, integral, watershed, applyColorMap, moments, convertMaps, getAffineTransform, getPerspectiveTransform, getRectSubPix, getRotationMatrix2D, getRotationMatrix2D_, invertAffineTransform, linearPolar, logPolar, remap, resize, warpAffine, warpPerspective, warpPolar, boxFilter, buildPyramid, filter2D, getDerivKernels, getGaborKernel, getGaussianKernel, getStructuringElement, Laplacian, morphologyDefaultBorderValue, pyrDown, pyrMeanShiftFiltering, pyrUp, Scharr, sepFilter2D, Sobel, spatialGradient, sqrBoxFilter, approxPolyDP, arcLength, boxPoints, connectedComponents, connectedComponentsWithStats, contourArea, convexHull, convexityDefects, createGeneralizedHoughBallard, createGeneralizedHoughGuil, fitEllipse, fitEllipseAMS, fitEllipseDirect, fitLine, HuMoments, intersectConvexConvex, isContourConvex, matchShapes, minAreaRect, minEnclosingCircle, minEnclosingTriangle, rotatedRectangleIntersection, ximgproc, CLAHE, Contour, Draw, drawCircle, drawEllipse, drawContour, drawContours, drawLine, drawPolygon, drawPolylines, drawRect, drawKeypoints, putText, getTextSize, getFontScaleFromHeight, loadFont, clipLine, Line, Mat, Affine3, Point, PointIterator, Rect, RotatedRect,SliceIterator, Subdiv2D, UMat, TickMeter, VideoCapture, VideoWriter, LineSegmentDetector, FastLineDetector, KeyPoint, AffineFeature, AgastFeatureDetector, AKAZE, BRISK, FastFeatureDetector, GFTTDetector, KAZE, MSER, ORB, SIFT, SimpleBlobDetector, AffineFeature2D, BoostDesc, BriefDescriptorExtractor, DAISY, FREAK, HarrisLaplaceFeatureDetector, LATCH, LUCID, MSDDetector, StarDetector, SURF, VGG, RaspiCam } from 'opencv';

function Clear(color = nvg.RGB(0, 0, 0)) {
  const { size } = glfw.context.current;

  nvg.Save();
  nvg.BeginPath();
  nvg.Rect(0, 0, ...size);
  nvg.FillColor(color);
  nvg.Fill();
  nvg.Restore();
}

function main(...args) {
  globalThis.console = new Console({
    inspectOptions: {
      maxStringLength: 200,
      maxArrayLength: 10,
      breakLength: Infinity,
      compact: 2,
      depth: 10
    }
  });

  let i = 0;
  let running = true;

  let context, window, position, size;

  if(true) {
    for(let [prop, value] of [
      [CONTEXT_VERSION_MAJOR, 3],
      [CONTEXT_VERSION_MINOR, 2],
      [OPENGL_PROFILE, OPENGL_CORE_PROFILE],
      [OPENGL_FORWARD_COMPAT, true],
      [RESIZABLE, true],
      [SAMPLES, 4]
    ])
      Window.hint(prop, value);

    window = glfw.context.current = new Window(1280, 900, 'ImGui test');

    context = {
      begin() {},
      end() {
        window.swapBuffers();
        poll();
      }
    };

    window.handleCharMods = (char, mods) => {
      console.log(`handleCharMods`, { char, mods });
    };
  } else {
    context = new GLFW(1280, 900, {
      title: scriptArgs[0],
      resizable: true,
      handleSize(width, height) {
        console.log('resized', { width, height });
      },
      handleKey(keyCode) {
        let charCode = keyCode & 0xff;
        console.log(`handleKey`, {
          keyCode: '0x' + keyCode.toString(16),
          charCode,
          char: String.fromCharCode(charCode)
        });
        let char = String.fromCodePoint(charCode);

        let handler = { '\x00': () => (running = false), Q: () => (running = false) }[char];
        if(handler) handler();
      },
      handleCharMods(char, mods) {
        console.log(`handleCharMods`, { char, mods });
      },
      handleMouseButton(button, action, mods) {
        console.log(`handleMouseButton`, { button, action, mods });
      },
      handleCursorPos(x, y) {
        //console.log(`handleCursorPos`, { x, y });
      }
    });
    window = context.window;
  }

  position = window.position;
  size = window.size;

  nvg.CreateGL3(nvg.STENCIL_STROKES | nvg.ANTIALIAS | nvg.DEBUG);

  ImGui.Init(ImGui.ImplGlfw, ImGui.ImplOpenGL3);
  ImGui.CreateContext(window);

  const { width, height } = size;
  const { x, y } = position;

  //console.log(`width: ${width}, height: ${height}, x: ${x}, y: ${y}`);

  let mat = new Mat(height, width, CV_8UC4);

  mat.setTo([11, 22, 33, 255]);

  let image2 = imread('Architektur.png');

  cvtColor(image2, image2, COLOR_BGR2BGRA);

  image2.copyTo(mat(new Rect(image2.size)));

  drawLine(mat, new Point(10, 10), new Point(size.width - 10, size.height - 10), [255, 255, 0, 255], 4, LINE_AA);
  drawLine(mat, new Point(size.width - 10, 10), new Point(10, size.height - 10), [255, 0, 0, 255], 4, LINE_AA);

  let { buffer } = mat;

  let pixels;
  let imgId = Mat2Image(mat);
  let img2Id = nvg.CreateImage('Muehleberg.png', 0);

  console.log(``, { imgId, img2Id });

  let img2Sz = nvg.ImageSize(img2Id);
  let imgSz = nvg.ImageSize(imgId);

  const timer = {
    ticks(rate = 1000) {
      const t = Date.now();

      return ((t - (this.start ??= t)) * rate) / 1000;
    },
    frame() {
      const t = Date.now();
      const { prev = this.start } = this;
      this.prev = t;
      return (this.fps = +(1000 / (t - prev)).toFixed(1));
    }
  };

  console.log('FPS:');

  let floatValue = 0.0,
    alphaValue = 127;

  while((running &&= !window.shouldClose)) {
    let index = Math.floor(timer.ticks(360) / 30);

    let color = new HSLA(index % 360, 100, 50 + 25 * Math.sin(timer.ticks(0.1) * Math.PI)).toRGBA();

    context.begin(color);

    nvg.BeginFrame(width, height, 1);

    Clear(nvg.RGB(...color));

    let m;
    nvg.CurrentTransform((m = []));

    let p, a;
    p = nvg.TransformIdentity();

    //   nvg.Scale(2,1);

    nvg.TransformMultiply(
      p,
      nvg.TransformTranslate(0, 100),
      nvg.TransformRotate(((timer.ticks(200) % 360) * Math.PI) / 180),
      nvg.TransformRotate(nvg.DegToRad(45)),
      nvg.TransformScale(2, 0.5),
      nvg.TransformRotate(nvg.DegToRad(-45))
    );
    //console.log('Transform', p);

    nvg.TransformPoint((a = []), p, 0, 0);

    // let pattern = nvg.ImagePattern(0, 0, ...img2Sz, 0, img2Id, 1);

    let center = new Position(size.width / 2, size.height / 2);
    let imgSz_2 = new Position(img2Sz.width * -0.5, img2Sz.height * -0.5);

    nvg.Save();

    nvg.Translate(...center);
    nvg.Scale(0.5, 0.5);
    nvg.Translate(...imgSz_2);

    let phi = ((timer.ticks(60) % 360) / 180) * Math.PI;
    let vec = [Math.cos(phi), Math.sin(phi)].map(n => n * 100);

    DrawImage(imgId, vec);
    //  nvg.Translate(imgSz_2.width * -1, imgSz_2.height * -1);
    nvg.Restore();
    nvg.Save();

    nvg.Translate(size.width / 4, size.height / 4);
    /*   nvg.Scale(2,1);
    nvg.Rotate(((timer.ticks(180) % 360) * Math.PI) / 180);
  nvg.Scale(0.5,1);
 */
    DrawCircle(a, 40, 2, [0, 0, 0], [255, 255, 0, 192]);

    nvg.Restore();

    DrawCircle(center, 100);

    nvg.EndFrame();

    ImGui.NewFrame();

    ImGui.Begin('Parameters', null, ImGui.WindowFlags.MenuBar);

    ImGui.SetWindowSize([400, 300]);
    ImGui.PushItemWidth(ImGui.GetFontSize() * -12);

    ImGui.Text('Adjust values for this processing step:');

   /* ImGui.DragFloat('Value', val => (val === undefined ? floatValue : (floatValue = val)), 0.0, 1.0, '%.3f');
    ImGui.SliderFloat('Value', val => (val === undefined ? floatValue : (floatValue = val)), 0.0, 1.0, '%.3f');
    ImGui.SliderInt('Alpha', val => (val === undefined ? Math.floor(alphaValue) : (alphaValue = Math.floor(val))), 0, 255);
*/
    ImGui.End();

    ImGui.Render();

    let data = ImGui.GetDrawData();

    ImGui.RenderDrawData(data);

    /* let { id,Valid, CmdListsCount, TotalIdxCount, TotalVtxCount, DisplayPos, DisplaySize, FramebufferScale } = data;
    console.log('data', id, {Valid, CmdListsCount, TotalIdxCount, TotalVtxCount, DisplayPos, DisplaySize, FramebufferScale });
*/
    context.end();
    /*window.swapBuffers();
    glfw.poll();*/
    i++;

    timer.frame();
  }
}

try {
  main(...scriptArgs.slice(1));
} catch(error) {
  console.log(`Exception (${className(error)}): `,error);
  console.log( 'FAIL: ' + error.message + '\n' + error.stack);
  //os.kill(process.pid, os.SIGUSR1);
  std.exit(1);
}
