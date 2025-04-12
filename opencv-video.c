//  g++ -Wl,-rpath=/opt/opencv-4.5.5-x86_64/lib $(pkgcfg --cflags  opencv4 |explode ' ')  -o
//  opencv-video{,.c} -g -Os -w -lav{format,codec,util} -lswresample  -lswscale $(pkgcfg --libs
//  opencv4|explode ' ')

#define __STDC_CONSTANT_MACROS
#include <stdio.h>
#include <unistd.h>
#include <time.h>
// Opencv
//#include <opencv/cv.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>

extern "C" {
#include "libavutil/avutil.h"
#include "libavcodec/avcodec.h"
#include "libavformat/avformat.h"
// The header file that needs to be introduced for the image conversion structure in the new
// version
#include "libswscale/swscale.h"
};

using namespace cv;
using namespace std;

int
main(int argc, char* argv[]) {
  // decoder pointer
  AVCodec *videoCodec, *audioCodec;
  // Class members of the ffmpeg decoding class
  AVCodecContext* pCodecCtx;
  // Multimedia frame, save the decoded data frame
  AVFrame* videoFrame;
  // Save video stream information
  AVFormatContext* videoFormat;

  if(argc <= 1) {
    printf("need filename\n");
    return -1;
  }
  char* filename = argv[1];
  // register all available file formats and encoders in the library
  av_register_all();

  videoFormat = avformat_alloc_context();
  // check file header
  if(avformat_open_input(&videoFormat, filename, NULL, NULL) != 0) {
    printf("Can't find the stream!\n");
  }
  // find stream information
  if(avformat_find_stream_info(videoFormat, NULL) < 0) {
    printf("Can't find the stream information !\n");
  }

  int videoindex = -1, audioindex = -1;
  // Traverse each stream, find the first video stream, and record the encoding information of
  // the stream
  for(int i = 0; i < videoFormat->nb_streams; ++i) {
    if(videoFormat->streams[i]->codec->codec_type == AVMEDIA_TYPE_VIDEO) {
      videoindex = i;
      break;
    }
  }
  if(videoindex == -1) {
    printf("Don't find a video stream !\n");
    return -1;
  }
  if((audioindex =
          av_find_best_stream(videoFormat, AVMEDIA_TYPE_AUDIO, -1, -1, &audioCodec, -1)) < 0) {
    printf("av_find_best_stream: Failed to locate audio stream\n");
    return -1;
  }
  printf("videoindex = %d, audioindex = %d\n", videoindex, audioindex);

  // get a context pointer to the video stream
  pCodecCtx = videoFormat->streams[videoindex]->codec;
  // to the decoder of this format
  videoCodec = avcodec_find_decoder(pCodecCtx->codec_id);
  if(videoCodec == NULL) {
    // find the decoder
    printf("Cant't find the decoder !\n");
    return -1;
  }
  // open the decoder
  if(avcodec_open2(pCodecCtx, videoCodec, NULL) < 0) {
    printf("Can't open the decoder !\n");
    return -1;
  }

  // allocate frame storage space
  videoFrame = av_frame_alloc();
  // Store the converted RGB data after decoding
  AVFrame* pFrameBGR = av_frame_alloc();

  // Save BGR, which is saved by BGR in opencv
  int size = avpicture_get_size(AV_PIX_FMT_BGR24, pCodecCtx->width, pCodecCtx->height);
  uint8_t* out_buffer = (uint8_t*)av_malloc(size);
  avpicture_fill(
      (AVPicture*)pFrameBGR, out_buffer, AV_PIX_FMT_BGR24, pCodecCtx->width, pCodecCtx->height);

  AVPacket* packet = (AVPacket*)malloc(sizeof(AVPacket));
  printf("------------file information---------\n");
  av_dump_format(videoFormat, 0, filename, 0);
  /*  printf("------------audio format---------\n");
  av_dump_format(audioFormat, 0, filename, 0);*/
  printf("-----------------------------");

  struct SwsContext* img_convert_ctx;
  img_convert_ctx = sws_getContext(pCodecCtx->width,
                                   pCodecCtx->height,
                                   pCodecCtx->pix_fmt,
                                   pCodecCtx->width,
                                   pCodecCtx->height,
                                   AV_PIX_FMT_BGR24,
                                   SWS_BICUBIC,
                                   NULL,
                                   NULL,
                                   NULL);

  // opencv
  cv::Mat pCvMat;
  pCvMat.create(cv::Size(pCodecCtx->width, pCodecCtx->height), CV_8UC3);

  int ret;
  int got_picture;

  Mat gray;
  /*  CascadeClassifier cascade;
    cascade.load("haarcascade_frontalface_alt.xml");
  */
  cv::namedWindow("RGB", 1);
  cv::resizeWindow("RGB", pCodecCtx->width, pCodecCtx->height);

  time_t t;
  while(1) {
    if(av_read_frame(videoFormat, packet) >= 0) {
      if(packet->stream_index == videoindex) {
        ret = avcodec_decode_video2(pCodecCtx, videoFrame, &got_picture, packet);
        if(ret < 0) {
          printf("Decode Error.(Decode Error)\n");
          return -1;
        }
        if(!got_picture) {
          printf("No picture (Decode Error)\n");
          return -1;
        }
        // YUV to RGB
        sws_scale(img_convert_ctx,
                  (const uint8_t* const*)videoFrame->data,
                  videoFrame->linesize,
                  0,
                  pCodecCtx->height,
                  pFrameBGR->data,
                  pFrameBGR->linesize);

        memcpy(pCvMat.data, out_buffer, size);

        cv::imshow("RGB", pCvMat);
        cv::waitKey(1);

        ///////////////////////////////////////////// //////////////
        /*  vector<Rect> faces(0);

          cvtColor(pCvMat, gray, cv::COLOR_BGR2GRAY);
          // Resize image, use bilinear difference
          //  resize(gray, smallImg, smallImg.size(), 0, 0, INTER_LINEAR);
          // The transformed image is processed by histogram averaging
          equalizeHist(gray, gray);

          time(&t);
          printf("before %s\n", ctime(&t));
          cascade.detectMultiScale(gray,
                                   faces,
                                   2,
                                   2,
                                   cv::CASCADE_FIND_BIGGEST_OBJECT | cv::CASCADE_DO_ROUGH_SEARCH
          | cv::CASCADE_SCALE_IMAGE, cv::Size(50, 50)); time(&t); printf("after %s\n",
          ctime(&t)); Mat face; Point text_lb;

          for(size_t i = 0; i < faces.size(); i++) {
            if(faces[i].height > 0 && faces[i].width > 0) {
              face = gray(faces[i]);
              text_lb = Point(faces[i].x, faces[i].y);

              rectangle(pCvMat, faces[i], Scalar(255, 0, 0), 1, 8, 0);
              // putText(pCvMat, "name", text_lb, FONT_HERSHEY_COMPLEX, 1, Scalar(0, 0, 255));
            }
          }*/
#if 0
Mat face_test;
int predictPCA = 0;
if (face.rows >= 120) {
resize(face, face_test, Size(92, 112));
#endif
      }
    }
  }
}
