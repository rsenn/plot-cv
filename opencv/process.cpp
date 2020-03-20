#include <thread>
#include <mutex>
#include <ctime>

#ifdef _WIN32
#include <windows.h>
#include "win.hpp"
#else // On Windows

#include <unistd.h>
#include "network/serverAPI.hpp"
#include "network/proxyAPI.hpp"
#endif // On Linux

#include "opencv/plate.hpp"
#include "opencv/svm.hpp"
#include "opencv/ocr.hpp"
#include "opencv/tools.hpp"

#include "wiringpi/lcd.hpp"
#include "wiringpi/barricade.hpp"

#include "process.hpp"

using namespace cv;
using namespace std;

LCD lcd;
Barricade brcd(1);

void
process::send2Server(const ParkingInfo& info, Table table[SEGMENTSIZE + 1], std::string ip) {

  string serveraddr = "13.124.74.249";
  string proxyaddr = ip;

  if(info.floor) {
    for(int i = 1; i <= SEGMENTSIZE; i++) {
      /* 이미 Server에 보낸 경우*/
      if(table[i].sended)
        continue;

      if(table[i].match < 0)
        continue;

      brcd.open();

      cout << "\tsend to server "
           << "(" << i << "/" << SEGMENTSIZE << ") (parking mode)" << endl;

      ps::ServerAPI api(serveraddr, 3000);

      api.parking(info.floor, info.zoneName, i, table[i].plateStr);

      api.resopnse();

      /* 주차 차량 정보 갱신 */
      table[i].sended = true;

      cout << "\tsending complete" << endl;
    }
  } else {
    /* 이미 Server에 보낸 경우*/
    if(table[0].sended)
      return;

    if(info.way == ENTER) {
      if(table[0].match < LEASTMATCH)
        return;
    }

    cout << "\tsend to server (entrance mode)" << endl;

    ps::ServerAPI api(serveraddr, 3000);
    ps::ProxyAPI proxyAPI(proxyaddr, 3001);

    if(info.way == ENTER) {
      api.enter(table[0].plateStr);

      proxyAPI.enter(table[0].plateStr);
    } else if(info.way == EXIT) {
      api.exit(table[0].plateStr);

      proxyAPI.exit(table[0].plateStr);
    }

    brcd.open();

    proxyAPI.resopnse();
    api.resopnse();

    /* 주차 차량 정보 갱신 */
    table[0].sended = true;

    cout << "\tsending complete" << endl;
  }
}

/* Proxy 서버와 동기화 */
void
process::sync(vector<string>* list, std::string ip) {
  // cout << "\tsync with Proxy" <<endl;
  ps::ProxyAPI api(ip, 3001);

  api.print();

  string recv;
  api.resopnse(&recv);

  int start = 0;

  string tmp = "";

  list->clear();

  int recvLen = recv.length();

  for(int i = 0; i < recvLen; i++) {
    if(recv[i] == '\n') {
      list->push_back(tmp);
      tmp = "";
    } else
      tmp += recv[i];
  }

  // for (auto text : *list)
  //	cout << "\t\t" << text << endl;

  // cout << "\tsync was done" <<endl;
}

int
process::deductIndex(const cv::Rect area[SEGMENTSIZE], const cv::Point& position) {

  for(int j = 0; j < SEGMENTSIZE; j++) {
    if(area[j].contains(position))
      return 4 - j;
  }

  return 0;
}

void
process::printTable(Table table[SEGMENTSIZE + 1]) {

  cout << "\t\tstr"
       << "\t";
  cout << "\t\tmatch"
       << "\t";
  cout << "\t\tsended"
       << "\t";
  cout << endl;

  for(int i = 0; i <= SEGMENTSIZE; i++) {
    cout << "\t\t" << table[i].plateStr << "\t";
    cout << "\t\t" << table[i].match << "\t";
    cout << "\t\t" << table[i].sended << "\t";
    cout << endl;
  }
}

int
process::startOpencv(
    int width, int height, int mode, ParkingInfo info, std::string answer, std::string ip) {

#if FROM == CAMERA
  VideoCapture camera;
  Mat cameraFrame;

  /** Camera 열기 */
  camera.open(0);
  while(!camera.isOpened()) {
    camera.open(0);
    cerr << "Can Not Access The Camera." << endl;
#ifdef _WIN32
    Sleep(2);
#else
    sleep(2);
#endif
  }

  camera.set(CV_CAP_PROP_FRAME_WIDTH, width);
  camera.set(CV_CAP_PROP_FRAME_HEIGHT, height);

#endif

  Mat image;

  /** 주차 영역 */
  Rect area[SEGMENTSIZE];
  Scalar blue(255, 0, 0), red(0, 0, 255), white(255, 255, 255);

  cout << "Loading Machine learning Modules." << endl;

  OCR *ocrChar, *ocrNum;
  Svm* svm;

/** OpenMP Thread 생성 */
#pragma omp parallel
#pragma omp sections
  {
#pragma omp section
    if(!(mode & NOTUSEML))
      ocrChar = new OCR(CHARACTER, OCR::READDT);
#pragma omp section
    if(!(mode & NOTUSEML))
      ocrNum = new OCR(NUMBER, OCR::READDT);
#pragma omp section
    svm = new Svm(Svm::READDT);
  }

  cout << "Loading was Complete." << endl;

  tools::Analyzer analyzer(answer);
  tools::Dicider dicider;
  OCRTrainer ocrtrainer(answer);
  SVMTrainer svmtrainer;

  mutex m;

  bool runing = true;

  vector<string> carList;
  carList.push_back("4898GXY");
  carList.push_back("3266CNT");
  carList.push_back("8995CCN");

  /** Camera image 불러오는 Thread */
  thread camThread([&] {
    cout << "camThread start" << endl;
    while(runing) {
      m.try_lock();
      camera >> cameraFrame;
      m.unlock();
    }
  });

  /* image 처리 Thread */
  thread procThread([&] {
    cout << "procThread start" << endl;
    Table table[SEGMENTSIZE + 1];
    clock_t otime = -1;
    clock_t ptime = -1;
    string goneCar = "0000TTT";

    /* ESC 입력 시 종료 */
    while(runing = (waitKey(50) != ESC)) {

#if FROM == CAMERA

      /* 하나의 Cycle 당 소요 시간*/
      double cycle_t = (double)getTickCount();

      m.try_lock();
      cameraFrame.copyTo(image);
      m.unlock();

      if(image.empty())
        continue;

      /* 각각의 주차 영역 계산 */
      for(int i = 0; i < SEGMENTSIZE; i++)
        area[i] = Rect(image.cols * i / SEGMENTSIZE, 0, image.cols / SEGMENTSIZE, image.rows);

#elif FROM == FILESYSTEM

      if(waitKey(0) == 27)
        break;

      int img_num;
      cout << "img_num : " << endl;
      cin >> img_num;
      if(!tools::readImage("InputImage/" + to_string(img_num) + ".jpg", image)) {
        cerr << "File No Exist." << endl;
        exit(1);
      }

#endif // File System 으로부터 입력받는 경우

      vector<Plate> possiblePlates;

      Plate::find(image, &possiblePlates);

      vector<Mat> sample;
      int k = 0;

      for(auto& plate : possiblePlates) {
        plate.setDebug(mode & WINDOWON);
        int response = (int)svm->predict(plate.canonical());

        if(mode & WINDOWON)
          imshow("plate", plate.img);

        if(mode & SVMTRAIN)
          svmtrainer.train(plate.img);

        /** Svm을 통해 번호판 여부 확인 */
        if(response != 1)
          continue;

        Plate& foundPlate = plate;

        /* 텍스트 추출 시 소요 시간 */
        double findText_t = (double)getTickCount();

        /* 추출된 텍스트가 TEXTSIZE 일치 여부 */
        bool isText = foundPlate.findTexts(TEXTSIZE);

        findText_t = (double)getTickCount() - findText_t;

#if FROM == CAMERA
        if(!isText)
          continue;
#endif // CAMERA로터 입력받는 경우

        if(mode & COSTTIME)
          cout << "\t\tCost Time In the FindTexts : " << findText_t * 1000 / getTickFrequency()
               << "ms" << endl;

        /* 주차 영역 번호 */
        int zoneIndex = 0;

        if(info.floor) {
          zoneIndex = deductIndex(area, plate.position);

          if(zoneIndex)
            circle(image, plate.position, 2, red, 2);
        }

        int textsSize = (int)foundPlate.texts.size();

        /* text 모음 - Debug 용 */
        Mat textCollection(Size(SAMPLESIZE * textsSize, SAMPLESIZE), CV_8UC3, white);

        /* 번호판의 텍스트 */
        string str;

        vector<Mat> outputs;

        /* 추출된 텍스트를 OCR를 통해 검사 */
        for(int j = 0; j < textsSize; j++) {
          Plate::Text& text = foundPlate.texts[j];

          OCR* ocr;
          /*
           * 문자 OCR과 숫자 OCR을 할당
           * j 번째를 문자로 설정 - 한글 : { 2 } 영문 : {4 ,5, 6}
           */
          if((j == 4) || (j == 5) || (j == 6))
            ocr = ocrChar;
          else
            ocr = ocrNum;

          Mat canonical = text.canonical(SAMPLESIZE);

          if(mode & OCRTRAIN)
            sample.push_back(canonical);

          if(!(mode & NOTUSEML)) {
            /* 정규화된 Text의 특징 */
            Mat feature = OCR::features(canonical, SAMPLESIZE);

            /* OCR에 의해 분석된 결과 */
            /*Mat output(1, ocr->numCharacters, CV_32FC1);*/
            Mat output;

            ocr->predict(feature, &output);
            str += ocr->classify(output);

            outputs.push_back(output);
          }

          Rect textArea = Rect(SAMPLESIZE * j, 0, SAMPLESIZE, SAMPLESIZE);
          cvtColor(canonical, textCollection(textArea), CV_GRAY2BGR);

          /* 붉은 사각형 hightlight */
          rectangle(textCollection, textArea, red);
        }

        if(info.way == EXIT)
          carList.push_back(goneCar);

        int index_ = OCR::maxProb(outputs, carList);

        if(info.way != ENTER) {
          if(index_ != -1)
            cout << "\t\tThe Largest Problity of Plate Text : " << carList[index_] << endl;
        }

        if(mode & NETWORK) {
          if(info.way != ENTER) {
            if(index_ != -1) {
              if(info.way == EXIT) {
                if((carList.size() - 1) == index_)
                  str = "";
                else {
                  carList.pop_back();
                  // 마지막 꺼 빼기
                  str = carList[index_];
                  goneCar = str;
                }
              } else
                str = carList[index_];
            } else
              str = "";
          }
        }

        if(info.way == NONE) {
          for(int j = 1; j <= SEGMENTSIZE; j++) {
            if((j != zoneIndex) && (table[j].plateStr == str))
              table[j].renew("");
          }
        }

        table[zoneIndex].renew(str);

        printTable(table);

        /* Plate Text 문자열을 image에 넣기 */
        putText(image, str, plate.position, cv::FONT_HERSHEY_SIMPLEX, 1, blue);

        if(mode & PLATESTR)
          cout << "\t\t str = " << str << endl;

        /*
        if (dicider.decide(str)) {
        std::cout << "\t\t\tThe answer is " << str << "  " << rand() % 256 << std::endl;
        }
        */

        if(mode & ANALYSIS)
          analyzer.analyze(str);

        if(mode & WINDOWON)
          imshow("Text", textCollection);

        if(mode & WINDOWON)
          imshow("warp" /*+ to_string(i)*/, foundPlate.img);

        k++;
      }

      cycle_t = (double)getTickCount() - cycle_t;

      if(mode & NETWORK) {
        if(k) {
          send2Server(info, table, ip);
          otime = clock();
          ptime = clock();
        } else if(otime != -1) {
          if((double)(clock() - otime) / CLOCKS_PER_SEC > 6) {
            brcd.close();
            ptime = clock();
            otime = -1;
          }
        }

        if(info.way != ENTER)
          sync(&carList, ip);
      }

      if(mode & COSTTIME)
        cout << "\tCost Time In a Cycle : " << cycle_t * 1000 / getTickFrequency() << "ms" << endl;

      if(mode & OCRTRAIN)
        ocrtrainer.train(sample);

      if(info.way == NONE)
        for(int i = 0; i < SEGMENTSIZE; i++) rectangle(image, area[i], red, 1);

      imshow("image", image);

      if((double)(clock() - ptime) / CLOCKS_PER_SEC > 1)
        brcd.hold();
    }
  });

  camThread.join();
  procThread.join();

  delete ocrChar;
  delete ocrNum;
  delete svm;

  return 0;
}
