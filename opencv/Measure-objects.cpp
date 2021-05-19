#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

// using namespace cv;
using namespace std;

int
main() {
  cv::VideoCapture capture = cv::VideoCapture(0); // usb camera
  string window_name[] = {"Obraz z kamery",
                          "Wykrywanie konturow",
                          "Obraz binarny 1",
                          "Obraz binarny 2"}; // source, contour detection, binary 1, binary 2
  cv::Mat frame, img, hsv_img, binary1, binary2;

  // export results to document
  fstream file;
  stringstream stream_to_file;
  string string_to_file;
  file.open("wyniki_pomiaru.txt", ios::out); // results.txt

  // contours
  cv::Mat cont;
  cv::Mat cont2;
  //
  vector<cv::Mat> hsv_split;
  // creating windows
  for(int i = 0; i < 4; i++) cv::namedWindow(window_name[i], cv::WINDOW_AUTOSIZE);
  int mt_seconds = 0;    // messuring time in seconds
  int frame_counter = 0; // frame counter set to 0
  int lowerb = 255, upperb = 255, low = 4, up = 255, odleglosc = 106;
  float szerokosc1 = 0, wysokosc1 = 0, wymiar = 0,
        index = 750, // index; calculated; allow conversion dimensions in ppi to mm
      szerokosc2 = 0, wysokosc2 = 0, wymiar2 = 0;
  cv::createTrackbar("Odleglosc obiektywu od obiektu",
                     window_name[0],
                     &odleglosc, // len-messured item distance
                     1000,
                     NULL);
  cv::createTrackbar("Dolna wartosc progowa 1",
                     window_name[2],
                     &lowerb,
                     255, // down treshold of binary1
                     NULL);
  cv::createTrackbar("Gorna wartosc progowa 1",
                     window_name[2],
                     &upperb,
                     255, // up treshold of binary1
                     NULL);
  cv::createTrackbar("Dolna wartosc progowa 2", window_name[3], &low, 255, NULL); // down treshold of binary1
  cv::createTrackbar("Gorna wartosc progowa 2", window_name[3], &up, 255, NULL);  // up treshold of binary1
  while(cv::waitKey(20) != 27) {                                                  // capturing and analizing 27 fps
    capture >> frame;
    frame_counter++;
    if(frame_counter == 27) { // count 27 frames and increment mt(messuring time)
      frame_counter = 0;
      mt_seconds++;
    }
    frame.copyTo(img);

    // frame preparation to finding contours
    cv::cvtColor(img, hsv_img, cv::COLOR_BGR2HSV);      // rbg to hsv conversion
    cv::split(hsv_img, hsv_split);                      // splitting hsv
    cv::inRange(hsv_split[0], lowerb, upperb, binary1); // edges detection
    cv::inRange(hsv_split[1], low, up, binary2);
    cv::erode(binary1, binary1, cv::Mat());
    cv::dilate(binary1, binary1, cv::Mat());
    cv::blur(binary1, binary1, cv::Size(3, 3));
    cv::erode(binary2, binary2, cv::Mat());
    cv::dilate(binary2, binary2, cv::Mat());
    cv::blur(binary2, binary2, cv::Size(3, 3));

    //***
    vector<vector<cv::Point>> contours1, contours2;
    vector<cv::Point> contours_poly1, contours_poly2;
    cv::Rect boundRect1, boundRect2;
    binary1.copyTo(cont);
    binary2.copyTo(cont2);
    cv::findContours(cont, contours1, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));  // find contours
    cv::findContours(cont2, contours2, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0)); // find contours
    int max1 = 0, i_cont1 = -1, max2 = 0, i_cont2 = -1;
    cv::Mat drawing = cv::Mat::zeros(cont.size(), CV_8UC3);

    for(int i = 0; i < contours1.size(); i++) // creating contour area for rectangles (hsv cv::split 1;binary1)
    {
      if(abs(cv::contourArea(cv::Mat(contours1[i]))) > max1) {
        max1 = abs(cv::contourArea(cv::Mat(contours1[i])));
        i_cont1 = i;
      }
    }
    for(int i = 0; i < contours2.size(); i++) // creating contour area for rectangles (hsv split2; binary2)
    {
      if(abs(cv::contourArea(cv::Mat(contours2[i]))) > max2) {
        max2 = abs(cv::contourArea(cv::Mat(contours2[i])));
        i_cont2 = i;
      }
    }
    if(i_cont1 >= 0) // analise of binary1 (hsv cv::split 1)
    {
      cv::approxPolyDP(cv::Mat(contours1[i_cont1]), contours_poly1, 3, true); // approximation of contours
      boundRect1 = cv::boundingRect(cv::Mat(contours_poly1));                 // creating bounding rectangle
      cv::fillConvexPoly(drawing, contours_poly1, contours_poly1.size());     // filling convex polymoph
      cv::rectangle(img,
                    boundRect1.tl(),
                    boundRect1.br(), // drawing cv::rectangle diagonals and flanks
                    cv::Scalar(125, 125, 125),
                    2,
                    8,
                    0);
      cv::line(img, boundRect1.tl(), boundRect1.br(), cv::Scalar(250, 250, 125), 1, 8, 0);
      cv::line(img,
               cv::Point(boundRect1.x + boundRect1.width, boundRect1.y),
               cv::Point(boundRect1.x, boundRect1.y + boundRect1.height),
               cv::Scalar(250, 250, 125),
               1,
               8,
               0);
      string s; // stream to put in text in window
      stringstream out;
      out << "Srodek: " << boundRect1.x + boundRect1.width / 2 << "x" // information about rectangles' width and height in ppi
          << boundRect1.y + boundRect1.height / 2;
      out << " Szerokosc: " << boundRect1.width << " Wysokosc: " << boundRect1.height;
      szerokosc1 = boundRect1.width; // width
      wysokosc1 = boundRect1.height; // height
      s = out.str();
      cv::putText(img,
                  s,
                  cv::Point(10, 10),
                  cv::FONT_HERSHEY_COMPLEX,
                  0.4, // puting information about rectangles in img window
                  cv::Scalar(20, 40, 80),
                  1,
                  8);
      cv::drawContours(drawing,
                       contours1,
                       i_cont1,
                       cv::Scalar(125, 125, 250),
                       2); // drawing detected contours in drawing window
    }
    if(i_cont2 >= 0) // analise of binary2 (hsv cv::split 2), same as 1st
    {
      cv::approxPolyDP(cv::Mat(contours2[i_cont2]), contours_poly2, 3, true);
      boundRect2 = cv::boundingRect(cv::Mat(contours_poly2));
      cv::fillConvexPoly(drawing, contours_poly2, contours_poly2.size());
      cv::rectangle(img, boundRect2.tl(), boundRect2.br(), cv::Scalar(125, 125, 125), 1, 8, 0);
      cv::line(img, boundRect2.tl(), boundRect2.br(), cv::Scalar(250, 250, 125), 1, 8, 0);
      cv::line(img,
               cv::Point(boundRect2.x + boundRect2.width, boundRect2.y),
               cv::Point(boundRect2.x, boundRect2.y + boundRect2.height),
               cv::Scalar(250, 250, 125),
               1,
               8,
               0);
      string st;
      stringstream out1;
      out1 << "Srodek: " << boundRect2.x + boundRect2.width / 2 << "x" << boundRect2.y + boundRect2.height / 2;
      out1 << " Szerokosc: " << boundRect2.width << " Wysokosc: " << boundRect2.height;
      szerokosc2 = boundRect2.width;
      wysokosc2 = boundRect2.height;
      st = out1.str();
      cv::putText(img, st, cv::Point(10, 40), cv::FONT_HERSHEY_COMPLEX, 0.4, cv::Scalar(20, 40, 80), 1, 8);
      cv::drawContours(drawing, contours2, i_cont2, cv::Scalar(125, 125, 250), 2);
    }
    string st1, st2;
    stringstream out2, out3;

    // converting dimensions in ppi to dimensions in mm
    szerokosc1 = (odleglosc * szerokosc1) / index;
    wysokosc1 = (odleglosc * wysokosc1) / index;
    szerokosc2 = (odleglosc * szerokosc2) / index;
    wysokosc2 = (odleglosc * wysokosc2) / index;
    out2 << "1: " << szerokosc1 << "x" << wysokosc1 << " mm";
    st1 = out2.str();
    cv::putText(img, st1, cv::Point(450, 10), cv::FONT_HERSHEY_COMPLEX, 0.4, cv::Scalar(20, 40, 80), 1, 8);
    out3 << "2: " << szerokosc2 << "x" << wysokosc2 << " mm";
    st2 = out3.str();
    cv::putText(img,
                st2,
                cv::Point(450, 40),
                cv::FONT_HERSHEY_COMPLEX,
                0.4, // puting dimensions in mm in img window
                cv::Scalar(20, 40, 80),
                1,
                8);
    stream_to_file << "Czas: " << mt_seconds << " s. Klatka: " // puting dimensions in mm to stream_to_file
                   << frame_counter << " Obiekt 1: " << szerokosc1 << " x " << wysokosc1 << " mm. Obiekt 2: " << szerokosc2
                   << " x " << wysokosc2 << " mm\n";
    string_to_file = stream_to_file.str();
    stream_to_file.flush();
    cv::imshow(window_name[1], drawing);
    //***

    cv::imshow(window_name[0], img);
    cv::imshow(window_name[2], binary1);
    cv::imshow(window_name[3], binary2);
  }
  capture.release();
  if(file.good()) {
    file << string_to_file << endl; // puting stream_to_file saved in string to document file
    file.close();
  } else
    cout << "blad!";
  return 0;
}
