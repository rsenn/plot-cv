#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "simple_svg_1.0.0.hpp"

using namespace cv;
using namespace std;

Mat src; Mat src_gray; Mat resize_img;
int thresh = 100;
int max_thresh = 255;
RNG rng(12345);
int largestContour = -1;
double max_svg_width = 1200; // pixels
double max_svg_height = 900;  // pixels
vector<Point> bigContour;

bool showDiagnostics = false;
bool showWindow = false;

/// Function header
void thresh_callback(int, void* );

//finds the largest contour and stores the bigContour and stores it index which are both global variables.
void filter_contours(vector<vector<Point>> contours_un)
{
    double maxArea = 0.0;
    for (int i =0; i < contours_un.size(); i++)
    {
        double area = contourArea(contours_un[i]);
        if (showDiagnostics)
        {
        std::cout<<"Area: "+ to_string(area) + "Index: " + to_string(i)<<std::endl;
        }
        if (area > maxArea)
        {
            maxArea = area;
            largestContour = i;
        }
    }
      bigContour = contours_un.at(largestContour);
}

void export_svg(vector<Point> contour_arg, string output_file)
{


    svg::Dimensions dimensions(max_svg_width,max_svg_height);
    svg::Document doc(output_file, svg::Layout(dimensions, svg::Layout::TopLeft));



    svg::LineChart chart(5.0);
    svg::Polyline polyline(svg::Stroke(1, svg::Color(255,0,0)));
    for (int i = 0; i < contour_arg.size(); i++)
    {
        svg::Point temp = svg::Point(contour_arg.at(i).x,contour_arg.at(i).y);
        polyline << temp;
    }
    doc << polyline;
    doc.save();

    std::cout<<"EXPORTED SVG"<<std::endl;
}

/** @function main */

//Format: imgfile parameter1 parameter2
int main( int argc, char** argv )
{
    if (argc >2)
    {
  if (std::strcmp(argv[2],"-S") == 0)
  {
      showWindow = true;

  }

  if (argc > 3 && std::strcmp(argv[3], "-D") == 0)
  {
       showDiagnostics = true;
  }
}

  /// Load source image and convert it to gray


  src = imread( argv[1], 1 );

  if(src.empty())
  {
        std::cout<<"failed to open image file. Make sure img exists and command is in correct format."<<std::endl;
  }
  else{

  Size dsize2 = Size(round(.35*src.cols),round(.35*src.rows));
  resize(src,resize_img, dsize2);

  src = resize_img;
  /// Convert image to gray and blur it
  cvtColor( src, src_gray, CV_BGR2GRAY );
  blur( src_gray, src_gray, Size(3,3) );

  /// Create Window
  char* source_window = "Source";
  if (showWindow)
  {
  namedWindow( source_window, CV_WINDOW_AUTOSIZE );
  imshow( source_window, src );
  }
  thresh_callback(0,0);
  export_svg(bigContour, "contour.svg");

  if (showWindow)
  {
  while (cvWaitKey(1) != '\80') {

  }
  }
  }
  return(0);
}

/** @function thresh_callback */
void thresh_callback(int, void* )
{
  Mat canny_output;
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;

  /// Detect edges using canny
  Canny( src_gray, canny_output, thresh, thresh*2, 3 );
  /// Find contours
  findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

  /// Draw contours
  filter_contours(contours);
  if (showDiagnostics)
  {
  std::cout<<"largest Contour Index: "+ to_string(largestContour)<<std::endl;
  std::cout<<"largest Contour Area: "+ to_string(contourArea(bigContour))<<std::endl;
  }
  Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
  Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
  drawContours( drawing, contours, largestContour, color, 2, 8, hierarchy, 0, Point() );

if (showDiagnostics)
{
    for( int i = 0; i< contours.size(); i++ )
         {
           Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
           drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
         }
}


  /// Show in a window
  if (showWindow)
  {
  namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
  imshow( "Contours", drawing );
  }
  }
