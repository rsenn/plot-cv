// shapefinder.h : Include file for standard system include files,
// or project specific include files.

#pragma once

#include <iostream>
#include <map>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <stdlib.h>

using namespace std;
using namespace cv;

void load_arguments(int argc, char* argv[]);
bool detect_shape();
short* get_rgb(string hex_string);
