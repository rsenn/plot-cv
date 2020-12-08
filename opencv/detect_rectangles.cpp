#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
// #include <string>

using namespace cv;
using namespace std;

Mat source, source_gray, debug_image;
int width = 1, height = 1, thin = 0, total_rectangles = 0, thresh = 200;
Rect zone(0, 0, 0, 0);
vector<Rect> rectangles;
bool debug_mode = false, count_black = false;

Mat createBinaryImage(Mat, Mat);
Mat keepLines(string);
Rect reduceRectangleSelection(Rect);
Rect buildZone(const char*);
string intToString(int);
vector<Rect> findRectangles(Mat);
void assignArgs(char*);
void assignSource(string);
void drawDebugImage(int, Rect, vector<vector<Point>>, int, vector<Vec4i>);
void setLabel(Mat&, const std::string, std::vector<Point>&);
void split(const string&, char, vector<string>&);

// ./bin/detect_rectangles [nom de l'image] w=[largeur] h=[hauteur] s=[épaisseur] nb=[nombre de rectangles] t=[threshold]
// ./bin/detect_rectangles mon-image.jpg w=20 h=30 s=3 nb=12 debug=true
int
main(int argc, char* argv[]) {

  // Initialise les parametres
  for(int i = 2; i < argc; i++) {
    assignArgs(argv[i]);
  }

  // Lit l'image source
  assignSource(argv[1]);

  // Créer une image contenant uniquement les lignes horizontales
  Mat horizontal_image = keepLines("horizontal");
  // Créer une image contenant uniquement les lignes verticales
  Mat vertical_image = keepLines("vertical");

  // Combine les 2 images précédentes pour ne garder que les rectangles
  Mat binary_image = createBinaryImage(horizontal_image, vertical_image);

  // Initialize l'image utilisée dans le mode debug
  if(debug_mode)
    debug_image = source.clone();

  // Retourne un tableau des rectangles trouvés
  rectangles = findRectangles(binary_image);

  stringstream return_string;
  int count_rectangles = rectangles.size();
  int compt = 0;

  for(size_t i = 0; i < count_rectangles; i++) {
    compt++;
    Rect rect = rectangles[i];
    rect = reduceRectangleSelection(rect);
    Mat source_check(source, rect);
    cvtColor(source_check, source_check, cv::COLOR_RGB2GRAY);
    Mat checkbox(source_gray, rect);
    threshold(checkbox, checkbox, 180, 255, cv::THRESH_BINARY);
    int total_pixels = checkbox.rows * checkbox.cols;
    int black_pixels = total_pixels - countNonZero(checkbox);
    int absolute_x = zone.tl().x + rect.tl().x;
    int absolute_y = zone.tl().y + rect.tl().y;
    Point absolute(absolute_x, absolute_y);

    if(count_black) {
      if(debug_mode) {
        cout << "[" << compt << "]"
             << "(" << absolute.x << "," << absolute.y << ")->" << black_pixels << endl;
      } else {
        return_string << absolute.x << "," << absolute.y << "," << black_pixels << "|";
      }
    } else {
      if(debug_mode) {
        cout << "[" << compt << "]"
             << "(" << absolute.x << "," << absolute.y << ")" << endl;
      } else {
        return_string << absolute.x << "," << absolute.y << "|";
      }
    }
  }

  if(debug_mode) {
    if(total_rectangles > 0 && count_rectangles != total_rectangles) {
      int missing_rectangles = total_rectangles - count_rectangles;
      cout << "Attention : " << missing_rectangles << " rectangles n'ont pas été trouvé." << endl;
    }
    namedWindow("Contours", cv::WINDOW_NORMAL);
    imshow("Contours", debug_image);
  } else {
    if(total_rectangles == 0) {
      cout << return_string.str() << endl;
    } else if(total_rectangles > 0 && count_rectangles == total_rectangles) {
      cout << return_string.str();
    }
  }

  waitKey(0);
  return (0);
}

Rect
reduceRectangleSelection(Rect rect) {
  return Rect(rect.tl().x + thin, rect.tl().y + thin, rect.width - thin, rect.height - thin);
}

vector<Rect>
findRectangles(Mat binary_image) {
  vector<Rect> boxes;
  vector<vector<Point>> contours;
  vector<Vec4i> hierarchy;

  findContours(binary_image, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE, Point(0, 0));

  double expected_area = width * height;
  double max_area = expected_area + expected_area * 0.2;
  int compt = 0;
  for(int idx = 0; idx >= 0; idx = hierarchy[idx][0]) {
    double area = contourArea(contours[idx]);
    Rect rect = boundingRect(contours[idx]);

    if(hierarchy[idx][3] < 0 && rect.width >= width && rect.height >= height && area <= max_area) {
      compt++;
      rect.width = width;
      rect.height = height;
      boxes.push_back(rect);
      if(debug_mode)
        drawDebugImage(compt, rect, contours, idx, hierarchy);
    }
  }

  return boxes;
}

// Draw each rectangles with its number on debug mode image
void
drawDebugImage(int compt, Rect rect, vector<vector<Point>> contours, int idx, vector<Vec4i> hierarchy) {
  rectangle(debug_image, rect.tl(), rect.br(), Scalar(0, 0, 255), 1, 8, 0);
  string checkbox_label = intToString(compt);
  setLabel(debug_image, checkbox_label, contours[idx]);
}

void
checkDebugMode(const char* arg) {
  string debug_argument;
  if(arg)
    debug_argument = arg;
  if(debug_argument == "-d")
    debug_mode = true;
}

// Combined horizontal and vertical image in order to create the binary image
Mat
createBinaryImage(Mat horizontal, Mat vertical) {
  Mat binary_image = horizontal & vertical;
  threshold(binary_image, binary_image, 75, 255.0, cv::THRESH_BINARY_INV);
  return binary_image;
}

Mat
keepLines(string dimension_type) {
  Mat morph, bin;

  Size size;
  if(dimension_type == "horizontal") {
    size = Size(width, 1);
  } else {
    size = Size(1, height);
  }

  Mat morph_kernel = getStructuringElement(MORPH_RECT, size);
  morphologyEx(source_gray, morph, MORPH_CLOSE, morph_kernel);
  threshold(morph, bin, thresh, 255, cv::THRESH_BINARY);
  // threshold(morph, bin, 100, 255.0, cv::THRESH_BINARY | cv::THRESH_OTSU);

  return bin;
}

void
setLabel(Mat& im, const std::string label, std::vector<Point>& contour) {
  double scale = 0.3;
  int baseline = 0;
  int fontface = FONT_HERSHEY_SIMPLEX;
  int thickness = 1;

  Size text = getTextSize(label, fontface, scale, thickness, &baseline);
  Rect r = boundingRect(contour);

  Point pt(r.x + ((r.width - text.width) / 2), r.y + ((r.height + text.height) / 2));
  rectangle(im, pt + Point(0, baseline), pt + Point(text.width, -text.height), CV_RGB(255, 255, 255), cv::FILLED);
  putText(im, label, pt, fontface, scale, CV_RGB(0, 0, 0), thickness, 8);
}

string
intToString(int number) {
  string result;
  ostringstream convert;
  convert << number;
  result = convert.str();

  return result;
}

void
assignSource(string filename) {
  source = imread(filename);
  if(!source.data) {
    cerr << "Problem loading image !" << endl;
    exit(EXIT_FAILURE);
  }

  if(zone.width != 0 && zone.height != 0) {
    source = source(zone);
  }
  cvtColor(source, source_gray, cv::COLOR_BGR2GRAY);
}

void
assignArgs(char* argv) {
  vector<string> v;
  split(argv, '=', v);

  if(v[0] == "w") {
    sscanf(v[1].c_str(), "%d", &width);
  } else if(v[0] == "h") {
    sscanf(v[1].c_str(), "%d", &height);
  } else if(v[0] == "s") {
    sscanf(v[1].c_str(), "%d", &thin);
  } else if(v[0] == "nb") {
    sscanf(v[1].c_str(), "%d", &total_rectangles);
  } else if(v[0] == "t") {
    sscanf(v[1].c_str(), "%d", &thresh);
  } else if(v[0] == "z" || v[0] == "zone") {
    zone = buildZone(v[1].c_str());
  } else if(v[0] == "black") {
    if(v[1] == "true")
      count_black = true;
  } else if(v[0] == "debug") {
    if(v[1] == "true")
      debug_mode = true;
  }
}

Rect
buildZone(const char* argv) {
  vector<string> v;
  int x = 0, y = 0, w = 0, h = 0;
  split(argv, ',', v);
  sscanf(v[0].c_str(), "%d", &x);
  sscanf(v[1].c_str(), "%d", &y);
  sscanf(v[2].c_str(), "%d", &w);
  sscanf(v[3].c_str(), "%d", &h);

  Rect rect(x, y, w, h);
  return rect;
}

void
split(const string& s, char c, vector<string>& v) {
  string::size_type i = 0;
  string::size_type j = s.find(c);

  while(j != string::npos) {
    v.push_back(s.substr(i, j - i));
    i = ++j;
    j = s.find(c, j);

    if(j == string::npos)
      v.push_back(s.substr(i, s.length()));
  }
}
