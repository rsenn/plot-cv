#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
// #include <string>

// using namespace cv;
using namespace std;

cv::Mat source, source_gray, debug_image;
int width = 1, height = 1, thin = 0, total_rectangles = 0, thresh = 200;
cv::Rect zone(0, 0, 0, 0);
vector<cv::Rect> rectangles;
bool debug_mode = false, count_black = false;

cv::Mat createBinaryImage(cv::Mat, cv::Mat);
cv::Mat keepLines(string);
cv::Rect reduceRectangleSelection(cv::Rect);
cv::Rect buildZone(const char*);
string intToString(int);
vector<cv::Rect> findRectangles(cv::Mat);
void assignArgs(char*);
void assignSource(string);
void drawDebugImage(int, cv::Rect, vector<vector<cv::Point>>, int, vector<cv::Vec4i>);
void setLabel(cv::Mat&, const std::string, std::vector<cv::Point>&);
void split(const string&, char, vector<string>&);

// ./bin/detect_rectangles [nom de l'image] w=[largeur] h=[hauteur] s=[épaisseur] nb=[nombre de rectangles]
// t=[cv::threshold]
// ./bin/detect_rectangles mon-image.jpg w=20 h=30 s=3 nb=12 debug=true
int
main(int argc, char* argv[]) {

  // Initialise les parametres
  for(int i = 2; i < argc; i++) { assignArgs(argv[i]); }

  // Lit l'image source
  assignSource(argv[1]);

  // Créer une image contenant uniquement les lignes horizontales
  cv::Mat horizontal_image = keepLines("horizontal");
  // Créer une image contenant uniquement les lignes verticales
  cv::Mat vertical_image = keepLines("vertical");

  // Combine les 2 images précédentes pour ne garder que les rectangles
  cv::Mat binary_image = createBinaryImage(horizontal_image, vertical_image);

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
    cv::Rect rect = rectangles[i];
    rect = reduceRectangleSelection(rect);
    cv::Mat source_check(source, rect);
    cv::cvtColor(source_check, source_check, cv::COLOR_RGB2GRAY);
    cv::Mat checkbox(source_gray, rect);
    cv::threshold(checkbox, checkbox, 180, 255, cv::THRESH_BINARY);
    int total_pixels = checkbox.rows * checkbox.cols;
    int black_pixels = total_pixels - cv::countNonZero(checkbox);
    int absolute_x = zone.tl().x + rect.tl().x;
    int absolute_y = zone.tl().y + rect.tl().y;
    cv::Point absolute(absolute_x, absolute_y);

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
    cv::namedWindow("Contours", cv::WINDOW_NORMAL);
    cv::imshow("Contours", debug_image);
  } else {
    if(total_rectangles == 0) {
      cout << return_string.str() << endl;
    } else if(total_rectangles > 0 && count_rectangles == total_rectangles) {
      cout << return_string.str();
    }
  }

  cv::waitKey(0);
  return (0);
}

cv::Rect
reduceRectangleSelection(cv::Rect rect) {
  return cv::Rect(rect.tl().x + thin, rect.tl().y + thin, rect.width - thin, rect.height - thin);
}

vector<cv::Rect>
findRectangles(cv::Mat binary_image) {
  vector<cv::Rect> boxes;
  vector<vector<cv::Point>> contours;
  vector<cv::Vec4i> hierarchy;

  cv::findContours(binary_image, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE, cv::Point(0, 0));

  double expected_area = width * height;
  double max_area = expected_area + expected_area * 0.2;
  int compt = 0;
  for(int idx = 0; idx >= 0; idx = hierarchy[idx][0]) {
    double area = cv::contourArea(contours[idx]);
    cv::Rect rect = cv::boundingRect(contours[idx]);

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
drawDebugImage(int compt, cv::Rect rect, vector<vector<cv::Point>> contours, int idx, vector<cv::Vec4i> hierarchy) {
  cv::rectangle(debug_image, rect.tl(), rect.br(), cv::Scalar(0, 0, 255), 1, 8, 0);
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
cv::Mat
createBinaryImage(cv::Mat horizontal, cv::Mat vertical) {
  cv::Mat binary_image = horizontal & vertical;
  cv::threshold(binary_image, binary_image, 75, 255.0, cv::THRESH_BINARY_INV);
  return binary_image;
}

cv::Mat
keepLines(string dimension_type) {
  cv::Mat morph, bin;

  cv::Size size;
  if(dimension_type == "horizontal") {
    size = cv::Size(width, 1);
  } else {
    size = cv::Size(1, height);
  }

  cv::Mat morph_kernel = cv::getStructuringElement(cv::MORPH_RECT, size);
  cv::morphologyEx(source_gray, morph, cv::MORPH_CLOSE, morph_kernel);
  cv::threshold(morph, bin, thresh, 255, cv::THRESH_BINARY);
  // cv::threshold(morph, bin, 100, 255.0, cv::THRESH_BINARY | cv::THRESH_OTSU);

  return bin;
}

void
setLabel(cv::Mat& im, const std::string label, std::vector<cv::Point>& contour) {
  double scale = 0.3;
  int baseline = 0;
  int fontface = cv::FONT_HERSHEY_SIMPLEX;
  int thickness = 1;

  cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);
  cv::Rect r = cv::boundingRect(contour);

  cv::Point pt(r.x + ((r.width - text.width) / 2), r.y + ((r.height + text.height) / 2));
  cv::rectangle(im, pt + cv::Point(0, baseline), pt + cv::Point(text.width, -text.height), CV_RGB(255, 255, 255), cv::FILLED);
  cv::putText(im, label, pt, fontface, scale, CV_RGB(0, 0, 0), thickness, 8);
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
  source = cv::imread(filename);
  if(!source.data) {
    cerr << "Problem loading image !" << endl;
    exit(EXIT_FAILURE);
  }

  if(zone.width != 0 && zone.height != 0) {
    source = source(zone);
  }
  cv::cvtColor(source, source_gray, cv::COLOR_BGR2GRAY);
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

cv::Rect
buildZone(const char* argv) {
  vector<string> v;
  int x = 0, y = 0, w = 0, h = 0;
  split(argv, ',', v);
  sscanf(v[0].c_str(), "%d", &x);
  sscanf(v[1].c_str(), "%d", &y);
  sscanf(v[2].c_str(), "%d", &w);
  sscanf(v[3].c_str(), "%d", &h);

  cv::Rect rect(x, y, w, h);
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
