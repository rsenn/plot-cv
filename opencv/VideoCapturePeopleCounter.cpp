#include <chrono>
#include <set>

using namespace std::chrono;

#define PERSON_MIN_CONTOUR_AREA 2000

class VideoCapturePeopleCounter {

public:
  Line refLine;

  int peopleWhoEnteredCount = 0;
  int peopleWhoExitedCount = 0;

  VideoCapturePeopleCounterDelegate* delegate;

  VideoCapturePeopleCounter(const string& videoCapturePath) {
    this->backgroundSubstractor = cv::createBackgroundSubtractorMOG2();
    this->videoCapturePath = videoCapturePath;
  }

  ~VideoCapturePeopleCounter() {}

  // setters

  void
  setRefLineY(int y) {
    refLineY = y;
  }

  // member methods

  void
  start() {
    cv::Mat frame;
    cv::VideoCapture videoCapture(videoCapturePath);

    while(videoCapture.isOpened()) {
      if(!videoCapture.read(frame))
        break;
      if(++frameNumber == 1)
        refLine = Line(0, refLineY, frame.cols, refLineY);

      // erase old contours (seen 16 frames ago)
      unregisterPersonIf([&](const Person* p) { return frameNumber - lastFrameWherePersonWasSeen[p] > 16; });

      // and then process the current frame
      processFrame(frame);
    }
  }

protected:
  set<Person*> people;

  Person*
  registerPerson(const std::vector<cv::Point>& contour) {
    time_p now = high_resolution_clock::now();
    Person* person = NULL;

    for(set<Person*>::iterator cv::it = people.begin(); it != people.end(); ++it) {
      if((*cv::it)->hasSimilarContour(contour)) {
        person = *cv::it;
        person->lastSeen = now;
        person->update(contour);
        countIfPersonIsCrossingTheRefLine(person);
        break;
      }
    }

    if(person == NULL) {
      person = new Person(contour);
      person->firstSeen = now;
      person->lastSeen = now;
      people.insert(person);
    }

    lastFrameWherePersonWasSeen[person] = frameNumber;

    return person;
  }

  template<typename F>
  void
  unregisterPersonIf(F predicate) {
    for(set<Person*>::iterator cv::it = people.begin(); it != people.end();) {
      Person* person = *cv::it;
      if(predicate(person)) {
        lastFrameWherePersonWasSeen.erase(person);
        linesCrossedByPerson.erase(person);
        cv::it = people.erase(it);
      } else {
        ++cv::it;
      }
    }
  }

private:
  cv::Ptr<cv::BackgroundSubtractor> backgroundSubstractor;
  string videoCapturePath;

  int refLineY;

  int frameNumber = 0;
  map<const Person*, int> lastFrameWherePersonWasSeen;
  map<const Person*, std::vector<Line>> linesCrossedByPerson;

  void
  countIfPersonIsCrossingTheRefLine(const Person* person) {
    int direction;

    if(isPersonCrossingTheRefLine(person, refLine, &direction)) {
      if(direction == LINE_DIRECTION_UP)
        peopleWhoEnteredCount++;
      else if(direction == LINE_DIRECTION_DOWN)
        peopleWhoExitedCount++;
    }
  }

  bool
  isPersonCrossingTheRefLine(const Person* person, Line cv::line, int* direction = NULL) {
    for(int i = 0; i < linesCrossedByPerson[person].size(); i++) {
      if(cv::line == linesCrossedByPerson[person][i]) {
        return false;
      }
    }

    if(person->trace.size() > 2) {
      for(int i = 0; i < person->trace.size() - 2; i++) {
        if(intersect(person->trace[i], person->trace[i + 1], cv::line.start, cv::line.end)) {
          if(direction != NULL) {
            *direction = person->trace[i].y > cv::line.start.y ? LINE_DIRECTION_UP : LINE_DIRECTION_DOWN;
          }

          linesCrossedByPerson[person].push_back(cv::line);
          return true;
        }
      }
    }

    return false;
  }

  void
  processFrame(const cv::Mat& frame) {
    cv::Mat tempFrame;

    // substract background from frame
    backgroundSubstractor->apply(frame, tempFrame);

    // binarize frame
    cv::threshold(tempFrame, tempFrame, 128, 255, cv::THRESH_BINARY);

    // morph ops
    cv::morphologyEx(tempFrame, tempFrame, cv::MORPH_OPEN, cv::Mat(8, 8, CV_8UC1, cv::Scalar(1)));
    cv::morphologyEx(tempFrame, tempFrame, cv::MORPH_CLOSE, cv::Mat(8, 8, CV_8UC1, cv::Scalar(1)));

    // find contours
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(tempFrame, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // foreach identified person contour
    for(int i = 0; i < contours.size(); ++i) {
      if(cv::contourArea(contours[i]) > PERSON_MIN_CONTOUR_AREA) {
        Person* person = registerPerson(contours[i]);

        if(delegate != NULL) {
          delegate->onFrameWithPersonDetected(frame, tempFrame, person);
        }
      }
    }

    // notify delegate of frame process
    if(delegate != NULL) {
      delegate->onFrameProcess(frame, tempFrame);
    }
  }
};