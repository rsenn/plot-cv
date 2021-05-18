#include </home/pi/surveillance_proj/events.h>

//using namespace cv;
// Event base class member functions

Event::Event(time_t execution_deadline, time_t event_creation_time, string event_name, int priority) {

  this->execution_deadline = execution_deadline;
  this->event_creation_time = event_creation_time;
  this->event_name = event_name;
  this->event_priority = priority;
}

time_t
Event::get_execution_deadline() {

  return this->execution_deadline;
}

time_t
Event::get_event_creation_time() {

  return this->event_creation_time;
}

void
Event::set_eventID(long int input_eventID) {

  this->eventID = input_eventID;
}

long int
Event::get_eventID() {

  return this->eventID;
}

string
Event::get_eventName() {

  return this->event_name;
}

int
Event::get_event_priority() {

  return this->event_priority;
}

// SurveillancePhoto member functions

SurveillancePhoto::SurveillancePhoto(time_t execution_deadline, time_t event_creation_time, string event_name, int priority)
    : Event(execution_deadline, event_creation_time, event_name, priority) {
  cout << "SurveillancePhoto Event successfully created...\n";
}

int
SurveillancePhoto::execute_event(cv::VideoCapture input_cap, int camera_index, string save_directory) {

  if(!input_cap.isOpened()) {
    cout << "Camera at index: " << camera_index << " failed to open...\n";
    cv::waitKey(0);
    return -1;
  } else {
    cv::Mat frame;
    bool read_success = input_cap.cv::read(frame);

    if(!read_success) {
      cout << "cv::Error reading frame from camera: " << camera_index << endl;
      return -1;
    } else {
      save_directory += this->get_eventName();
      save_directory += ".jpg";
      vector<int> compression_params;
      compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
      compression_params.push_back(100);

      bool write_success = cv::imwrite(save_directory, frame, compression_params);

      if(!write_success) {
        cout << "cv::Error saving frame to: " << save_directory << endl;
        return -1;
      }
    }
  }

  return 0;
}

// SurveillanceVideo member functions

SurveillanceVideo::SurveillanceVideo(
    time_t videoLen_s, time_t execution_deadline, time_t event_creation_time, string event_name, int priority)
    : Event(execution_deadline, event_creation_time, event_name, priority) {

  this->videoLen_s = videoLen_s;
  cout << "SurveillanceVideo Event successfully created...\n";
}

int
SurveillanceVideo::execute_event(cv::VideoCapture input_cap, int camera_index, string save_directory) {

  if(!input_cap.isOpened()) {
    cout << "Camera at index: " << camera_index << " failed to open. \n";
    return -1;
  } else {
    double dWidth = input_cap.get(cv::CAP_PROP_FRAME_WIDTH);
    double dHeight = input_cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    cv::Size frameSize(static_cast<int>(dWidth), static_cast<int>(dHeight));
    save_directory += this->get_eventName();
    save_directory += ".avi";
    cv::VideoWriter oVideoWriter(save_directory, CV_FOURCC('P', 'I', 'M', '1'), 20, frameSize, true);

    if(!oVideoWriter.isOpened()) {
      cout << "Failed to initilize cv::VideoWriter at directory: " << save_directory << endl;
      oVideoWriter.release();
      oVideoWriter.~cv::VideoWriter();
      return -1;
    } else {

      cv::Mat frame;
      bool success;
      time_t start_time;
      time_t current_time;
      time(&start_time);
      time(&current_time);

      while(difftime(current_time, start_time) <= this->videoLen_s) {
        time(&current_time);
        success = input_cap.cv::read(frame);

        if(!success) {
          cout << "cv::Error: Failed to cv::read frame from camera at index: " << camera_index << endl;
          oVideoWriter.release();
          oVideoWriter.~cv::VideoWriter();
          return -1;
        } else {
          oVideoWriter.cv::write(frame);
        }
      }

      oVideoWriter.release();
      oVideoWriter.~cv::VideoWriter();
    }
  }

  return 0;
}
