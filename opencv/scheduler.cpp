#include </home/pi/surveillance_proj/scheduler.h>

Scheduler::Scheduler(int input_camera_index,
                     string save_directory,
                     string cascade_path,
                     string cascade_name,
                     string nested_cascade_name,
                     VideoCapture camera) {

  this->scheduler_resolution = 1000; // 1000ms default execution cycle delay.
  this->camera_index = input_camera_index;
  this->save_directory = save_directory;

  this->camera = camera;
  this->frame_processor = new Image_Processor(cascade_path, cascade_name, nested_cascade_name);
  this->responder = new Response_Module();
  cout << "Scheduler successfully created...\n";
}

list<Event*>*
Scheduler::get_event_schedule() {

  return &this->event_schedule;
}

int
Scheduler::add_event(Event* input_event) {

  bool element_inserted = false;

  for(list<Event*>::const_iterator iterator = event_schedule.begin(), end = event_schedule.end();
      iterator != end;
      iterator++) {
    Event* temp = *iterator;
    if(temp->get_execution_deadline() >= input_event->get_execution_deadline()) {
      continue;
    } else {
      event_schedule.insert(iterator, input_event);
      element_inserted = true;
      num_of_events++;
      input_event->set_eventID(next_eventID);
      next_eventID++;
      break;
    }
  }

  if(!element_inserted) {
    event_schedule.push_back(input_event);
    num_of_events++;
    input_event->set_eventID(next_eventID);
    next_eventID++;
  }

  return 0;
}

int
Scheduler::remove_event(long int eventID) {

  bool element_removed = false;

  for(list<Event*>::const_iterator iterator = event_schedule.begin(), end = event_schedule.end();
      iterator != end;
      iterator++) {
    Event* temp = *iterator;
    if(temp->get_eventID() == eventID) {
      event_schedule.erase(iterator);
      element_removed = true;
      num_of_events--;
      break;
    }
  }

  if(!element_removed) {
    cout << "Error: Attempt to remove Event from Scheduler->event_schedule failed due to no "
            "matching eventID found. \n";
    return -1;
  }

  return 0;
}

int
Scheduler::create_new_event() {

  while(1) {
    Event* temp_event;
    string event_name;
    int event_priority;
    time_t event_creation_time;
    time_t event_execution_deadline;
    time_t video_length_sec;

    char choice;
    cout << "[1] Schedule Video \n";
    cout << "[2] Schedule Photo \n";
    cout << "[3] Back \n";
    cout << "Input: ";
    cin >> choice;
    cout << "\n\n";

    switch(choice) {
      case('1'):
        cout << "Input Event_Name: ";
        cin >> event_name;
        cout << endl;
        cout << "Input Event_Priority(1-10): ";
        cin >> event_priority;
        cout << endl;
        cout << "Input Video Length(sec): ";
        cin >> video_length_sec;
        cout << endl;
        cout << "Input Video Execution Deadline (+int representing number of seconds from current "
                "time): ";
        cin >> event_execution_deadline;
        cout << endl;
        time(&event_creation_time);
        temp_event = new SurveillanceVideo(video_length_sec,
                                           (event_creation_time + event_execution_deadline),
                                           event_creation_time,
                                           event_name,
                                           event_priority);
        this->add_event(temp_event);
        break;

      case('2'):
        cout << "Input Event_Name: ";
        cin >> event_name;
        cout << endl;
        cout << "Input Event_Priority(1-10): ";
        cin >> event_priority;
        cout << endl;
        cout << "Input Photo Execution Deadline (+int representing number of seconds from current "
                "time): ";
        cin >> event_execution_deadline;
        cout << endl;
        time(&event_creation_time);
        temp_event = new SurveillancePhoto((event_creation_time + event_execution_deadline),
                                           event_creation_time,
                                           event_name,
                                           event_priority);
        this->add_event(temp_event);
        break;

      case('3'): return 0;

      default: cout << "Input not recognized, please try again. \n";
    }
  }
}

int
Scheduler::delete_existing_event() {

  long int event_id;

  cout << "Input Event_ID of Event you want to delete: ";
  cin >> event_id;
  cout << "\n\n";
  this->remove_event(event_id);

  return 0;
}

void
Scheduler::print_event_schedule() {

  int event_num = 0;
  Event* temp = NULL;

  for(list<Event*>::const_iterator iterator = event_schedule.begin(), end = event_schedule.end();
      iterator != end;
      iterator++) {
    temp = *iterator;
    cout << "E#" << event_num << endl;
    cout << "E_ADDR=" << temp << endl;
    cout << "E_ED=" << temp->get_execution_deadline() << endl;
    cout << "E_ID=" << temp->get_eventID() << endl;
    cout << endl;
    event_num++;
  }
}

void
Scheduler::set_scheduler_resolution(int scheduler_resolution) {

  this->scheduler_resolution = scheduler_resolution;
  cout << "scheduler_resolution set to: " << scheduler_resolution << endl;
}

int
Scheduler::get_scheduler_resolution() {

  return this->scheduler_resolution;
}

void
Scheduler::set_camera_index(int input_index) {

  this->camera_index = input_index;
  cout << "camera_index set to: " << input_index << endl;
}

int
Scheduler::get_camera_index() {

  return this->camera_index;
}

void
Scheduler::set_save_directory(string input_save_directory) {

  this->save_directory = input_save_directory;
  cout << "save_directory set to: " << input_save_directory << endl;
}

string
Scheduler::get_save_directory() {

  return this->save_directory;
}

Event* Scheduler::check_overdueEvents() { // Scans event_schedule and returns the first overdue
                                          // Event it finds.
  // Returns NULL is event_schedule is empty or if it finds no overdue Events.
  time_t current_time;
  Event* temp = NULL;

  for(list<Event*>::const_reverse_iterator iterator = event_schedule.rbegin();
      iterator != event_schedule.rend();
      iterator++) {

    time(&current_time);

    if(current_time <= -1) {
      cout << "Error: check_overdueEvents() could not retrieve system time. \n";
      return NULL;
    }

    temp = *iterator; // Couldn't figure out how to access list elements directly from
                      // reverse_iterator so doing this
    // instead...
    if(current_time >= temp->get_execution_deadline()) {
      return temp; // Overdue Event found.
    }
  }

  return NULL; // No overdue Events.
}

int
Scheduler::process_overdueEvents(Event* input_event) {

  time_t current_time;

  if(input_event != NULL) { // If overdue event found -> check if execution is valid
    time(&current_time);

    if((current_time) - (input_event->get_execution_deadline()) > 1800 &&
       input_event->get_event_priority() < 10) { // If Event is >= 30 mins past ED && Event priority
                                                 // < 10, discard without executing.
      this->remove_event(input_event->get_eventID());
      cout << "Discarding [" << input_event->get_eventName()
           << "] with eventID=" << input_event->get_eventID()
           << " due to expiration && priority < 10.\n";

      return 0; // Event not executed.
    } else {    // If Event not expired, execute.
      cout << "Removing event with eventID=" << input_event->get_eventID() << "..." << endl;
      this->remove_event(input_event->get_eventID());
      cout << "Beginning execution of [" << input_event->get_eventName()
           << "] with eventID=" << input_event->get_eventID() << "...\n";
      input_event->execute_event(this->camera, this->camera_index, this->save_directory);
      cout << "Execution of [" << input_event->get_eventName()
           << "] with eventID=" << input_event->get_eventID() << " has completed.\n";

      return 1; // Event executed.
    }
  } else {
    return 0; // Event not executed.
  }
}

Image_Processor*
Scheduler::get_image_processor() {

  return this->frame_processor;
}

Response_Module*
Scheduler::get_response_module() {

  return this->responder;
}

int
Scheduler::display_camera_feed() {

  if(!this->camera.isOpened()) {
    cout << "Error: Scheduler::display_camera_feed() could not access camera at camera_index= "
         << this->camera_index << endl;
    return -1;
  } else {
    Mat frame;
    string window_name = "Camera Feed: camera_index= " + this->camera_index;
    namedWindow(window_name, CV_WINDOW_AUTOSIZE);

    while(1) {
      if(!this->camera.read(frame)) {
        cout << "Error: Scheduler::display_camera_feed() could not read frame from camera at "
                "camera_index= "
             << this->camera_index << endl;
        return -1;
      } else {
        imshow(window_name, frame);
      }

      if(waitKey(30) == 27) {
        cout << "Exiting camera feed. Returning to main menu. \n";
        destroyAllWindows();
        return 0;
      }
    }
  }

  return 0;
}

VideoCapture*
Scheduler::get_camera() {

  return &this->camera;
}

int
Scheduler::scheduler_execution_cycle() {

  Mat frame_temp;

  if(!this->camera.isOpened()) {
    cout << "Camera at index: " << camera_index << " failed to open. \n";
    this->camera.release();
    this->camera.~VideoCapture();
    return -1;
  } else {
    this->camera.read(frame_temp);
  }

  /*Execution stage1: Check event_schedule for overdue events*/
  this->process_overdueEvents(this->check_overdueEvents()); // Check if any events are overdue and
                                                            // pass them to process_overdueEvents().
  /*Execution stage1: complete*/

  // Execution stage2: Run enabled frame analyzing functions
  this->camera.read(frame_temp);

  int colour_analysis_code =
      this->frame_processor->frame_colour_analysis(frame_temp, this->camera_index);
  if(colour_analysis_code == 1) {
    this->responder->set_colour_detected(true);
  }

  secondary_cascade_return_pkg pkg =
      this->frame_processor->frame_humanface_analysis(frame_temp,
                                                      this->camera_index,
                                                      this->save_directory,
                                                      this->camera_index);
  int cascade_detection_code = pkg.ret_val;

  if(cascade_detection_code == 2) {
    this->responder->set_primary_cascade_detected(true);
    this->responder->set_secondary_cascade_detected(true);
  } else if(cascade_detection_code == 1) {
    this->responder->set_primary_cascade_detected(true);
  }
  /*Execution stage2: complete*/

  /* Execution stage3: call responder->execution_cycle() */
  this->responder->response_execution_cycle(pkg.save_dir);
  /* Execution stage3: complete*/

  cout << "Sleeping for " << scheduler_resolution << "ms...\n";
  cout << endl << endl;
  sleep(scheduler_resolution / 1000);
  return 0;
}
