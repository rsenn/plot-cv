#include </home/pi/surveillance_proj/configuration_manager.h>

int
Configuration_Manager::init_scheduler() {

  int camera_index;
  VideoCapture camera_uninit;
  string event_save_dir;
  string cascade_path;
  string primary_cascade_name;
  string secondary_cascade_name;
  ifstream config_file("/home/pi/surveillance_proj/configuration.txt");

  if(!config_file.is_open()) {
    cout << "Error: Configuration_Manager::init_scheduler() could not open configuration file. \n";
    return -1;
  } else {
    cout << "Reading default configuration for Configuration_Manager::init_scheduler()... \n";
    getline(config_file, event_save_dir);
    getline(config_file, cascade_path);
    getline(config_file, primary_cascade_name);
    getline(config_file, secondary_cascade_name);

    config_file.close();
    cout << "Default configuration acquired...\n";

    event_save_dir.erase(event_save_dir.size() - 1);
    cascade_path.erase(cascade_path.size() - 1);
    primary_cascade_name.erase(primary_cascade_name.size() - 1);
    secondary_cascade_name.erase(secondary_cascade_name.size() - 1);
  }

  cout << "Input system index of camera to initialize: ";
  cin >> camera_index;
  cout << "\n";

  for(unsigned int i = 0; i < this->camera_indexes_in_use.size(); i++) {
    if(camera_index == camera_indexes_in_use[i]) {
      cout << "Error: Configuration_Manager::init_scheduler() tried to initialize camera that is "
              "already in use. \n";
      return -1;
    }
  }

  cout << "Initializing camera at camera_index=" << camera_index << "...\n";

  if(!camera_uninit.open(camera_index)) {
    cout << "Error: Configuration_Manager::init_scheduler() failed to open camera at camera_index=" << camera_index
         << ".\n";
    cout << "Returning to main menu." << endl;
    return -1;
  } else {
    cout << "Camera at camera_index=" << camera_index << " successfully initiliazed.\n";
  }

  cout << "Beginning initialization of Scheduler...\n";
  Scheduler* new_scheduler = new Scheduler(
      camera_index, event_save_dir, cascade_path, primary_cascade_name, secondary_cascade_name, camera_uninit);
  this->scheduler_list.push_back(new_scheduler);
  camera_indexes_in_use.push_back(camera_index);
  cout << "Creation of Scheduler completed.\n";
  return 0;
}

int
Configuration_Manager::scheduler_manager() {

  int selection;
  Scheduler* selected_scheduler = NULL;

  cout << "Select Camera to configure;\n\n";

  for(int i = 0; i < this->scheduler_list.size(); i++) { cout << "-Scheduler_" << i << endl; }
  cout << "\n Input: ";
  cin >> selection;
  cout << "\n\n";

  if(selection >= 0 && selection < this->scheduler_list.size()) {
    selected_scheduler = this->get_scheduler(selection);
    if(selected_scheduler != NULL) {
      cout << "Scheduler_" << selection << " successfully selected.\n";
    }
  } else {
    cout << "Error: Scheduler selection out of range. Returning to menu.\n";
    return 0;
  }

  while(1) {
    char choice;
    cout << "[1] Configure Scheduler \n";
    cout << "[2] Event Manager \n";
    cout << "[3] Image Processor Manager \n";
    cout << "[4] Reponse Module Manager \n";
    cout << "[5] Back \n";
    cout << "Input: ";
    cin >> choice;
    cout << "\n\n";

    switch(choice) {
      case('1'): this->configure_scheduler(selected_scheduler); break;

      case('2'): this->configure_events(selected_scheduler); break;

      case('3'): this->configure_image_processor(selected_scheduler); break;

      case('4'): this->configure_response_module(selected_scheduler); break;

      case('5'): return 0;

      default: cout << "Input not recognized, please try again.\n";
    }
  }

  return 0;
}

int
Configuration_Manager::configure_scheduler(Scheduler* input_sched) {

  while(1) {
    char choice;
    string new_save_dir;
    int new_sched_res;

    cout << "Camera Manager \n\n";
    cout << "[1] Print Camera Index \n";
    cout << "[2] Print Directory Where Footage is Saved \n";
    cout << "[3] Set Directory Where Footage is Saved \n";
    cout << "[4] Print Scheduler Resolution \n";
    cout << "[5] Set Scheduler Resolution \n";
    cout << "[6] Back \n";
    cin >> choice;
    cout << "\n\n";

    switch(choice) {
      case('1'): cout << "Camera_Index= " << input_sched->get_camera_index() << "\n"; break;

      case('2'): cout << "Save_Directory= " << input_sched->get_save_directory() << "\n"; break;

      case('3'):
        cout << "Input New Save Directory: ";
        cin >> new_save_dir;
        cout << "\n\n";
        input_sched->set_save_directory(new_save_dir);
        break;

      case('4'): cout << "Scheduler_Resolution= " << input_sched->get_scheduler_resolution() << "\n"; break;

      case('5'):
        cout << "Input New Scheduler Resolution: ";
        cin >> new_sched_res;
        cout << "\n\n";
        input_sched->set_scheduler_resolution(new_sched_res);
        break;

      case('6'): return 0;

      default: cout << "Input not recognized, please try again. \n";
    }
  }
}
int
Configuration_Manager::configure_events(Scheduler* input_sched) {

  while(1) {
    char choice;

    cout << "Event Manager \n\n";
    cout << "[1] Print Event Schedule \n";
    cout << "[2] Create New Event \n";
    cout << "[3] Delete Existing Event \n";
    cout << "[4] Back \n";
    cout << "Input: ";
    cin >> choice;
    cout << "\n\n";

    switch(choice) {
      case('1'): input_sched->print_event_schedule(); break;

      case('2'): input_sched->create_new_event(); break;

      case('3'): input_sched->delete_existing_event(); break;

      case('4'): return 0;

      default: cout << "Input not recognized, please try again. \n";
    }
  }
}

int
Configuration_Manager::configure_image_processor(Scheduler* input_sched) {

  Image_Processor* temp_proc = input_sched->get_image_processor();

  while(1) {
    char choice;
    int cascade_choice;
    string cascade_path;
    cout << "[1] Print Cascade Path \n";
    cout << "[2] Set Cascade Path \n";
    cout << "[3] Print Primary Cascade Name \n";
    cout << "[4] Set Primary Cascade Name \n";
    cout << "[5] Print Secondary Cascade Name \n";
    cout << "[6] Set Secondary Cascade Name \n";
    cout << "[7] Back \n";
    cout << "Input: ";
    cin >> choice;
    cout << "\n\n";

    switch(choice) {
      case('1'): cout << "Cascade_Path= " << temp_proc->cascade_path << endl; break;

      case('2'):
        cout << "Input New Cascade Path: ";
        cin >> cascade_path;
        temp_proc->set_cascade_path(cascade_path);
        break;

      case('3'): cout << "Primary_Cascade_Name= " << temp_proc->cascade_name << endl; break;

      case('4'):
        cout << "Select New Primary Cascade: ";

        for(unsigned int i = 0; i < this->cascade_files_size; i++) {
          cout << "[" << i << "] " << this->cascade_files[i] << endl;
        }
        cout << "Input: ";
        cin >> cascade_choice;
        cout << "\n\n";

        if(cascade_choice >= 0 && cascade_choice < this->cascade_files_size) {
          temp_proc->set_cascade_name(this->cascade_files[cascade_choice]);
        } else {
          cout << "Error: Selection out of range, returning to menu. \n";
        }
        break;

      case('5'): cout << "Secondary_Cascade_Name= " << temp_proc->nested_cascade_name << endl; break;

      case('6'):
        cout << "Select New Secondary Cascade" << endl;

        for(unsigned int i = 0; i < this->cascade_files_size; i++) {
          cout << "[" << i << "] " << this->cascade_files[i] << endl;
        }
        cout << "Input: ";
        cin >> cascade_choice;
        cout << "\n\n";

        if(cascade_choice >= 0 && cascade_choice < this->cascade_files_size) {
          temp_proc->set_nested_cascade_name(this->cascade_files[cascade_choice]);
        } else {
          cout << "Error: Selection out of range, returning to menu. \n";
        }

        break;

      case('7'): return 0;

      default: cout << "Input not recognized, please try again. \n";
    }
  }
  cout << "Image Processor Manager\n";

  return 0;
}

int
Configuration_Manager::configure_response_module(Scheduler* input_sched) {

  Response_Module* temp_resp = input_sched->get_response_module();

  while(1) {

    char menu_choice;
    cout << "\n\n";
    cout << "[1] Get Notification Destination Email Address\n";
    cout << "[2] Set Notification Destination Email Address\n";
    cout << "[3] Main Menu\n";
    cout << "Input: ";
    cin >> menu_choice;
    cout << "\n\n";

    std::string new_dest_email;

    switch(menu_choice) {
      case('1'): cout << temp_resp->get_mail_destination_address() << endl; break;

      case('2'):
        cout << "Input new destination email address: ";
        cin >> new_dest_email;
        temp_resp->set_mail_destination_address(new_dest_email);
        break;

      case('3'): return 0;

      default: cout << "Input not recognized, please try again.\n";
    }
  }

  return 0;
}

int
Configuration_Manager::config_manager_mainloop() {

  while(1) {
    char menu_choice;
    cout << "\n\n";
    cout << "[1] Initialize New Camera\n";
    cout << "[2] Manage Existing Camera\n";
    cout << "[3] Main Menu\n";
    cout << "Input: ";
    cin >> menu_choice;
    cout << "\n\n";

    switch(menu_choice) {
      case('1'): this->init_scheduler(); break;

      case('2'): this->scheduler_manager(); break;

      case('3'): return 0;

      default: cout << "Input not recognized, please try again.\n";
    }
  }
}

Scheduler*
Configuration_Manager::get_scheduler(int scheduler_index) {

  if(scheduler_index < scheduler_list.size() && scheduler_index >= 0) {
    return this->scheduler_list.at(scheduler_index);
  }

  cout << "Configuration_Manager::get_scheduler() tried to access "
          "Configuration_Manager::scheduler_list out of bounds. "
          "\n";
  return NULL;
}

int
Configuration_Manager::get_scheduler_list_size() {

  return this->scheduler_list.size();
}

int
Configuration_Manager::select_camera_feed() {

  char choice;
  int scheduler_index_selection = 0;
  Scheduler* selected_scheduler = NULL;

  while(1) {
    cout << "[1] Display Feed From Single Scheduler \n";
    cout << "[2] Display Feed From All Schedulers \n";
    cout << "[3] Back \n";
    cout << "Input: ";
    cin >> choice;
    cout << "\n\n";

    switch(choice) {
      case('1'):

        if(!(this->get_scheduler_list_size() > 0)) {
          cout << "Error: Configuration_Manager::select_camera_feed() detected 0 active schedulers "
                  "\n";
          return -1;
        } else {
          cout << "Input Scheduler Index: ";
          cin >> scheduler_index_selection;
          cout << "\n\n";
          selected_scheduler = this->get_scheduler(scheduler_index_selection);

          if(selected_scheduler == NULL) {
            cout << "Configuration_Manager::select_camera_feed() failed to access scheduler_"
                 << scheduler_index_selection << endl;
            return -1;
          } else {
            selected_scheduler->display_camera_feed();
            return 0;
          }
        }

        break;

      case('2'): this->display_all_feeds(); return 0;

      case('3'): return 0;

      default: cout << "Input not recognized, please try again. \n";
    }
  }
}

int
Configuration_Manager::display_all_feeds() {

  if(this->get_scheduler_list_size() <= 0) {
    cout << "Error: Configuration_Manager::display_all_feeds() detected 0 active feeds. \n";
    return -1;
  }

  vector<VideoCapture*> cameras;
  vector<String> windows;
  string window_name_temp;
  Scheduler* scheduler_temp;
  Mat frame_temp;

  for(unsigned int i = 0; i < this->scheduler_list.size(); i++) { // Gather camera info
    scheduler_temp = this->get_scheduler(i);
    cameras.push_back(scheduler_temp->get_camera());
    window_name_temp = "Feed From Camera_" + scheduler_temp->get_camera_index();
    windows.push_back(window_name_temp);
    namedWindow(window_name_temp, cv::WINDOW_AUTOSIZE);
  }

  while(1) {                                           // Read frames and update windows until exit signal
    for(unsigned int i = 0; i < cameras.size(); i++) { // Read frame from each camera and update it's window
      if(!cameras.at(i)->read(frame_temp)) {
        cout << "Error: Configuration_Manager::display_all_feeds() could not read frame from "
                "Scheduler_"
             << i << "'s camera. \n";
        return -1;
      } else {
        imshow(windows.at(i), frame_temp);
      }
    }

    if(waitKey(30) == 27) {
      cout << "Exiting camera feeds. Returning to main menu. \n";
      destroyAllWindows();
      return 0;
    }
  }

  return 0;
}
