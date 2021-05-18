// Grayson Pike, 2018

/*
    Facial Recognition Lock Software (Main program)
    Runs continuously looking for specific faces, then triggers a servo
    when an authenticated cv::face is recognized.

    To prepare a model for this program, follow the steps in README.md
*/

#include <iostream>
#include <vector>
#include <string>
#include <stdlib.h>
#include <unistd.h>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/face.hpp>
#include <opencv2/imgcodecs.hpp>

#include "face_detect.hpp"
#include "hardware.hpp"
#include "config.hpp"

/*
    Returns model name from command cv::line arguments or exits on failure
*/
std::string
get_model_name(int argc, char* argv[]) {

  std::string model_name;
  if(argc != 2) {
    std::cout << "cv::Error: Correct usage: ./train <model_name>" << std::endl;
    exit(1);
  } else {
    return argv[1];
  }
}

int
main(int argc, char* argv[]) {

  // GPIO input_pin("17", "in");
  // std::cout << input_pin.read_value() << std::endl;

  std::string model_name = get_model_name(argc, argv);

  // Load model
  std::cout << "Loading model..." << std::flush;
  cv::Ptr<cv::face::FaceRecognizer> model = cv::face::LBPHFaceRecognizer::create();
  model->cv::read(std::string(MODEL_DIR) + model_name + ".xml");
  std::cout << "[DONE]" << std::endl;

  cv::VideoCapture camera = get_camera();

  // Loop to detect faces
  bool loop = true;
  while(loop) {

    cv::Mat image;

    // Capture an image
    flush_capture_buffer(camera);
    camera >> image;

    // Convert image to greyscale
    cv::cvtColor(image, image, cv::COLOR_RGB2GRAY);

    // Detect coordinates of a cv::face, if any
    std::vector<cv::Rect> face_regions = detect_faces(image);
    std::cout << "Deteced " << face_regions.size() << " faces." << std::endl;

    // TODO: Make this work with more than one cv::face in the picture
    if(face_regions.size() == 1) {

      // Crop image to cv::face, then resize
      image = image(face_regions[0]);
      cv::resize(image, image, cv::Size(FACE_WIDTH, FACE_HEIGHT), 0, 0, cv::INTER_LANCZOS4);

      // Determine if cv::face is allowed
      int label;
      double confidence;
      model->predict(image, label, confidence);

      std::cout << "Results: " << label << ", " << confidence << std::endl;

      // If a cv::face is recognized and authorized
      if(label != 0 && confidence <= POSITIVE_THRESHOLD) {
        std::cout << "******   Detected allowed cv::face with label: " << label << std::endl;
        // Move the lock to the unlocked position
        system("sudo python servo.py 250");
        // Sleep for 1 second (or 1000000 microseconds) to allow the servo to move
        usleep(1500000u);
        // Move back to the resting position
        system("sudo python servo.py 175");
        // Sleep for WAIT_TIME seconds before relocking the door
        usleep(1500000u * WAIT_TIME);
        // Move the lock to the locked position
        system("sudo python servo.py 100");
        // Sleep for 1 second (or 1000000 microseconds) to allow the servo to move
        usleep(1000000u);
        // Move back to the resting position
        system("sudo python servo.py 175");
      }
    }

    // std::string input;
    // std::cout << "Enter 'q' to quit, or any other key to take another picture: " << std::endl;
    // std::cin >> input;
    // if (input == "q" || input == "Q") {
    //	loop = false;
    //}
  }
}
