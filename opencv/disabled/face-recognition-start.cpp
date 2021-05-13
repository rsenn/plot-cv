#include <raspicam/raspicam_cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <iostream>
#include <fstream>
#include <future>
#include <time.h>
#include <dirent.h>
#include <iomanip>
#include <wiringPi.h>

using namespace std;

// ##### Define the function to return the ultrasonic sensor distance
double
getDistance(int triggerGpioPin, int echoGpioPin) {
  // Initialise the objects
  long startTime, endTime, travelTime;
  double speedOfSound = 340.29; // Metres Per Second at sea level
  double distance = 0.0;

  // Initialise the Trigger pin to LOW and pause for 0.5 seconds
  digitalWrite(triggerGpioPin, LOW);
  delay(500);

  // Triggering the sensor for 10 microseconds
  // Will send out 8 ultrasonic (40kHz) bursts and listen for echos
  digitalWrite(triggerGpioPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerGpioPin, LOW);

  // Waiting for echo start
  while(digitalRead(echoGpioPin) == LOW)
    ;
  startTime = micros();

  // Waiting for echo end
  while(digitalRead(echoGpioPin) == HIGH)
    ;
  endTime = micros();

  // Calculate travel time
  travelTime = endTime - startTime;

  // Calculate distance
  distance = ((travelTime / 1000000.0) * speedOfSound) / 2;

  // Return distance in centimetres
  return distance * 100;
}

// ##### Define the function to load the face detection cascade files
bool
loadCascadeFiles(string openCVCascadePath,
                 cv::CascadeClassifier& faceCascade,
                 cv::CascadeClassifier& eyeCascade,
                 cv::CascadeClassifier& noseCascade) {
  // Set the path to the cascade files
  string faceCascadeFile = openCVCascadePath + "haarcascade_frontalface_default.xml";
  string eyeCascadeFile = openCVCascadePath + "haarcascade_eye.xml";
  string noseCascadeFile = openCVCascadePath + "haarcascade_mcs_nose.xml";

  // Load face cascade
  cout << "Loading face cascade..." << endl;
  if(!faceCascade.load(faceCascadeFile)) {
    cerr << "Error loading face cascade!" << endl;
    return false;
  }

  // Load eye cascade
  cout << "Loading eye cascade..." << endl;
  if(!eyeCascade.load(eyeCascadeFile)) {
    cerr << "Error loading eye cascade!" << endl;
    return false;
  }

  // Load nose cascade
  cout << "Loading nose cascade..." << endl;
  if(!noseCascade.load(noseCascadeFile)) {
    cerr << "Error loading nose cascade!" << endl;
    return false;
  }

  return true;
}

// ##### Define the function to load the face recognizer training images
bool
loadFaceRecognizerTrainingImages(string openCVFaceRecognizerImagesPath,
                                 vector<cv::Mat>& faceRecognizerImages,
                                 vector<int>& faceRecognizerLabels,
                                 vector<string>& faceRecognizerLabelNames) {
  // Initialise variables for loading of training face image files
  DIR* dir;
  dirent* pdir;
  vector<string> faceImageFilenames;
  string faceImageFilename;
  string faceRecognizerLabelName;
  cv::Mat faceRecognizerImageLoad;
  size_t underscorePosition;
  vector<string>::iterator findIterator;
  int index;

  // Read in the face recognizer image filenames
  dir = opendir(openCVFaceRecognizerImagesPath.c_str());
  while((pdir = readdir(dir))) {
    faceImageFilename = pdir->d_name;

    if(faceImageFilename.size() > 4) {
      if(faceImageFilename.substr(faceImageFilename.size() - 4, 4) == ".jpg") {
        faceImageFilenames.push_back(pdir->d_name);
      }
    }
  }

  // Read in the face recognizer images
  for(size_t i = 0; i < faceImageFilenames.size(); i++) {
    faceImageFilename = faceImageFilenames[i];
    cout << "Loading face recognizer images and labels..." << faceImageFilename << endl;

    // Load the face image and add to array
    faceRecognizerImageLoad = cv::imread(openCVFaceRecognizerImagesPath + faceImageFilename, cv::LOAD_IMAGE_GRAYSCALE);
    cv::resize(faceRecognizerImageLoad, faceRecognizerImageLoad, cv::Size(100, 100));
    faceRecognizerImages.push_back(faceRecognizerImageLoad);

    // Find the position of the underscore in the filename and retrieve the label name
    underscorePosition = faceImageFilename.find_first_of("_");
    faceRecognizerLabelName = faceImageFilename.substr(0, underscorePosition);

    // Check if the label name exists in the label names array
    findIterator = find(faceRecognizerLabelNames.begin(), faceRecognizerLabelNames.end(), faceRecognizerLabelName);
    if(findIterator == faceRecognizerLabelNames.end()) {
      faceRecognizerLabelNames.push_back(faceRecognizerLabelName);
      faceRecognizerLabels.push_back(faceRecognizerLabelNames.size());
    } else {
      index = distance(faceRecognizerLabelNames.begin(), findIterator);
      faceRecognizerLabels.push_back(index + 1);
    }
  }

  // Output the face recognizer labels and label names
  for(size_t i = 0; i < faceRecognizerLabels.size(); i++) {
    cout << "Face Recognizer Label..." << faceRecognizerLabels[i] << endl;
  }
  for(size_t i = 0; i < faceRecognizerLabelNames.size(); i++) {
    cout << "Face Recognizer Label Name..." << faceRecognizerLabelNames[i] << endl;
  }

  return true;
}

// ##### Define the method to perform the face detection
void
detectFaces(vector<cv::Mat>& faceROIImages,
            cv::Mat frame,
            cv::CascadeClassifier faceCascade,
            cv::CascadeClassifier eyeCascade,
            cv::CascadeClassifier noseCascade,
            bool trainingMode) {
  // Intialise the function objects
  cv::Mat frameGrey;
  cv::Mat faceROI;
  cv::Mat faceROIGrey;
  vector<cv::Rect> faces;
  vector<cv::Rect> eyes;
  vector<cv::Rect> nose;
  int lineThickness = 1;
  int lineType = cv::LINE_AA; // 8, 4 or cv::LINE_AA

  // Convert frame to grayscale, normalize the brightness, and increase the contrast
  cv::cvtColor(frame, frameGrey, cv::COLOR_BGR2GRAY);
  cv::equalizeHist(frameGrey, frameGrey);

  // Detect any faces
  faceCascade.detectMultiScale(frameGrey, faces, 1.1, 5, cv::CASCADE_SCALE_IMAGE, cv::Size(10, 10), cv::Size(400, 400));

  // Loop through each face
  for(size_t i = 0; i < faces.size(); i++) {
    // Get the face region of interest
    faceROI = frame(faces[i]);
    faceROIGrey = frameGrey(faces[i]);

    // Draw a rectangle around the face
    // cv::Point topLeft(faces[i].x, faces[i].y);
    // cv::Point bottomRight(faces[i].x + faces[i].width, faces[i].y + faces[i].height);
    // cv::rectangle(faceROI, topLeft, bottomRight , cv::Scalar(255, 255, 255), lineThickness,
    // lineType, 0);

    // Detect eyes
    eyeCascade.detectMultiScale(faceROIGrey, eyes, 1.1, 5, cv::CASCADE_SCALE_IMAGE, cv::Size(10, 10), cv::Size(50, 50));

    // Draw an ellipse around each eye
    if(eyes.size() == 2 && !trainingMode) {
      for(size_t j = 0; j < 2; j++) {
        cv::Point center(eyes[j].x + eyes[j].width * 0.5, eyes[j].y + eyes[j].height * 0.5);
        cv::ellipse(faceROI,
                    center,
                    cv::Size(eyes[j].width * 0.5, eyes[j].height * 0.5),
                    0,
                    0,
                    360,
                    cv::Scalar(255, 0, 0),
                    lineThickness,
                    lineType,
                    0);
      }
    }

    // Detect nose
    noseCascade.detectMultiScale(faceROIGrey, nose, 1.1, 5, cv::CASCADE_SCALE_IMAGE, cv::Size(10, 10), cv::Size(200, 200));

    // Draw a rectangle around the nose
    if(nose.size() == 1 && !trainingMode) {
      // cv::Point center(nose[0].x + nose[0].width * 0.5, nose[0].y + nose[0].height * 0.5);
      // cv::ellipse(faceROI, center, cv::Size(nose[0].width * 0.5, nose[0].height * 0.5), 0, 0,
      // 360, cv::Scalar(255, 150, 0), lineThickness, lineType, 0);
      cv::Point topLeft(nose[0].x, nose[0].y);
      cv::Point bottomRight(nose[0].x + nose[0].width, nose[0].y + nose[0].height);
      cv::rectangle(faceROI, topLeft, bottomRight, cv::Scalar(255, 150, 0), lineThickness, lineType, 0);
    }

    // Add to the face ROI images array
    if(eyes.size() == 2 && nose.size() == 1) {
      faceROIImages.push_back(faceROI);
    }
  }

  return;
}

int
main(int argc, char** argv) {
  // Initialise the input parameter mode objects
  bool faceRecognitionMode = false;
  bool trainingMode = false;
  bool showPreview = false; // Set to true to display video window (stop VNC desktop first)
  bool ultrasonicSensor = false;
  string trainingLabel;

  // Initialise the font for display of text
  int font = cv::FONT_HERSHEY_DUPLEX;
  cv::Scalar fontColour = cv::Scalar(255, 255, 255);
  double fontSizeSmall = 0.3;
  double fontSizeLarge = 0.4;
  int fontLineThickness = 1;
  int fontLineType = cv::LINE_AA; // 8, 4 or cv::LINE_AA
  string text;
  cv::Size textSize;
  int textBaseline = 0;
  int textMargin = 10;

  // Initialise the OpenCV directory path
  string openCVPath = "/home/pi/projects/facerecognition/";
  string openCVCascadePath = openCVPath + "data/haarcascades/";
  string openCVFaceRecognizerImagesPath = openCVPath + "data/faceimages/";
  string stopVideoCaptureFilePath = openCVPath + "stop-video-capture.txt";

  // Initialise the camera objects
  raspicam::RaspiCam_Cv camera;
  int cameraWidth = 640;
  int cameraHeight = 480;
  size_t frameCount = 0, startFrameCount = 0, framesPerSecond = 0;
  time_t startTime, endTime;
  cv::Mat frame;
  time_t dateTimestamp;
  char dateTimestampFormatted[20];

  // Initialise the file cascade objects
  cv::CascadeClassifier faceCascade, eyeCascade, noseCascade;

  // Initialise the async future objects
  bool futureFacesRunning = false;
  future<void> futureFaces;
  future_status futureStatus;

  // Initialise the face ROI objects
  vector<cv::Mat> faceROIImages;
  cv::Mat faceROIImage, faceROIImageGrey;
  int faceROIImageWidth = 100;
  int faceROIImageHeight = 100;
  time_t faceROITimestamp;
  char faceROITimestampFormatted[20];
  int imageMargin = 10;
  int lineThickness = 2;
  int lineType = cv::LINE_AA; // 8, 4 or cv::LINE_AA

  // Initialise the face recognizer objects
  string faceImageFilename;
  vector<cv::Mat> faceRecognizerImages;
  vector<int> faceRecognizerLabels;
  vector<string> faceRecognizerLabelNames;
  cv::Ptr<cv::FaceRecognizer> faceRecognizerModel;
  int faceRecognizerPredictionLabel = 0;
  string faceRecognizerPredictionLabelName;
  double faceRecognizerPredictionConfidence = 0.0;

  // Initialise the training objects
  int trainingFaceImageCounter = 0, maxTrainingFaceImages = 10;
  time_t trainingFilenameTimestamp;
  char trainingFilenameTimestampFormatted[20];

  // Initialise the Ultrasonic Sensor objects
  int triggerGpioPin = 23;
  int echoGpioPin = 24;
  double distanceToObject = 0.0;

  // Check the input arguments and set the face recognition mode
  if(argc >= 2) {
    if(string(argv[1]) == "TRUE") {
      faceRecognitionMode = true;
      cout << "Face Recognition Mode...ON" << endl;
    }
    if(argv[2] != NULL) {
      trainingMode = true;
      trainingLabel = string(argv[2]);
      cout << "Training Mode...ON..." << trainingLabel << endl;
    }
  }

  // If preview mode then create a window to display the video
  if(showPreview) {
    cv::namedWindow("Video", 1);
  }

  // Load the face detection cascade files
  loadCascadeFiles(openCVCascadePath, faceCascade, eyeCascade, noseCascade);

  // Load in the face recognizer training images
  loadFaceRecognizerTrainingImages(openCVFaceRecognizerImagesPath,
                                   faceRecognizerImages,
                                   faceRecognizerLabels,
                                   faceRecognizerLabelNames);

  // Create a face recognizer and train it on the given images
  if(faceRecognizerLabelNames.size() >= 2) {
    cout << "Training face recognizer..." << endl;
    try {
      faceRecognizerModel = cv::createFisherFaceRecognizer();
      faceRecognizerModel->train(faceRecognizerImages, faceRecognizerLabels);
    } catch(cv::Exception ex) {
      cerr << "Error training face recognizer!..." << ex.msg << endl;
      return -1;
    }
  }

  // Setup the GPIO wiring pins for the Ultrasonic Sensor
  if(ultrasonicSensor) {
    cout << "Initialising GPIO pins..." << endl;
    wiringPiSetupGpio();
    pinMode(triggerGpioPin, OUTPUT);
    pinMode(echoGpioPin, INPUT);
  }

  // Set camera params
  camera.set(cv::CAP_PROP_FRAME_WIDTH, cameraWidth);
  camera.set(cv::CAP_PROP_FRAME_HEIGHT, cameraHeight);
  camera.set(cv::CAP_PROP_FORMAT, CV_8UC3); // For color
  // camera.set(cv::CAP_PROP_FPS, 10);
  // camera.set(cv::CAP_PROP_BRIGHTNESS, 75); // 0 - 100
  // camera.set(cv::CAP_PROP_CONTRAST, 60); // 0 - 100

  // Open camera
  cout << "Opening camera..." << endl;
  if(!camera.open()) {
    cerr << "Error opening camera!" << endl;
    return -1;
  }
  cout << "Camera opened successfully..." << endl;

  // Start capturing
  for(;; frameCount++) {
    // Set the start time if on the first frame
    if(frameCount == 1) {
      time(&startTime);
    }

    // Set the end time
    time(&endTime);

    // Check if 1 seconds has passed
    if(endTime - startTime == 1) {
      // Calculate the frames per second (FPS)
      framesPerSecond = frameCount - startFrameCount;

      // Reset the start time and start frame counter
      time(&startTime);
      startFrameCount = frameCount;
    }

    // Get the frame from the camera
    camera.grab();
    camera.retrieve(frame);

    // Convert frame to RGB
    cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);

    // Flip the image around both x and y-axis
    cv::flip(frame, frame, -1);

    // Run the face detection if input argument is TRUE
    if(faceRecognitionMode) {
      // Call the detect faces function asynchronously
      if(!futureFacesRunning) {
        // Detect faces
        futureFaces = async(
            launch::async, detectFaces, ref(faceROIImages), frame.clone(), faceCascade, eyeCascade, noseCascade, trainingMode);

        // Set the future faces running flag to true
        futureFacesRunning = true;
      }

      // Get the status of the future
      futureStatus = futureFaces.wait_for(chrono::seconds(0));

      // Check if the future has completed
      if(futureStatus == future_status::ready) {
        // Get the images array from the future
        futureFaces.get();

        // Reset the future faces running flag to false
        futureFacesRunning = false;
      }

      // Check if there are some face ROI images returned
      if(faceROIImages.size() > 0) {
        cout << "Face(s) detected..." << faceROIImages.size() << endl;

        // Retrieve the first face ROI image into the objects and resize
        cv::resize(faceROIImages[0], faceROIImage, cv::Size(faceROIImageWidth, faceROIImageHeight));

        // Get the face ROI timestamp formatted char
        faceROITimestamp = time(NULL);
        strftime(faceROITimestampFormatted,
                 sizeof(faceROITimestampFormatted),
                 "%d/%m/%Y %H:%M:%S",
                 localtime(&faceROITimestamp));

        if(trainingMode && trainingFaceImageCounter < maxTrainingFaceImages) {
          // Add to the training face image counter
          trainingFaceImageCounter += 1;

          // Get the timestamp formatted char
          trainingFilenameTimestamp = time(NULL);
          strftime(trainingFilenameTimestampFormatted,
                   sizeof(trainingFilenameTimestampFormatted),
                   "%d%m%Y%H%M%S",
                   localtime(&trainingFilenameTimestamp));

          // Write the training face image to jpg file
          faceImageFilename = trainingLabel + "_" + trainingFilenameTimestampFormatted + ".jpg";
          cv::imwrite(openCVFaceRecognizerImagesPath + faceImageFilename, faceROIImage);
        }

        // Reset the images array
        faceROIImages.clear();
      }

      // Check if the face ROI image is not empty
      if(!faceROIImage.empty()) {
        // Draw the rectangle background
        cv::Point topLeft(imageMargin, imageMargin);
        cv::Point bottomRight(faceROIImageWidth + imageMargin, faceROIImageHeight + imageMargin);
        cv::rectangle(frame, topLeft, bottomRight, cv::Scalar(255, 255, 255), lineThickness, lineType, 0);

        // Copy the face ROI image into the frame
        faceROIImage.copyTo(frame(cv::Rect(imageMargin, imageMargin, faceROIImage.cols, faceROIImage.rows)));

        // Convert face ROI image to grayscale, normalize the brightness, and increase the contrast
        cv::cvtColor(faceROIImage, faceROIImageGrey, cv::COLOR_BGR2GRAY);
        cv::equalizeHist(faceROIImageGrey, faceROIImageGrey);

        // Use the face recognizer model to perform the face recognition prediction
        if(faceRecognizerLabelNames.size() >= 2) {
          faceRecognizerModel->predict(faceROIImageGrey, faceRecognizerPredictionLabel, faceRecognizerPredictionConfidence);
          faceRecognizerPredictionLabelName = faceRecognizerLabelNames[faceRecognizerPredictionLabel - 1];

          // Add the prediction image name text
          text = "Name: " + faceRecognizerPredictionLabelName;
          textSize = cv::getTextSize(text, font, fontSizeSmall, fontLineThickness, &textBaseline);
          cv::putText(frame,
                      text,
                      cv::Point(textMargin, faceROIImageHeight + imageMargin + textMargin + textSize.height),
                      font,
                      fontSizeSmall,
                      fontColour,
                      fontLineThickness,
                      fontLineType);

          // Add the prediction image confidence text
          text = "Confidence: " + cv::format("%.2f", faceRecognizerPredictionConfidence);
          textSize = cv::getTextSize(text, font, fontSizeSmall, fontLineThickness, &textBaseline);
          cv::putText(frame,
                      text,
                      cv::Point(textMargin, faceROIImageHeight + imageMargin + (textMargin * 2) + (textSize.height * 2)),
                      font,
                      fontSizeSmall,
                      fontColour,
                      fontLineThickness,
                      fontLineType);

          // Add the face ROI timestamp text
          text = "Date: " + string(faceROITimestampFormatted).substr(0, 10);
          textSize = cv::getTextSize(text, font, fontSizeSmall, fontLineThickness, &textBaseline);
          cv::putText(frame,
                      text,
                      cv::Point(textMargin, faceROIImageHeight + imageMargin + (textMargin * 3) + (textSize.height * 3)),
                      font,
                      fontSizeSmall,
                      fontColour,
                      fontLineThickness,
                      fontLineType);

          // Add the face ROI timestamp text
          text = "Time: " + string(faceROITimestampFormatted).substr(11, 8);
          textSize = cv::getTextSize(text, font, fontSizeSmall, fontLineThickness, &textBaseline);
          cv::putText(frame,
                      text,
                      cv::Point(textMargin, faceROIImageHeight + imageMargin + (textMargin * 4) + (textSize.height * 4)),
                      font,
                      fontSizeSmall,
                      fontColour,
                      fontLineThickness,
                      fontLineType);
        }
      }
    }

    // Add the face recognition mode text
    if(faceRecognitionMode) {
      text = "Face Recognition Mode: ON";
      textSize = cv::getTextSize(text, font, fontSizeLarge, fontLineThickness, &textBaseline);
      cv::putText(frame,
                  text,
                  cv::Point(textMargin, cameraHeight - textSize.height - (textMargin * 2)),
                  font,
                  fontSizeLarge,
                  fontColour,
                  fontLineThickness,
                  fontLineType);
    } else {
      text = "Face Recognition Mode: OFF";
      textSize = cv::getTextSize(text, font, fontSizeLarge, fontLineThickness, &textBaseline);
      cv::putText(frame,
                  text,
                  cv::Point(textMargin, cameraHeight - textSize.height - (textMargin * 2)),
                  font,
                  fontSizeLarge,
                  fontColour,
                  fontLineThickness,
                  fontLineType);
    }

    // Add the training mode text
    if(trainingMode) {
      text = "Training Mode: ON (" + trainingLabel + "..." + to_string(trainingFaceImageCounter) + "\\" +
             to_string(maxTrainingFaceImages) + ")";
      textSize = cv::getTextSize(text, font, fontSizeLarge, fontLineThickness, &textBaseline);
      cv::putText(frame,
                  text,
                  cv::Point(textMargin, cameraHeight - textMargin),
                  font,
                  fontSizeLarge,
                  fontColour,
                  fontLineThickness,
                  fontLineType);
    } else {
      text = "Training Mode: OFF";
      textSize = cv::getTextSize(text, font, fontSizeLarge, fontLineThickness, &textBaseline);
      cv::putText(frame,
                  text,
                  cv::Point(textMargin, cameraHeight - textMargin),
                  font,
                  fontSizeLarge,
                  fontColour,
                  fontLineThickness,
                  fontLineType);
    }

    // Add the frames per second text
    text = "FPS: " + to_string(framesPerSecond);
    textSize = cv::getTextSize(text, font, fontSizeLarge, fontLineThickness, &textBaseline);
    cv::putText(frame,
                text,
                cv::Point(cameraWidth - textSize.width - textMargin, cameraHeight - textSize.height - (textMargin * 2)),
                font,
                fontSizeLarge,
                fontColour,
                fontLineThickness,
                fontLineType);

    // Add the date and timestamp text
    dateTimestamp = time(NULL);
    strftime(dateTimestampFormatted, sizeof(dateTimestampFormatted), "%d/%m/%Y %H:%M:%S", localtime(&dateTimestamp));
    text = dateTimestampFormatted;
    textSize = cv::getTextSize(text, font, fontSizeLarge, fontLineThickness, &textBaseline);
    cv::putText(frame,
                text,
                cv::Point(cameraWidth - textSize.width - textMargin, cameraHeight - textMargin),
                font,
                fontSizeLarge,
                fontColour,
                fontLineThickness,
                fontLineType);

    // Check if the ultrasonic distance is enabled
    if(ultrasonicSensor) {

      // Get the ultrasonic distance
      if(frameCount % 150 == 0) {
        distanceToObject = getDistance(triggerGpioPin, echoGpioPin);
      }

      // Add the distance text
      text = "Distance: " + cv::format("%.2f", distanceToObject) + "cm";
      textSize = cv::getTextSize(text, font, fontSizeLarge, fontLineThickness, &textBaseline);
      cv::putText(frame,
                  text,
                  cv::Point(cameraWidth - textSize.width - textMargin, textSize.height + textMargin),
                  font,
                  fontSizeLarge,
                  fontColour,
                  fontLineThickness,
                  fontLineType);
    }

    // Write the image file or display the frame
    if(showPreview) {
      // Display the frame in the window
      cv::imshow("Video", frame);
    } else {
      // Write the image to jpg file
      cv::imwrite(openCVPath + "image.jpg", frame);
    }

    // Check if the stop file exists
    if(ifstream(stopVideoCaptureFilePath)) {
      // Delete the stop file
      remove(stopVideoCaptureFilePath.c_str());

      // Stop the loop
      break;
    }
  }

  // Clean up the objects
  cout << "Stopping camera..." << endl;
  camera.release();

  return 0;
}
