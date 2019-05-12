// Code copied from https://github.com/opencv/opencv/blob/master/samples/cpp/squares.cpp
// https://docs.opencv.org/3.1.0/d2/d0a/tutorial_introduction_to_tracker.html

// The "Square Detector" program.
// It loads several images sequentially and tries to find squares in
// each image

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
// #include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>
#include "opencv2/objdetect/objdetect.hpp"
#include <opencv2/dnn.hpp>
#include <cstring>
#include "opencv2/objdetect.hpp"

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

#include <iostream>

using namespace cv;
using namespace std;
using namespace cluon;


void detectAndDisplayCars( Mat frame);

//defining variables for stop sign
String carsCascadeName;
CascadeClassifier carsCascadeClassifier;

bool carPresent = false;
const int lookBackNoOfFrames = 20;
int NO_OF_CARS_REQUIRED = 5;
int currentIndex = 0;
bool seenFrameCar[lookBackNoOfFrames] = {false};

static void help(const char* programName)
{
    cout <<
    "\n Stop Sign Recognition.\n"
    "Call:\n"
    "./" << programName << " [file_name (optional)]\n"
    "Using OpenCV version " << CV_VERSION << "\n" << endl;
}

int main(int argc, char** argv) {

   Mat frame;
   Mat frame_HSV;
   Mat frame_gray;
   Mat cropped_frame;


   //Loading the haar cascade
   //"../cars.xml" because the build file is in another folder, necessary to build for testing
   //classifier trained by ourseves using this youtube tutorial as guidance https://www.youtube.com/watch?time_continue=203&v=WEzm7L5zoZE
   //The pictures taken for the classifier where from: https://github.com/chalmers-revere/opendlv-kiwi-data/tree/master/kiwi_detection
   carsCascadeName = "../cars.xml";
   if(!carsCascadeClassifier.load(carsCascadeName)){printf("--(!)Error loading stopsign cascade\n"); return -1; };

   // Capture the video stream from default or supplied capturing device.
   VideoCapture cap(argc > 1 ? atoi(argv[1]) : 0);

   while (true) {
// get frame from the video
     cap >> frame;
//   roi=selectROI("tracker",frame);
     if(frame.empty()) {
         break;
         help(argv[0]);
     }

// Method for detecting stop sign with haar cascade
     detectAndDisplayCars(frame);

     //Method for detecting cars
    // detectAndDisplayCars(frame);

      int key = (char) waitKey(30);
      if ( key == 'q' || key == 27 ) {
          break;
      }
   }
    return 0;
}


bool insertCurrentFrameCar(bool carCurrentFrame) {

        seenFrameCar[currentIndex] = carCurrentFrame;
        currentIndex++;
        if(currentIndex >= lookBackNoOfFrames) {
            //Because we don't wanna go outside of the array.
            currentIndex = 0;
        }
        
        int noOfFramesWithCars = 0; 
        //Loop over the array and collect all the trues.
        for(int i = 0; i < lookBackNoOfFrames; i++) {
            if(seenFrameCar[i]) {
                noOfFramesWithCars++;
            }
        }
        if(noOfFramesWithCars < NO_OF_CARS_REQUIRED) {
            return false;
        }
        else {
            return true;
        }
}

//Haar cascade for cars copied and modified from
//https://docs.opencv.org/3.4.1/db/d28/tutorial_cascade_classifier.html

void detectAndDisplayCars( Mat frame)
{
    //Sending messages for car detection
    CarPresenceUpdate carPresenceUpdate;

    std::vector<Rect> cars;
    Mat frame_gray;
    cvtColor( frame, frame_gray, COLOR_BGR2GRAY );
    equalizeHist( frame_gray, frame_gray );
    //-- Detect cars
    carsCascadeClassifier.detectMultiScale(frame_gray, cars, 1.1, 2, 0|CASCADE_SCALE_IMAGE, Size(60, 60));
    //checks if the car is present in the current frame
    
        float carArea = 0;
        for (size_t i = 0; i < cars.size(); i++)
        {
            Point center( cars[i].x + cars[i].width/2, cars[i].y + cars[i].height/2 );
            //Draw a circle when recognized
            ellipse( frame, center, Size( cars[i].width/2, cars[i].height/2 ), 0, 0, 360, Scalar( 0, 0, 255 ), 4, 8, 0 );
            Mat faceROI = frame_gray( cars[i] );
            carArea += cars[i].width * cars[i].height;
        }

        //It compares the previous state with the current one and it reports it if there is a change of state
            bool valueToReport = insertCurrentFrameCar(carArea > 200);
            if(carPresent != valueToReport){
                carPresent = valueToReport;
                carPresenceUpdate.carPresence(valueToReport);
                if(valueToReport) {
                    std::cout << "Car detected " << std::endl;
                } else {
                    std::cout << "There are NO cars " << std::endl;
                }
                //od4->send(carPresenceUpdate);
            }
    // -- Opens a new window with the car recognition on
    imshow( "cars", frame );
}
