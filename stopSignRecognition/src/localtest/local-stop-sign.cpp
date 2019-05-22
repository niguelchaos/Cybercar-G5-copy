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

//defining variables for stop sign
String stopSignCascadeName;
CascadeClassifier stopSignCascade;


bool stopSignPresent = false;
const int lookBackNoOfFrames = 7;
int NO_OF_STOPSIGNS_REQUIRED = 5;
int currentIndex = 0;
bool seenFrameStopsigns[lookBackNoOfFrames] = {false};

bool stopSignPresent = false;
const int lookBackNoOfFrames = 8;
int NO_OF_STOPSIGNS_REQUIRED = 5;
int currentIndex = 0;
bool seenFrameStopsigns[lookBackNoOfFrames] = {false};

void detectAndDisplayStopSign( Mat frame );

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
   //"../stopSignClassifier.xml" because the build file is in another folder, necessary to build for testing
   stopSignCascadeName = "../stopSignClassifier.xml";
   if(!stopSignCascade.load(stopSignCascadeName)){printf("--(!)Error loading stopsign cascade\n"); return -1; };


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
     detectAndDisplayStopSign(frame);

      int key = (char) waitKey(30);
      if ( key == 'q' || key == 27 ) {
          break;
      }
   }
    return 0;
}


bool insertCurrentFrameStopSign(bool stopSignCurrentFrame) {

        seenFrameStopsigns[currentIndex] = stopSignCurrentFrame;
        currentIndex++;
        if(currentIndex >= lookBackNoOfFrames) {
            //Because we don't wanna go outside of the array.
            currentIndex = 0;
        }
        
        int noOfFramesWithStopsigns = 0; 
        //Loop over the array and collect all the trues.
        for(int i = 0; i < lookBackNoOfFrames; i++) {
            if(seenFrameStopsigns[i]) {
                noOfFramesWithStopsigns++;
            }
        }
        if(noOfFramesWithStopsigns < NO_OF_STOPSIGNS_REQUIRED) {
            return false;
        }
        else {
            return true;
        }
}

//Haar cascade for Stop sign copied and modified from
//https://docs.opencv.org/3.4.1/db/d28/tutorial_cascade_classifier.html
//Classifier gotten from : https://github.com/markgaynor/stopsigns
void detectAndDisplayStopSign( Mat frame)
{
    //Sending messages for stop sign detection
    StopSignPresenceUpdate stopSignPresenceUpdate;

    std::vector<Rect> stopsigns;
    Mat frame_gray;
    cvtColor( frame, frame_gray, COLOR_BGR2GRAY );
    equalizeHist( frame_gray, frame_gray );
    //-- Detect stop signs
    stopSignCascade.detectMultiScale(frame_gray, stopsigns, 1.1, 2, 0|CASCADE_SCALE_IMAGE, Size(60, 60));
    //checks if the stop sign is present in the current frame
    
        float stopSignArea = 0;
        for (size_t i = 0; i < stopsigns.size(); i++)
        {
            Point center( stopsigns[i].x + stopsigns[i].width/2, stopsigns[i].y + stopsigns[i].height/2 );
            //Draw a circle when recognized
            ellipse( frame, center, Size( stopsigns[i].width/2, stopsigns[i].height/2 ), 0, 0, 360, Scalar( 0, 0, 255 ), 4, 8, 0 );
            Mat faceROI = frame_gray( stopsigns[i] );
            stopSignArea = stopsigns[i].width * stopsigns[i].height;
        }

        //It compares the previous state with the current one and it reports it if there is a change of state
            bool valueToReport = insertCurrentFrameStopSign(stopSignArea > 3500);
            if(stopSignPresent != valueToReport){
                stopSignPresent = valueToReport;
                stopSignPresenceUpdate.stopSignPresence(valueToReport);
                if(valueToReport) {
                    std::cout << "you are currectly seeing a STOP sign " << std::endl;
                } else {
                    std::cout << "sending NO stop sign present message: " << std::endl;
                     // od4->send(stopSignPresenceUpdate);
                }
              
            }
    // -- Opens a new window with the Stop sign recognition on.
    imshow( "stopSign", frame );
}
