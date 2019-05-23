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


void detectAndDisplayBananas( Mat frame);

//defining variables for stop sign
String bananaCascadeName;
CascadeClassifier bananaCascadeClassifier;

bool bananaPresent = false;
const int lookBackNoOfFrames = 10;
int NO_OF_BANANAS_REQUIRED = 6;
int currentIndex = 0;
bool seenFrameBanana[lookBackNoOfFrames] = {false};

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
   //classifier trained by ourseves using this youtube tutorial as guidance https://www.youtube.com/watch?time_continue=203&v=WEzm7L5zoZE
   //The pictures taken for the classifier where from: https://github.com/cfizette/road-sign-cascades
   bananaCascadeName = "../yieldsign.xml";
   if(!bananaCascadeClassifier.load(bananaCascadeName)){printf("--(!)Error loading stopsign cascade\n"); return -1; };

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
     detectAndDisplayBananas(frame);


      int key = (char) waitKey(30);
      if ( key == 'q' || key == 27 ) {
          break;
      }
   }
    return 0;
}


bool insertCurrentFrameBanana(bool bananaCurrentFrame) {

        seenFrameBanana[currentIndex] = bananaCurrentFrame;
        currentIndex++;
        if(currentIndex >= lookBackNoOfFrames) {
            //Because we don't wanna go outside of the array.
            currentIndex = 0;
        }
        
        int noOfFramesWithBanana = 0; 
        //Loop over the array and collect all the trues.
        for(int i = 0; i < lookBackNoOfFrames; i++) {
            if(seenFrameBanana[i]) {
                noOfFramesWithBanana++;
            }
        }
        if(noOfFramesWithBanana < NO_OF_BANANAS_REQUIRED) {
            return false;
        }
        else {
            return true;
        }
}

//Haar cascade for cars copied and modified from
//https://docs.opencv.org/3.4.1/db/d28/tutorial_cascade_classifier.html

void detectAndDisplayBananas( Mat frame)
{
    //Sending messages for banana detection
    BananaPresenceUpdate bananaPresenceUpdate;

    std::vector<Rect> bananas;
    Mat frame_gray;
    cvtColor( frame, frame_gray, COLOR_BGR2GRAY );
    equalizeHist( frame_gray, frame_gray );
    //-- Detect bananas
    bananaCascadeClassifier.detectMultiScale(frame_gray, bananas, 1.1, 2, 0|CASCADE_SCALE_IMAGE, Size(60, 60));
    //checks if the bananas is present in the current frame
    
        float bananaArea = 0;
        for (size_t i = 0; i < bananas.size(); i++)
        {
            Point center( bananas[i].x + bananas[i].width/2, bananas[i].y + bananas[i].height/2 );
            //Draw a circle when recognized
            ellipse( frame, center, Size( bananas[i].width/2, bananas[i].height/2 ), 0, 0, 360, Scalar( 0, 0, 255 ), 4, 8, 0 );
            Mat faceROI = frame_gray( bananas[i] );
            bananaArea = bananas[i].width * bananas[i].height;
        }

        //It compares the previous state with the current one and it reports it if there is a change of state
            bool valueToReport = insertCurrentFrameBanana(bananaArea > 3000);
            if(bananaPresent != valueToReport){
                bananaPresent = valueToReport;
                bananaPresenceUpdate.bananaPresence(valueToReport);
                if(valueToReport) {
                    std::cout << "There is a banana, Don't turn right! " << std::endl;
                    //od4->send(bananaPresenceUpdate);
                } else {
                    std::cout << "There are NO bananas anymore " << std::endl;
                }
                // maybe say something here like "no bananas are being seen yet!
            }
    // -- Opens a new window with the banana recognition on
    imshow( "bananas", frame );
}
