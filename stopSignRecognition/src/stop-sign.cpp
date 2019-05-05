/*
 * Copyright (C) 2019  Christian Berger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

#include "opencv2/core.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/imgcodecs.hpp"

#include "opencv2/objdetect/objdetect.hpp"
#include <time.h> // to calculate time needed
#include <limits.h> // to get INT_MAX, to protect against overflow

#include <opencv2/videoio.hpp>
#include <cstring>
#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <stdio.h>

#include <ctime>
#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>


using namespace std;
using namespace cv;
using namespace cluon;

void detectAndDisplayStopSign( Mat frame, OD4Session *od4);

//defining variables for stop sign
String stopSignCascadeName;
CascadeClassifier stopSignCascade;
bool stopSignPresent = false;
const int lookBackNoOfFrames = 20;
int NO_OF_STOPSIGNS_REQUIRED = 5;
int currentIndex = 0;
bool seenFrameStopsigns[lookBackNoOfFrames] = {false};

int32_t main(int32_t argc, char **argv) {
    int32_t retCode{1};
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);

    if ( (0 == commandlineArguments.count("cid")) ||
        (0 == commandlineArguments.count("name")) ||
        (0 == commandlineArguments.count("width")) ||
        (0 == commandlineArguments.count("height")) ) {
        std::cerr << argv[0] << " attaches to a shared memory area containing an ARGB image." << std::endl;

        std::cerr << "Usage:   " << argv[0] << " --cid=<OD4 session> --name=<name of shared memory area> [--verbose]" << std::endl;
        std::cerr << "         --cid:    CID of the OD4Session to send and receive messages" << std::endl;
        std::cerr << "         --name:   name of the shared memory area to attach" << std::endl;
        std::cerr << "         --width:  width of the frame" << std::endl;
        std::cerr << "         --height: height of the frame" << std::endl;
        std::cerr << "Example: " << argv[0] << " --cid=112 --name=img.i420 --width=640 --height=480" << std::endl;
    }
    else {
        const std::string NAME{commandlineArguments["name"]};
        const uint32_t WIDTH{static_cast<uint32_t>(std::stoi(commandlineArguments["width"]))};
        const uint32_t HEIGHT{static_cast<uint32_t>(std::stoi(commandlineArguments["height"]))};
        const bool VERBOSE{commandlineArguments.count("verbose") != 0};

        // Attach to the shared memory.
        std::unique_ptr<cluon::SharedMemory> sharedMemory{new cluon::SharedMemory{NAME}};
        if (sharedMemory && sharedMemory->valid()) {
            std::clog << argv[0] << ": Attached to shared memory '" << sharedMemory->name() << " (" << sharedMemory->size() << " bytes)." << std::endl;



            //Loading the haar cascade
            //"../stopSignClassifier.xml" because the build file is in another folder, necessary to build for testing
            stopSignCascadeName = "/usr/bin/stopSignClassifier.xml";
            if(!stopSignCascade.load(stopSignCascadeName)) {
               printf("--(!)Error loading stopsign cascade\n");
               return -1;
            };

            // Interface to a running OpenDaVINCI session; here, you can send and receive messages.
            cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

            // //Receving messages
            // auto onStopCar  { [&od4, VERBOSE](cluon::data::Envelope &&envelope) {
            //       auto msg = cluon::extractMessage<StopCarRequest>(std::move(envelope));
            //       float amount = msg.amount(); // Get the amount
            //       if (VERBOSE) {
            //          std::cout << "Received stop request message: " << amount << std::endl;
            //       }
            //    }
            // };
            // od4.dataTrigger(StopCarRequest::ID(), onStopCar);

            // Endless loop; end the program by pressing Ctrl-C.
         while (od4.isRunning()) {
             Mat frame;
             Mat frame_HSV;
             Mat frame_gray;
             Mat cropped_frame;

             // Wait for a notification of a new frame.
             sharedMemory->wait();

             // Lock the shared memory.
             sharedMemory->lock();
             {
                 // Copy image into cvMat structure.
                 // Be aware of that any code between lock/unlock is blocking
                 // the camera to provide the next frame. Thus, any
                 // computationally heavy algorithms should be placed outside
                 // lock/unlock.
                 cv::Mat wrapped(HEIGHT, WIDTH, CV_8UC4, sharedMemory->data());
                 frame = wrapped.clone();
             }
             sharedMemory->unlock();

             frame(Rect(Point(100, 150), Point(580, 400))).copyTo(cropped_frame);
             // Method for detecting stop sign with haar cascade
             detectAndDisplayStopSign(frame , &od4);

             // Display image.
            if (VERBOSE) {
               cv::waitKey(1);
            }

         }
      }
     retCode = 0;
   }
   return retCode;
}

//If there is a stop sing in the current frame then it returns a boolean weather 
//
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
void detectAndDisplayStopSign( Mat frame, OD4Session *od4)
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
            stopSignArea += stopsigns[i].width * stopsigns[i].height;
        }

        //It compares the previous state with the current one and it reports it if there is a change of state
            bool valueToReport = insertCurrentFrameStopSign(stopSignArea > 200);
            if(stopSignPresent != valueToReport){
                stopSignPresent = valueToReport;
                stopSignPresenceUpdate.stopSignPresence(valueToReport);
                if(valueToReport) {
                    std::cout << "sending stop sign detected message: " << std::endl;
                } else {
                    std::cout << "sending NO stop sign present message: " << std::endl;
                    od4->send(stopSignPresenceUpdate);
                }
               // od4->send(stopSignPresenceUpdate);
            }
    // -- Opens a new window with the Stop sign recognition on
   // imshow( "stopSign", frame );
}
