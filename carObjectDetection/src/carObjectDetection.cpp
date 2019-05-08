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

void detectAndDisplayCars( Mat frame, OD4Session *od4);

//defining variables for stop sign
String carsCascadeName;
CascadeClassifier carsCascadeClassifier;

bool carPresent = false;
const int lookBackNoOfFrames = 20;
int NO_OF_CARS_REQUIRED = 5;
int currentIndex = 0;
bool seenFrameCar[lookBackNoOfFrames] = {false};

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
   //"../cars.xml" because the build file is in another folder, necessary to build for testing
   //classifier trained by ourseves using this youtube tutoriastopSignCascadeNamel as guidance https://www.youtube.com/watch?time_continue=203&v=WEzm7L5zoZE
   //The pictures taken for the classifier where from: https://github.com/chalmers-revere/opendlv-kiwi-data/tree/master/kiwi_detection
            carsCascadeName = "/usr/bin/cars.xml";
            if(!carsCascadeClassifier.load(carsCascadeName)) {
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
             detectAndDisplayCars(frame , &od4);

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

void detectAndDisplayCars( Mat frame, OD4Session *od4)
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
                od4->send(carPresenceUpdate);
            }
    // -- Opens a new window with the car recognition on
    imshow( "cars", frame );
}

