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

// parts of Code copied from https://github.com/opencv/opencv/blob/master/samples/cpp/squares.cpp
// https://docs.opencv.org/3.1.0/d2/d0a/tutorial_introduction_to_tracker.html

// The "Square Detector" program.
// It loads several images sequentially and tries to find squares in
// each image



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

// static double angle( Point pt1, Point pt2, Point pt0 );
// static void findSquares( const Mat& image, vector<vector<Point> >& squares );
// static Mat drawSquares( Mat& image, const vector<vector<Point> >& squares, vector<Rect> &boundRects, OD4Session *od4);
void findCars(Mat &frame, vector<Rect>& foundCars, CascadeClassifier carsCascadeClassifier);

void removeCarFromQueue( vector<Point> &initial_car_positions, int *cars_in_queue, int *car_leave_timeout_counter);
void checkCarPosition(OD4Session *od4, double *prev_centerX, double *prev_centerY, double centerX, double centerY,
   double *prev_area, double area, bool *stop_line_arrived, bool *stop_line_arrived_trigger, bool *left_car_is_12oclock_car,
   vector<Point> &initial_car_positions, int *cars_in_queue, int *car_leave_timeout_counter);
void countCars(Mat frame, vector<Point> &initial_car_positions , int *cars_in_queue, bool *stop_line_arrived);
void detectCars(
   OD4Session *od4, Mat& image, vector<Rect> &foundCars,
   double *prev_area, double *prev_centerX, double *prev_centerY,
   int *cars_in_queue, int *car_leave_timeout_counter, bool *stop_line_arrived, bool *stop_line_arrived_trigger,
   vector<Point> &initial_car_positions, bool *left_car_is_12oclock_car);
void BrightnessAndContrastAuto(const cv::Mat &src, cv::Mat &dst, float clipHistPercent);

int32_t main(int32_t argc, char **argv) {
   int32_t retCode{1};
   auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
   if((0 == commandlineArguments.count("cid")) ||
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
   } else {
      const std::string NAME{commandlineArguments["name"]};
      const uint32_t WIDTH{static_cast<uint32_t>(std::stoi(commandlineArguments["width"]))};
      const uint32_t HEIGHT{static_cast<uint32_t>(std::stoi(commandlineArguments["height"]))};
      const bool VERBOSE{commandlineArguments.count("verbose") != 0};

      // Attach to the shared memory.
      std::unique_ptr<cluon::SharedMemory> sharedMemory{new cluon::SharedMemory{NAME}};
      if (sharedMemory && sharedMemory->valid()) {
         std::clog << argv[0] << ": Attached to shared memory '" << sharedMemory->name() << " (" << sharedMemory->size() << " bytes)." << std::endl;

         // Interface to a running OpenDaVINCI session; here, you can send and receive messages.
         cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

         String carsCascadeName;
         CascadeClassifier carsCascadeClassifier;

         // XML trained by Group 8. Permission Given by Group 8 and Student TAs.
         // == for local testing ==
         // carsCascadeName = "../src/car-28-stages.xml";

         carsCascadeName = "/usr/bin/car-28-stages.xml";
         if(!carsCascadeClassifier.load(carsCascadeName)) {
            printf("--(!)Error loading car cascade xml \n");
            return -1;
         };

         // Measure beginning time
         int64_t starttimestampmicro = cluon::time::toMicroseconds(cluon::time::now());
         int64_t starttimestampsecs = starttimestampmicro / 1000000;
         cout << "Starting Timestamp: " << starttimestampsecs << endl;

         // Stupid warnings say it needs to be initialized so here you go compiler stop complaining
         int64_t prevtimestampsecs = 0;
         int framecounter = 0;

         double prev_area = 0; // used to determine whether car is moving and amount of acceleration
         double prev_centerX = 0; // used to track cars
         double prev_centerY = 0;

         int cars_in_queue = 0; // keeps track of the highest amount of cars
         int car_leave_timeout_counter = 0; // prevents counting 1 car leaving as 2
         vector<Point> initial_car_positions {Point(0,0), Point(0,0), Point(0,0)}; // { left | mid | right } respectively

         bool stop_line_arrived = false;
         bool stop_line_arrived_trigger = false;
         int stop_line_arrived_trigger_counter = 4; // sets stop_line_arrived to true when 0 - makes sure car is stopped.

         // needs to know when we are close
         bool left_car_is_12oclock_car = false;

         bool leading_car_gone = false; // know when to start looking for cars
         bool yeet_sent = false; // used to know when to stop looking for cars

         // sensor values for detecting leaving cars
         const float MINFRONTDIST = 0.1f;
         // const float MAXFRONTDIST = 0.8f;
         const float LEFTINTERSECTFRONTDIST = 0.4f;

         const float MINLEFTDIST = 0.05f;
         const float MAXLEFTDIST = 0.4f;

         // Listen for when the car has arrived at stop line
         auto onStopCar {
            [&od4, &stop_line_arrived, &left_car_is_12oclock_car, &leading_car_gone, &stop_line_arrived_trigger, &yeet_sent]
            (cluon::data::Envelope &&envelope) {

               auto msg = cluon::extractMessage<StopSignPresenceUpdate>(std::move(envelope));
               bool stopSignPresence = msg.stopSignPresence(); // Get the bool
               if (stopSignPresence == false) {
                  if (leading_car_gone == false) { // if leading car not gone, then stopsign shouldnt be triggered
                     cout << "Stop sign message received, but leading car has not left yet." << endl;
                  }
                  if (leading_car_gone == true && yeet_sent == false) { // yeet_sent = safe to go
                     cout << "   [ We have arrived at the stop line, waiting 4 seconds. ] " << endl;
                     stop_line_arrived_trigger = true; // make sure car is stopped.
                     left_car_is_12oclock_car = true;
                  }
               }
            }
         };
         od4.dataTrigger(StopSignPresenceUpdate::ID(), onStopCar);

         // sensors are used here to detect leaving cars
         float currentDistance{0.0};
         auto onDistanceReadingAtStopLine {
            [&od4, &stop_line_arrived, &currentDistance, &initial_car_positions,
            &cars_in_queue, &car_leave_timeout_counter, &yeet_sent,
            MINFRONTDIST, LEFTINTERSECTFRONTDIST, MINLEFTDIST, MAXLEFTDIST]
            (cluon::data::Envelope &&envelope) {
               auto msg = cluon::extractMessage<opendlv::proxy::DistanceReading>(std::move(envelope));
      			// senderStamp 0 corresponds to front ultra-sound distance sensor
      	      const uint16_t senderStamp = envelope.senderStamp();
      	      currentDistance = msg.distance(); // Get the distance

               if (yeet_sent == false && stop_line_arrived == true && car_leave_timeout_counter == 0)  {

                  if(senderStamp == 0) { // front sensor
                     if (currentDistance > MINFRONTDIST && currentDistance < LEFTINTERSECTFRONTDIST) {
                        cout << "    Sensor Detection    || Car passed by, either going right or straight: " << currentDistance << endl;
                        removeCarFromQueue(initial_car_positions, &cars_in_queue, &car_leave_timeout_counter);
                     }
                  }

                  if (senderStamp == 1) {
                     if (currentDistance > MINLEFTDIST && currentDistance < MAXLEFTDIST) {
                        cout << "    Sensor Detection     /// Car has left intersection on our lane: " << currentDistance << endl;
                        removeCarFromQueue(initial_car_positions, &cars_in_queue, &car_leave_timeout_counter);
                     }
                  }

               }
            }
         };
         od4.dataTrigger(opendlv::proxy::DistanceReading::ID(), onDistanceReadingAtStopLine);

         // start detecting other cars when approaching stop line
      	auto onCarOutOfSight {
            [&od4, VERBOSE, &leading_car_gone, &stop_line_arrived, &yeet_sent, &stop_line_arrived_trigger](cluon::data::Envelope &&envelope) {

      		   auto msg = cluon::extractMessage<CarOutOfSight>(std::move(envelope));

               if (leading_car_gone == true && stop_line_arrived == true && yeet_sent == true) {
                  cout << "      Scenario is over, resetting. " << endl;
                  stop_line_arrived = false;
                  yeet_sent = false;
                  leading_car_gone = true;
                  stop_line_arrived_trigger = 4;
                  cout << "     [ Leading Car out of sight ]  " << endl;
               }

               if (stop_line_arrived == false) {
                  cout << "     [ Leading Car out of sight ]  " << endl;
                  leading_car_gone = true;
               }
               if (leading_car_gone == false) {
                  if (stop_line_arrived == true) { // should not be at stop line if leading car gone (that is, message should not repeat)
                     cout << "   Already stopped at line, leading car should already have left. Resetting stop line to false." << endl;
                     stop_line_arrived = false;
                  }
               }
      		}
      	};
      	od4.dataTrigger(CarOutOfSight::ID(), onCarOutOfSight);

         // Endless loop; end the program by pressing Ctrl-C.
         while (od4.isRunning()) {
            Mat frame;
            Mat frame_HSV;
            Mat frame_gray;
            Mat cropped_frame;
            Mat brightened_frame;
            Mat frame_threshold;
            Mat final_frame;

            vector<Rect> foundCars;

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
            // TODO: Do something with the frame.

            // measure current time; needs to be after frame is copied to shared memory. I think.
            int64_t timestampmicro = cluon::time::toMicroseconds(cluon::time::now());
            int64_t timestampsecs = timestampmicro / 1000000;

            // Crop the frame to get useful stuff
            frame(Rect(Point(0, 0), Point(640, 370))).copyTo(cropped_frame);

            // only start detecting cars when leading car is gone
            if (leading_car_gone == true) {
               if (yeet_sent == false) {

                  // Method for detecting car with haar cascade
                  findCars(cropped_frame, foundCars, carsCascadeClassifier);

                  // checks position and location of cars
                  // no theres no time to separate this function ok
                  detectCars(&od4, final_frame, foundCars, &prev_area, &prev_centerX, &prev_centerY,
                     &cars_in_queue, &car_leave_timeout_counter, &stop_line_arrived, &stop_line_arrived_trigger,
                     initial_car_positions, &left_car_is_12oclock_car);
               }
            }

            // notify movecar when it is time to go
            if (leading_car_gone == true && stop_line_arrived == true && cars_in_queue == 0 && yeet_sent == false) {
               SafeToGo yeet;
               od4.send(yeet);
               cout << endl << " --=== Time to leave intersection. Waiting for direction. ===-- " << endl;
               yeet_sent = true;
            }
             // Display image. For testing recordings only.
            if (VERBOSE) {
               // imshow("Detected cars", final_frame);
               // imshow("original frame", cropped_frame);
               // imshow("brightened bois", brightened_frame);
               cv::waitKey(1);
            }

            // measures FPS
            framecounter++;
            if (timestampsecs != prevtimestampsecs) {

               if (stop_line_arrived_trigger == true && stop_line_arrived == false) {
                  cout << "                   [ STOP LINE ARRIVED TRIGGERED: " << stop_line_arrived_trigger_counter << "]" << endl;
                  // wait 4 seconds
                  if (stop_line_arrived_trigger_counter > 0) { stop_line_arrived_trigger_counter -= 1;   }
                  if (stop_line_arrived_trigger_counter == 0) { stop_line_arrived = true;  }
               }

               if (car_leave_timeout_counter > 0) {
                  car_leave_timeout_counter -= 1;
                  if (car_leave_timeout_counter == 0) {
                     cout << "                   [ READY TO DETECT LEAVING CARS ]" << endl;
                  }
               }

               cout << endl << "Timestamp: " << timestampsecs << "          FPS: " << framecounter << endl;
               prevtimestampsecs = timestampsecs;
               framecounter = 0;
            }

         }
      }
     retCode = 0;
   }
   return retCode;
}

void findCars(Mat &frame, vector<Rect>& foundCars, CascadeClassifier carsCascadeClassifier) {

   Mat frame_gray;
   // the amount of overlapping squares on 1 place to confirm it is a car
   int min_neighbors = 3;

   cvtColor(frame, frame_gray, COLOR_RGB2GRAY );
   equalizeHist(frame_gray, frame_gray);
   carsCascadeClassifier.detectMultiScale(frame_gray, foundCars, 1.1, min_neighbors);

}

void countCars(Mat frame, vector<Point> &initial_car_positions, int *cars_in_queue, bool *stop_line_arrived) {
   int car_num = 0;
   if (initial_car_positions[0] != Point(0,0)) { // if theres a car on the left..
      car_num += 1;
      cout << endl << " <<<< ";
   }
   if (initial_car_positions[1] != Point(0,0)) { // if theres a car in the middle..
      car_num += 1;
      cout  << " |||| ";
   }
   if (initial_car_positions[2] != Point(0,0)) { // if theres a car on the right..
      car_num += 1;
      cout << " >>>> " << endl;
   }
   std::string car_count = std::to_string(car_num);


   if (*stop_line_arrived == false) {
      if (*cars_in_queue < car_num) {
         *cars_in_queue = car_num; // remember the maximum amount of cars seen
      }
   }
   std::string max_car_count = std::to_string(*cars_in_queue);

   cout << "  [<    CARS IN QUEUE : " << max_car_count << "  >]" ;
   if (car_num == 0) {
      // cout << "No cars. ";
   }
   else if (car_num == 1) {
      cout << "           [<  " << car_count << " car. >]  " << endl;
   } else {
      cout << "           [<  " << car_count << " cars. >] " << endl;
   }
   putText(frame, car_count, Point(5,100), FONT_HERSHEY_DUPLEX, 1, Scalar(255,255,255), 2);
   putText(frame, max_car_count, Point(630,100), FONT_HERSHEY_DUPLEX, 1, Scalar(255,255,255), 2);
}

void detectCars(
   OD4Session *od4, Mat& image, vector<Rect> &foundCars,
   double *prev_area, double *prev_centerX, double *prev_centerY,
   int *cars_in_queue, int *car_leave_timeout_counter, bool *stop_line_arrived, bool *stop_line_arrived_trigger,
   vector<Point> &initial_car_positions, bool *left_car_is_12oclock_car) {

   double rect_area = 0;
   double rect_centerX = 0; // valid range from 0 - 640
   double rect_centerY = 0; // valid range from 0 - 480

   for (size_t i = 0; i < foundCars.size(); i++) {
      int rect_x = foundCars[i].x;
      int rect_y = foundCars[i].y;
      int rect_width = foundCars[i].width;
      int rect_height = foundCars[i].height;
      rect_area = foundCars[i].area();

      rect_centerX = rect_x + 0.5 * rect_width;
      rect_centerY = rect_y + 0.5 * rect_height;

      // Now with those parameters you can calculate the 4 points
      Point top_left(rect_x,rect_y);
      Point top_right(rect_x + rect_width, rect_y);
      Point bot_left(rect_x, rect_y + rect_height);
      Point bot_right(rect_x + rect_width, rect_y + rect_height);

      checkCarPosition(
         od4, prev_centerX, prev_centerY, rect_centerX, rect_centerY,
         prev_area, rect_area, stop_line_arrived, stop_line_arrived_trigger, left_car_is_12oclock_car,
         initial_car_positions, cars_in_queue, car_leave_timeout_counter);

      countCars(image, initial_car_positions, cars_in_queue, stop_line_arrived);

      *prev_area = rect_area; // remember this rect's area and position for the next frame
      *prev_centerX = rect_centerX;
      *prev_centerY = rect_centerY;
   }
}

void removeCarFromQueue( vector<Point> &initial_car_positions, int *cars_in_queue, int *car_leave_timeout_counter) {

   if (*cars_in_queue == 1) { // deleting the only one left
      if (initial_car_positions[0] != Point(0,0)) {
         initial_car_positions[0] = Point(0,0);
         cout << "   Left Car deleted from queue." << endl;
      }
      else if (initial_car_positions[1] != Point(0,0)) {
         initial_car_positions[1] = Point(0,0);
         cout << "   Middle Car deleted from queue." << endl;
      }
      else if (initial_car_positions[2] != Point(0,0)) {
         initial_car_positions[2] = Point(0,0);
         cout << "   Right Car deleted from queue." << endl;
      }
   }

   else if (*cars_in_queue == 2) {
      if (initial_car_positions[0] == Point(0,0)) {
         cout << "No left car. Therefore  |||| >>>> ." << endl;
      }
      if (initial_car_positions[1] == Point(0,0)) {
         cout << "No middle car. Therefore   <<<< >>>> ." << endl;
      }
      if (initial_car_positions[2] == Point(0,0)) {
         cout << "No right car. Therefore    <<<< |||| ." << endl;
      }
      for (int i = 0; i < 3; i++) {
         if (initial_car_positions[i] != Point(0,0)) {
            initial_car_positions[i] = Point(0,0);
            cout << "Car deleted from queue." << endl;
            break;
         }
      }

   }
   else if (*cars_in_queue == 3) {
      for (int i = 0; i < 3; i++) {
         if (initial_car_positions[i] != Point(0,0)) {
            initial_car_positions[i] = Point(0,0);
            cout << "Car deleted from queue." << endl;
            break;
         }
      }
   }

   if (*cars_in_queue > 0) {
      *cars_in_queue -= 1;
      *car_leave_timeout_counter = 5; // new car must wait before leaving
      cout << "            [ TIMEOUT ON ] " << *car_leave_timeout_counter << endl;
   }
}

void checkCarPosition( OD4Session *od4,
   double *prev_centerX, double *prev_centerY, double centerX, double centerY,
   double *prev_area, double area, bool *stop_line_arrived, bool *stop_line_arrived_trigger, bool *left_car_is_12oclock_car,
   vector<Point> &initial_car_positions, int *cars_in_queue, int *car_leave_timeout_counter ) {

   double centerX_diff = centerX - *prev_centerX; // looks at whether or not car has moved
   double centerY_diff = centerY - *prev_centerY;

   int frame_center = 320;
   int left_offset; // section of the frame
   int right_offset;

   // special section of the frame that overlaps both left and middle.
   // detects when 12 o clock goes to the left side.
   int stop_line_arrival_offset = 220;

// different offsets are needed in different phases
   if (*stop_line_arrived == false) {
      left_offset = 270;
      right_offset = 50;
   }
   if (*stop_line_arrived == true) {
      left_offset = 320;
      right_offset = 25;
   }

   cout << "   [ Area: " << area << " ] " << endl;

   switch (*stop_line_arrived) {
      case false:

      cout << " [ center X: " << centerX << " ] // " << " [ X diff: " << centerX_diff <<  " ] ";
      cout << " // [[center Y: " << centerY << " ]] // " << " [ Y diff: " << centerY_diff << " ] "<< endl;

      // old code, used to know when we have arrived at the stop line without the need of a message.
      if (centerX < frame_center - stop_line_arrival_offset) {
         if (centerX_diff > -200 && centerX_diff < 0 && centerY_diff > 0) {
            if (*prev_centerX >= 30) {
               cout << "Detected car on left side, probably car at 12 o clock. Also probably close to stop line." << endl;
               *left_car_is_12oclock_car = true;
               *stop_line_arrived_trigger = true;
            }
         }
      }

      // if car on the left side of frame...
      if (centerX < frame_center - left_offset) {
         if (*left_car_is_12oclock_car == false) { // and we know we are not at the stop line...
            if (initial_car_positions[0] == Point(0,0)) { // only add if there is no existing left car
               initial_car_positions[0] = Point((int) centerX, (int) centerY);
               cout << "   <<<< ADDED LEFT CAR: " << initial_car_positions[0] << endl;
            }
            // the -200 check is to make sure it is the same car we are comparing.
            // else if car is going towards lower left corner of the frame....
            else if (centerX_diff > -200 && centerX_diff < 0 && centerY_diff > 0) {
               cout << "Detected car on left side, probably car at 3 o clock going out of sight" << endl;
            }
         }
      }

      if (centerX >= frame_center - left_offset && centerX < frame_center + right_offset) {
         if (initial_car_positions[1] == Point(0,0)) {
            initial_car_positions[1] = Point((int) centerX, (int) centerY);
            cout << "   |||| ADDED MIDDLE CAR: " << initial_car_positions[1] << endl;
         }
         else if (centerX_diff < 0 && centerY_diff > 0) { // going towards bot left corner
            if (*left_car_is_12oclock_car == false) {
               cout << "Detected car on middle, probably car at 12 o clock." << endl;
            }
         }
      }

      if (centerX > frame_center + right_offset) {
         if (area > 10000) { // prevent stopsign from being recognized as car
            if (initial_car_positions[2] == Point(0,0)) {
               initial_car_positions[2] = Point((int) centerX, (int) centerY);
               cout << "   >>>> ADDED RIGHT CAR: " << initial_car_positions[2] << endl;
            }
         }
         else if (centerX_diff > 0 && centerY_diff > 0) { // towards bot right
            cout << "Detected car on right side, probably car at 3 o clock." << endl;
         }
      }
      break;

      case true:
      // cout << "At Stop Line" << endl;
      cout << " [ center X: " << centerX << " ] // " << " [ X diff: " << centerX_diff <<  " ] ";
      cout << " // [[center Y: " << centerY << " ]] // " << " [ Y diff: " << centerY_diff << " ] "<< endl;

      if (*car_leave_timeout_counter == 0) {

         // car is on middle/left - camera too close to see left
         if (centerX >= frame_center - left_offset && centerX < frame_center + right_offset) {

            if (centerX_diff > -30 && centerX_diff < 0 && // makes sure it is the same car we are comparing
               centerY_diff > 0 && centerY_diff < 40) { // moving closer and towards the left
               if (area > 10000) {                        // if car is getting bigger
                  cout << "   / Car leaving Intersection, towards our lane or 9 o clock. /" << endl;
                  if (centerX < 220 && centerY > 230) {   // if very close to the bottom left of the frame
                     cout << " </< Car has left intersection at 9 o clock or towards our lane. </<" << endl;
                     removeCarFromQueue(initial_car_positions, cars_in_queue, car_leave_timeout_counter);
                  }
               }
            }

            //  if car is going higher in the frame and relatively straight
            else if (centerY_diff > -0.5 && centerY_diff < 200 &&
                     centerX_diff > -20 && centerX_diff < 10) {
               if (centerX > 210) {
                  cout << "   | Car leaving Intersection, towards 12 o clock. | " << endl;
                  if (area < 15000) {            // if car is very far away (enough)
                     cout << "    || Car has left intersection at 12 o clock. || " << endl;
                     removeCarFromQueue(initial_car_positions, cars_in_queue, car_leave_timeout_counter);
                  }
               }
            }

            // if car is relatively going in a straight line horizontally and is moving left
            else if (centerX_diff > -200 && centerX_diff < -0.5 &&
                     centerY_diff > -5 && centerY_diff < 5 ) {
               cout << "   < Car leaving Intersection, towards 9 o clock. < " << endl;
               if (centerX < 180 && centerY > 150 && centerY <= 240) { // if *relatively* at the edge of frame
                  cout << "   <<< Car has left Intersection, towards 9 o clock. <<< " << endl;
                  removeCarFromQueue(initial_car_positions, cars_in_queue, car_leave_timeout_counter);
               }
            }
         }

         // Moving in a straight line, from left to right
         if (centerX > frame_center + right_offset) {
            if (centerX_diff > 0.5 && centerX_diff < 200 &&
                  centerY_diff > -5 && centerY_diff < 5) {
               cout << "    > Car leaving Intersection towards 3 o clock >" << endl;
               if (centerX > 510) { // if on the far right
                  cout << "   >> Car has left at 3 o clock >> " << endl;
                  removeCarFromQueue(initial_car_positions, cars_in_queue, car_leave_timeout_counter);
               }
            }
         }
      }

      break;

   }
}
