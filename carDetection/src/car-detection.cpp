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

static double angle( Point pt1, Point pt2, Point pt0 );
static void findSquares( const Mat& image, vector<vector<Point> >& squares );
static Mat drawSquares( Mat& image, const vector<vector<Point> >& squares, vector<Rect> &boundRects, OD4Session *od4);

void checkCarPosition(OD4Session *od4, double *prev_centerX, double *prev_centerY, double centerX, double centerY,
   double *prev_area, double area, bool *stop_line_arrived,
   vector<Point> &initial_car_positions, int *cars_in_queue, int *car_leave_timeout_counter);
void countCars(Mat frame, vector<Point> &initial_car_positions , int *cars_in_queue);
void detectCars(
   OD4Session *od4, Mat& image, const vector<vector<Point> >& squares, vector<Rect> &boundRects,
   double *prev_area, double *prev_centerX, double *prev_centerY,
   int *cars_in_queue, int *car_leave_timeout_counter, bool *stop_line_arrived, vector<Point> &initial_car_positions);
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
         // Listen for when the car has arrived at stop line
   		auto onArrivedAtStopLine {
            [&od4, &stop_line_arrived](cluon::data::Envelope&&) {
					cout << "We have arrived at the stop line. " << endl;
   				stop_line_arrived = true;
            }
         };
         od4.dataTrigger(ArrivedAtStopLine::ID(), onArrivedAtStopLine);

         // Endless loop; end the program by pressing Ctrl-C.
         while (od4.isRunning()) {
            Mat frame;
            Mat frame_HSV;
            Mat frame_gray;
            Mat cropped_frame;
            Mat brightened_frame;
            Mat frame_threshold;
            Mat final_frame;
            vector<vector<Point> > squares;
            vector<Rect> boundRects;

            const int max_value_H = 360/2;
            const int max_value = 255;

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

         // Pink
            int low_H_pink = 135;
            int low_S_pink = 50;
            int low_V_pink = 65;
            int high_H_pink = max_value_H;
            int high_S_pink = max_value;
            int high_V_pink = max_value;

         // Green
            // int low_H_green = 42;
            // int low_S_green = 18;
            // int low_V_green = 102;
            // int high_H_green = 92;
            // int high_S_green = 182;
            // int high_V_green = 255;

            // Crop the frame to get useful stuff
            frame(Rect(Point(0, 0), Point(640, 370))).copyTo(cropped_frame);

            // auto brighten needs to be in BGR
            // RGB > BGR (brighten frame) > HSV (detect colors)
            // cvtColor(cropped_frame, brightened_frame, COLOR_RGB2BGR);
            // Automatically increase the brightness and contrast of the video.
            BrightnessAndContrastAuto(cropped_frame, brightened_frame, 0.6f);
            // Convert from BGR to HSV colorspace
            cvtColor(brightened_frame, frame_HSV, COLOR_RGB2HSV);
            // Detect the object based on HSV Range Values
            inRange(frame_HSV, Scalar(low_H_pink, low_S_pink, low_V_pink), Scalar(high_H_pink, high_S_pink, high_V_pink), frame_threshold);

            findSquares(frame_threshold, squares);
            final_frame = drawSquares(frame_threshold, squares, boundRects, &od4);

            detectCars(&od4, final_frame, squares, boundRects, &prev_area, &prev_centerX, &prev_centerY,
               &cars_in_queue, &car_leave_timeout_counter, &stop_line_arrived, initial_car_positions);

            // notify movecar when it is time to go
            if (stop_line_arrived == true && cars_in_queue == 0) {
               TimeToYeetOutOfIntersection yeet;
               od4.send(yeet);
               cout << " --=== Time to leave intersection. Waiting for direction. ===-- " << endl;
            }
             // Display image. For testing recordings only.
            if (VERBOSE) {
               imshow("Detected cars", final_frame);
               imshow("original frame", cropped_frame);
               imshow("brightened bois", brightened_frame);
               cv::waitKey(1);
            }

            // measures FPS
            framecounter++;
            if (timestampsecs != prevtimestampsecs) {
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

/**
 * Helper function to find a cosine of angle between vectors
 * from pt0->pt1 and pt0->pt2
 */
static double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0) {
   double dx1 = pt1.x - pt0.x;
   double dy1 = pt1.y - pt0.y;
   double dx2 = pt2.x - pt0.x;
   double dy2 = pt2.y - pt0.y;
   return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

// returns sequence of squares detected on the image.
static void findSquares( const Mat& image, vector<vector<Point> >& squares ) {
   int thresh = 50, N = 11;
   squares.clear();

   Mat pyr, timg, gray0(image.size(), CV_8U), gray;

   // down-scale and upscale the image to filter out the noise
   // blur works too.
   pyrDown(image, pyr, Size(image.cols/2, image.rows/2));
   pyrUp(pyr, timg, image.size());
   vector<vector<Point> > contours;

   // find squares in every color plane of the image
   for( int c = 0; c < 1; c++ ) {
      int ch[] = {c, 0};
      mixChannels(&timg, 1, &gray0, 1, ch, 1);

      // try several threshold levels
      for( int l = 0; l < N; l++ ) {
         // hack: use Canny instead of zero threshold level.
         // Canny helps to catch squares with gradient shading
         if( l == 0 ) {
             // apply Canny. Take the upper threshold from slider
             // and set the lower to 0 (which forces edges merging)
             Canny(gray0, gray, 0, thresh, 5);
             // dilate canny output to remove potential
             // holes between edge segments
             dilate(gray, gray, Mat(), Point(-1,-1));
         }
         else {
             // apply threshold if l!=0:
             //     tgray(x,y) = gray(x,y) < (l+1)*255/N ? 255 : 0
             // 255 is max saturation/value. N is just a threshold level.
             gray = gray0 >= (l+1)*255/N;
         }

         // find contours and store them all as a list
         findContours(gray, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);

         vector<Point> approx;

         // test each contour
         for( size_t i = 0; i < contours.size(); i++ ) {
            // approximate contour with accuracy proportional
            // to the contour perimeter
            approxPolyDP(contours[i], approx, arcLength(contours[i], true)*0.02, true);

            // square contours should have 4 vertices after approximation
            // relatively large area (to filter out noisy contours)
            // and be convex.
            // Note: absolute value (fabs) of an area is used because
            // area may be positive or negative - in accordance with the
            // contour orientation

            if( approx.size() == 4 && // if there are 4 sides...
            fabs(contourArea(approx)) > 700 && // and the square is big enough...
            fabs(contourArea(approx)) < 200000 &&
            isContourConvex(approx) ) { // and square is convex...

               double maxCosine = 0;
               for( int j = 2; j < 5; j++ ) {
                  // find the maximum cosine of the angle between joint edges
                  double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
                  maxCosine = MAX(maxCosine, cosine);
               }

               // if cosines of all angles are small
               // (all angles are ~90 degree) then its a square/rectangle.
               // push the vertices to resultant sequence(array)
               if( maxCosine < 0.25 ) {
                  squares.push_back(approx);
               }
            }
         }
      }
   }
}

// the function draws all the squares in the image
static Mat drawSquares( Mat& image, const vector<vector<Point> >& squares, vector<Rect> &boundRects, OD4Session *od4)
{
   Scalar color = Scalar(255,0,0 );

   int group_thresh = 1;
   double merge_box_diff = 0.6;

   boundRects.resize(squares.size());
// since the detected square may not always be straight, a bounding box around the...square ensures that its straight
   for( size_t i = 0; i < squares.size(); i++ ) {
      // Code from http://answers.opencv.org/question/72237/measuring-width-height-of-bounding-box/
      boundRects[i] = boundingRect(squares[i]);
      rectangle(image, boundRects[i].tl(), boundRects[i].br(), color, 2 );
   }
   groupRectangles(boundRects, group_thresh, merge_box_diff);  //group overlapping rectangles into 1
   // cout << "rectangles grouped/merged." << endl;
   return image;
}

void countCars(Mat frame, vector<Point> &initial_car_positions, int *cars_in_queue) {
   int car_num = 0;
   if (initial_car_positions[0] != Point(0,0)) {
      car_num += 1;
      cout << endl << " <<<< ";
   }
   if (initial_car_positions[1] != Point(0,0)) {
      car_num += 1;
      cout  << " |||| ";
   }
   if (initial_car_positions[2] != Point(0,0)) {
      car_num += 1;
      cout << " >>>> " << endl;
   }
   std::string car_count = std::to_string(car_num);
   std::string max_car_count = std::to_string(*cars_in_queue);

   if (*cars_in_queue < car_num) {
      *cars_in_queue = car_num; // remember the maximum amount of cars seen
   }

   cout << "  [<    CARS IN QUEUE : " << max_car_count << "  >]" << endl ;
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
   OD4Session *od4, Mat& image, const vector<vector<Point> >& squares, vector<Rect> &boundRects,
   double *prev_area, double *prev_centerX, double *prev_centerY,
   int *cars_in_queue, int *car_leave_timeout_counter, bool *stop_line_arrived, vector<Point> &initial_car_positions) {

   double rect_area = 0;
   double rect_centerX = 0; // valid range from 0 - 640
   double rect_centerY = 0; // valid range from 0 - 480

   for (size_t i = 0; i < boundRects.size(); i++) {
      int rect_x = boundRects[i].x;
      int rect_y = boundRects[i].y;
      int rect_width = boundRects[i].width;
      int rect_height = boundRects[i].height;
      rect_area = boundRects[i].area();

      rect_centerX = rect_x + 0.5 * rect_width;
      rect_centerY = rect_y + 0.5 * rect_height;

      // Now with those parameters you can calculate the 4 points
      Point top_left(rect_x,rect_y);
      Point top_right(rect_x + rect_width, rect_y);
      Point bot_left(rect_x, rect_y + rect_height);
      Point bot_right(rect_x + rect_width, rect_y + rect_height);

      checkCarPosition(
         od4, prev_centerX, prev_centerY, rect_centerX, rect_centerY,
         prev_area, rect_area,
         stop_line_arrived, initial_car_positions, cars_in_queue, car_leave_timeout_counter);

      if (*stop_line_arrived == false) {
         countCars(image, initial_car_positions, cars_in_queue);
      }

      *prev_area = rect_area; // remember this frame's area for the next frame
      *prev_centerX = rect_centerX;
      *prev_centerY = rect_centerY;
   }
}

void checkCarPosition( OD4Session *od4,
   double *prev_centerX, double *prev_centerY, double centerX, double centerY,
   double *prev_area, double area, bool *stop_line_arrived,
   vector<Point> &initial_car_positions, int *cars_in_queue, int *car_leave_timeout_counter) {

   double centerX_diff = centerX - *prev_centerX; // looks at whether or not car has moved
   double centerY_diff = centerY - *prev_centerY;

   int frame_center = 320;
   int left_offset;
   int right_offset;

   if (*stop_line_arrived == false) {
      left_offset = 220;
      right_offset = 50;
   }
   if (*stop_line_arrived == true) {
      left_offset = 320;
      right_offset = 25;
   }

   switch (*stop_line_arrived) {
      case false:

      cout << " [ center X: " << centerX << " ]   // " << " [ X diff: " << centerX_diff <<  " ] ";
      cout << " // [[center Y: " << centerY << " ]] // " << " [ Y diff: " << centerY_diff << " ] "<< endl;

      if (centerX < frame_center - left_offset) {
         if (initial_car_positions[0] == Point(0,0)) {
            initial_car_positions[0] = Point((int) centerX, (int) centerY);
            cout << "   <<<< ADDED LEFT CAR: " << initial_car_positions[0] << endl;
         }
         else if (centerX_diff > -200 && centerX_diff < 0 && centerY_diff > 0) {
            if (*prev_centerX >= 50) {
               cout << "Detected car on left side, probably car at 12 o clock. Also probably close to stop line." << endl;
               *stop_line_arrived = true;
            }
            else {
               cout << "Detected car on left side, probably car at 3 o clock going out of sight" << endl;
            }
         }
      }

      if (centerX >= frame_center - left_offset && centerX < frame_center + right_offset) {
         if (initial_car_positions[1] == Point(0,0)) {
            initial_car_positions[1] = Point((int) centerX, (int) centerY);
            cout << "   |||| ADDED MIDDLE CAR: " << initial_car_positions[1] << endl;
         }
         else if (centerX_diff < 0 && centerY_diff > 0) {
            cout << "Detected car on middle, probably car at 12 o clock." << endl;
         }
      }

      if (centerX > frame_center + right_offset) {
         if (initial_car_positions[2] == Point(0,0)) {
            initial_car_positions[2] = Point((int) centerX, (int) centerY);
            cout << "   >>>> ADDED RIGHT CAR: " << initial_car_positions[2] << endl;
         }
         else if (centerX_diff > 0 && centerY_diff > 0) {
            cout << "Detected car on right side, probably car at 3 o clock." << endl;
         }
      }
      break;

      case true:
      // cout << "At Stop Line" << endl;
      cout << " [ center X: " << centerX << " ]   // " << " [ X diff: " << centerX_diff <<  " ] ";
      cout << " // [[center Y: " << centerY << " ]] // " << " [ Y diff: " << centerY_diff << " ] "<< endl;

      // car is on middle/left - camera too close to see left
      if (centerX >= frame_center - left_offset && centerX < frame_center + right_offset) {

         if (centerX_diff < 0 && centerY_diff > 0) {
            if (area > 3500) {
               cout << "   / Car leaving Intersection, towards our lane. /" << endl;
               if (centerX < 40 && centerY > 180) {
                  cout << " /// Car has left intersection on our lane. ///" << endl;
                  *cars_in_queue -= 1;
                  *car_leave_timeout_counter = 5; // new car must wait 5 seconds before leaving
                  cout << "            [ TIMEOUT ON ] " << *car_leave_timeout_counter << endl;
               }
            }
         }

         else if (centerY_diff < 0) {
            cout << "   | Car leaving Intersection, towards 12 o clock. | " << endl;
            if (area < 3500) {
               cout << "    || Car has left intersection at 12 o clock. || " << endl;
               *cars_in_queue -= 1;
               *car_leave_timeout_counter = 5;
               cout << "            [ TIMEOUT ON ] " << *car_leave_timeout_counter << endl;
            }
         }

         else if (centerX_diff < 0 ) {
            cout << "   < Car leaving Intersection, towards 9 o clock. < " << endl;
            if (centerX < 40 && centerY < 180) {
               cout << "   <<< Car has left Intersection, towards 9 o clock. <<< " << endl;
               *cars_in_queue -= 1;
               *car_leave_timeout_counter = 5;
               cout << "            [ TIMEOUT ON ] " << *car_leave_timeout_counter << endl;
            }
         }
      }

      if (centerX > frame_center + right_offset) {

         if (centerX_diff > 0) {
            cout << "    > Car leaving Intersection towards 3 o clock >" << endl;
            if (centerX > 600) {
               cout << "   >> Car has left at 3 o clock >> " << endl;
               *cars_in_queue -= 1;
               *car_leave_timeout_counter = 5;
               cout << "            [ TIMEOUT ON ] " << *car_leave_timeout_counter << endl;
            }
         }
      }
      break;

   }

   // cout << " [[ area: " << area << " ]]";
   // cout << " // Area diff: " << area_diff << " // ";
   // if (area_diff < -100) { // If the car is moving away
   //    cout << "Car moving away   " << endl;
   // }
   // if (area_diff >= -100 && area_diff < 100) {
   //    cout << "car stationary " << endl;
   // }
   // if (area_diff >= 100) {
   //    cout << " car coming closer   " << endl ;
   // }
}

// https://answers.opencv.org/question/75510/how-to-make-auto-adjustmentsbrightness-and-contrast-for-image-android-opencv-image-correction/
void BrightnessAndContrastAuto(const cv::Mat &src, cv::Mat &dst, float clipHistPercent)
{
   // Tries to stretch/average out the range on a greyscale image. Might need tuning.
   // works only on gray images and ARGB color space.

    CV_Assert(clipHistPercent >= 0);
    CV_Assert((src.type() == CV_8UC1) || (src.type() == CV_8UC3) || (src.type() == CV_8UC4));

   // alpha (contrast) operates as color range amplifier, values from 0 - 3
   // beta (brightness) operates as range shift, values from 0 - 100
    int histSize = 256;
    float alpha, beta;
    double minGray = 0, maxGray = 0;

    //to calculate grayscale histogram
    cv::Mat gray;
    if (src.type() == CV_8UC1) gray = src;
    else if (src.type() == CV_8UC3) cvtColor(src, gray, COLOR_RGB2GRAY);
    else if (src.type() == CV_8UC4) cvtColor(src, gray, COLOR_RGBA2GRAY);
    if (clipHistPercent <= 0)
    {
        // keep full available range
        // Finds the global minimum and maximum in an array.
        cv::minMaxLoc(gray, &minGray, &maxGray);
        // cout << "Min:  || " << minGray << "Max" << maxGray << "||" << endl;
    }
    else
    {
        cv::Mat hist; //the grayscale histogram

        float range[] = { 0, 256 };
        const float* histRange = { range };
        bool uniform = true;
        bool accumulate = false;
        calcHist(&gray, 1, 0, cv::Mat (), hist, 1, &histSize, &histRange, uniform, accumulate);

        // calculate cumulative distribution from the histogram
        std::vector<float> accumulator(histSize);
        accumulator[0] = hist.at<float>(0);
        // cout << "accumulator [0]: " << accumulator[0] << endl;

        for (int i = 1; i < histSize; i++)
        {
            accumulator[i] = accumulator[i - 1] + hist.at<float>(i);
        }

        // locate points that cuts at required value
        float max = accumulator.back();
        // cout << "accumulator max: " << max << endl;

        clipHistPercent *= (max / 100.0f); //make percent as absolute
        // cout << "clipHistPercent % : " << clipHistPercent << endl;

        clipHistPercent /= 1.2f; // left and right wings
        // cout << "clipHistPercent L/R wings : " << clipHistPercent << endl;

        // locate left cut
        minGray = 0;
        while (accumulator[(long)minGray] < clipHistPercent) {
           minGray++;
        }

        // locate right cut
        maxGray = histSize - 1;
        while (accumulator[(long)maxGray] >= (max - clipHistPercent)) {
           maxGray--;
        }
    }
    // cout << "Min:  " << minGray << "  ||  Max" << maxGray << "   || " << endl;
    // current range - inputrange is always 255.
    // maxgray is always 255. min gray is always 0.
    float inputRange = (float)maxGray - (float)minGray;
    // cout << "Input range: " << inputRange << endl;
    alpha = (histSize - 1) / inputRange;   // alpha expands current range to histsize range
    beta = (float)-minGray * (float)alpha; // beta shifts current range so that minGray will go to 0
    // cout << " Added Contrast: " << alpha << "   || " << " Added Brightness: " << beta << endl;

    // does the actual brightening.
    // Apply brightness and contrast normalization
    // convertTo operates with saturate_cast
    src.convertTo(dst, -1, alpha, beta);

    // restore alpha channel from source / merges the frame back together from gray
    if (dst.type() == CV_8UC4)
    {
        int from_to[] = { 3, 3};
        cv::mixChannels(&src, 4, &dst,1, from_to, 1);
    }
    return;
}
