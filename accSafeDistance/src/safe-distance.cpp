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


static Mat drawSquares( Mat& image, const vector<vector<Point> >& squares, OD4Session *od4,
   double *prev_area, int *lost_visual_frame_counter, bool *sent_lost_visual, bool *stop_line_arrived);
static void findSquares( const Mat& image, vector<vector<Point> >& squares );
static double angle( Point pt1, Point pt2, Point pt0 );
void countCars(Mat frame, vector<Rect>& rects);
void checkCarPosition(double centerX, OD4Session *od4) ;
void checkCarDistance(double *prev_area, double area, double centerY, OD4Session *od4);
void BrightnessAndContrastAuto(const cv::Mat &src, cv::Mat &dst, float clipHistPercent);
void stopLineLostVisual(OD4Session *od4, int *lost_visual_sec_count, bool *sent_lost_visual);

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

         int64_t prevtimestampsecs = 0;
         int framecounter = 0;

         double prev_area = 0; // used to determine whether car is moving and amount of acceleration

         int lost_visual_frame_counter = 0; // needs 3 frames to trigger 1 lost_visual_sec_count
         int lost_visual_sec_count = 0;
         bool sent_lost_visual = false;

         bool stop_line_arrived = false;

         auto onStopCar {
            [&od4, &stop_line_arrived]
            (cluon::data::Envelope &&envelope) {

               auto msg = cluon::extractMessage<StopSignPresenceUpdate>(std::move(envelope));
               bool stopSignPresence = msg.stopSignPresence(); // Get the bool
               if (stopSignPresence == false) {
                  cout << "We have arrived at the stop line. " << endl;
      				stop_line_arrived = true;
               }
            }
         };
         od4.dataTrigger(StopSignPresenceUpdate::ID(), onStopCar);

         // Endless loop; end the program by pressing Ctrl-C.
         while (od4.isRunning()) {
            Mat frame;
            Mat frame_HSV;
            Mat frame_gray;
            Mat cropped_frame;
            // Mat cropped_frame_BGR;
            Mat brightened_frame;

            Mat frame_threshold_pink;
            Mat finalFramePink;
            vector<vector<Point> > pinkSquares;

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
            int low_S_pink = 53;
            int low_V_pink = 65;
            int high_H_pink = max_value_H;
            int high_S_pink = max_value;
            int high_V_pink = max_value;

            // Crop the frame to get useful stuff
            frame(Rect(Point(0, 0), Point(640, 370))).copyTo(cropped_frame);

            // only follow a car when we have not arrived at the line
            if (stop_line_arrived == false) {
               //////////////////// auto brightness /////////////////////
               // Automatically increase the brightness and contrast of the video.
               BrightnessAndContrastAuto(cropped_frame, brightened_frame, 0.6f);
               // Convert from BGR to HSV colorspace
               cvtColor(brightened_frame, frame_HSV, COLOR_RGB2HSV);
               //==//////////////////////////////////////////////////////==//

               ////////////// no auto brightness ////////////////////
               // cvtColor(cropped_frame, frame_HSV, COLOR_RGB2HSV);
               ////////////////////////////////////////////////

               // Detect the object based on HSV Range Values
               inRange(frame_HSV, Scalar(low_H_pink, low_S_pink, low_V_pink), Scalar(high_H_pink, high_S_pink, high_V_pink), frame_threshold_pink);

               findSquares(frame_threshold_pink, pinkSquares);
               finalFramePink = drawSquares(frame_threshold_pink, pinkSquares, &od4, &prev_area, &lost_visual_frame_counter, &sent_lost_visual, &stop_line_arrived); // pass reference of prev_area

               // findSquares(frame_threshold_green, greenSquares);
               // finalFrameGreen = drawSquares(frame_threshold_green, greenSquares, 0, &od4);
            }

             // Display image. For testing recordings only.
            if (VERBOSE) {
               // imshow("Pink", finalFramePink);
               // // imshow("Green", finalFrameGreen);
               // imshow("original frame", cropped_frame);
               // imshow("brightened bois", brightened_frame);
               cv::waitKey(1);
            }

            // measures FPS
            framecounter++;
            if (timestampsecs > prevtimestampsecs) {
               // framecounter > 1 reduces accidental increases due to startup of image
               // minimum frames changed to 2 because everything slower when running
               // only send message once
               if (lost_visual_frame_counter == 3 && framecounter > 0 && sent_lost_visual == false) {
                  stopLineLostVisual(&od4, &lost_visual_sec_count, &sent_lost_visual);
               }
               // reset if car is seen again
               if (lost_visual_frame_counter == 0) {   lost_visual_sec_count = 0;  }

               cout << "Timestamp: " << timestampsecs << "          FPS: " << framecounter << endl;
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
             // 255 is max saturation/value. N is just a thresh level.
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
            fabs(contourArea(approx)) > 1000 && // and the square is big enough...
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

void checkCarDistance(double *prev_area, double area, double centerY, OD4Session *od4) {
// PID controller
// https://robotics.stackexchange.com/questions/9786/how-do-the-pid-parameters-kp-ki-and-kd-affect-the-heading-of-a-differential
   SpeedCorrectionRequest speed_correction;
   float correction_speed;
   const float LOSTVISUAL = 1337;
   const float DECELERATE = 999;  // code for Artificial "Letting go of the pedal"

   if (area < 1 || centerY > LOSTVISUAL - 1) {
      // as per Elsadas request, attempted to put as much logic back into safedistance.cpp
      // correction_speed = LOSTVISUAL; // special code for visual lost
      correction_speed = -0.002;
   }
   else {
      float optimal_area = 8000; // default optimal area
      float area_diff = (float)area - (float) *prev_area; // looks at how much car has accelerated/deccelerated
      float accel_area_diff_thresh = 0;
      float brake_area_diff_thresh = 600;

      if (area_diff < accel_area_diff_thresh) { // If the car is moving away
         optimal_area = optimal_area + (area_diff * 0.2f); // dampen (softly) accelerate
      }

      if (area_diff >= brake_area_diff_thresh) {
         // make optimal area farther(smaller) if the car has accelerated a lot to brake earlier and harder.
         // small differences dont make much of a difference - deals with large variations of area
         optimal_area = optimal_area - (area_diff * 1.5f);
      }
      cout << endl << "         // Area diff: " << area_diff << "//    ";
      cout << "New Opt Area: " << optimal_area << "//" << endl;

   // the Integral and Derivative was not used due to time constraints.
      float error = optimal_area - (float) area;
      float kp = 1; // proportional gain constant, tunes controller. In our case, we have it as 1 to make it balanced.
      float output = kp * error;

      // braking needs to be stronger than accelerating, need to modify correction to suit it.
      if (output > 0) { correction_speed = output / 7500000; }
      if (output <= 0) { correction_speed = output / 100000; }

   // braking needs to be faster than accelerating. I dont care.
      if (correction_speed <= 0) { correction_speed = correction_speed * 5; } // hard multiplier by 5

   // If the car in front has somewhat been maintaining the distance and area > 5000 (close enough)
      if (error > 0 && error < 3000 && area_diff >= accel_area_diff_thresh && area_diff < brake_area_diff_thresh) {
         correction_speed = DECELERATE;
      }

      cout << " [[ area: " << area << " ]]";
      cout << "  // [[center Y: " << centerY << " ]] // ";
      cout << "  << speed correction : " << correction_speed << " >> // " << endl;
   }

   speed_correction.amount(correction_speed);
   od4->send(speed_correction);
}

void checkCarPosition(double centerX, OD4Session *od4) {

   int frame_center = 320; // Setpoint - we want the car to ideally be in center.
// PID controller test
// https://robotics.stackexchange.com/questions/9786/how-do-the-pid-parameters-kp-ki-and-kd-affect-the-heading-of-a-differential
   SteeringCorrectionRequest steering_correction;
   float correction_angle;
   const float LOSTVISUAL = 1337;

   if (centerX > LOSTVISUAL - 1) { // notify movecar - compiler complains if its == 1337, so now its > 1336
      correction_angle = LOSTVISUAL; // Special code for visual lost
   }
   else {
      float error = frame_center - (float) centerX;

   //////////////////////// Absolute pid steering correction ////////////////////
      float kp = 1;
      float output = kp * error; // Ki * integral + Kd * derivative
      correction_angle = output / 800; // because groundsteering max = 0.4 here

      cout << "[center X: " << centerX << " ]";
      cout << " // [[ steering correction: " << correction_angle << " ]]  // " << endl;
   }

   /////////////////////////// PID Controller test ////////////////////////////
   steering_correction.amount(correction_angle);
   od4->send(steering_correction);
}

void stopLineLostVisual(OD4Session *od4, int *lost_visual_sec_count, bool *sent_lost_visual) {
   CarOutOfSight car_outta_sight;
   *lost_visual_sec_count += 1;
   cout << "lost visual secs: " << *lost_visual_sec_count << endl;

   if (*lost_visual_sec_count > 2) {
      cout << "            << Lost Visual Sent. >> " << endl;
      *lost_visual_sec_count = 0;
      *sent_lost_visual = true;
      od4->send(car_outta_sight);
   }
}

// the function draws all the squares in the image
static Mat drawSquares(
   Mat& image, const vector<vector<Point> >& squares, OD4Session *od4,
   double *prev_area, int *lost_visual_frame_counter, bool *sent_lost_visual, bool *stop_line_arrived)
{
   Scalar color = Scalar(255,0,0 );
   vector<Rect> boundRects( squares.size() );

   int group_thresh = 1;
   double merge_box_diff = 0.6;

   for( size_t i = 0; i < squares.size(); i++ ) {
      // Code from http://answers.opencv.org/question/72237/measuring-width-height-of-bounding-box/
      boundRects[i] = boundingRect(squares[i]);
      rectangle(image, boundRects[i].tl(), boundRects[i].br(), color, 2 );
   }

   groupRectangles(boundRects, group_thresh, merge_box_diff);  //group overlapping rectangles into 1

   double rect_area = 0;
   double rect_centerX = 1337; // valid range from 0 - 640
   double rect_centerY = 1337; // valid range from 0 - 480

   // if there are no bounding Rects....
   if (boundRects.size() < 1) {
      // ...notify Movecar component that car is nowhere to be seen / lost visual
      checkCarDistance( prev_area, rect_area, rect_centerY, od4);
      checkCarPosition( rect_centerX, od4);
      // also begin counting if car is not seen for 5 seconds
      if (*lost_visual_frame_counter < 3) {
        *lost_visual_frame_counter += 1;
      }

      cout << "   counter: " << *lost_visual_frame_counter;
      cout << "   | Lost Visual | " << endl;
   }

   // if no bounding boxes, for loop is not entered because "i < boundrects.size" is -1
   // only check distance and steering corrections, along with number of cars, after merging.
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

     checkCarDistance( prev_area, rect_area, rect_centerY, od4);
     checkCarPosition( rect_centerX, od4);

     if (rect_area > 30000) { // for testing
        *sent_lost_visual = false;
        cout << "         << Lost Visual MESSAGE RESET. >> " << endl;
     }
     if (rect_area > 60000) {
        *stop_line_arrived = false;
        cout << "          [< Stop Line Reset - Scenario reset. >]" << endl;
     }

     *prev_area = rect_area; // remember this frame's area for the next frame
     *lost_visual_frame_counter = 0; // resets everything if a car is seen again
   }
   return image;
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
    // modified from BGR to RGB
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
        // cout << "accumulator [0]: " << accumulator[0];


        for (int i = 1; i < histSize; i++)
        {
            accumulator[i] = accumulator[i - 1] + hist.at<float>(i);
        }

        // locate points that cuts at required value
        float max = accumulator.back(); // last element of the array
         // cout << "      accumulator max: " << max << endl;
        clipHistPercent *= (max / 100.0f); //make percent as absolute
        // cout << "clipHistPercent % : " << clipHistPercent << endl;

        clipHistPercent /= 1.6f; // left and right wings
        // cout << "clipHistPercent L/R wings : " << clipHistPercent << endl;

        // accumulator[0] = 0;

// as the picamera seems to have a lower...color range than the algorithm,
// a hardcoded little boost of brightness and contrast is given
// by artificially bumping/lowering the values of both the upper and lower ends of the histogram
        // for (int a = 1; a < histSize/10; a++) {
        //     accumulator[a] = (accumulator[a - 1] + hist.at<float>(a)) / 2;
        //     // cout << "accumulator ["<< a << "]: " << accumulator[a] << endl;
        // }
        // for (int i = histSize - 1; i > histSize - histSize/11 ; i--) {
        //     accumulator[i] = (accumulator[i - 1] + hist.at<float>(i)) * 2;
        //     // cout << "accumulator ["<< i << "]: " << accumulator[i] << endl;
        // }

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
    // cout << "Min:  " << minGray << " || Max" << maxGray << endl;

    // current range
    float inputRange = (float)maxGray - (float)minGray;
    // histSize is always 255
    // inputrange in the end is always from 0 to 255
    // maxgray is always 255 as well, and mingray is always 0 in the end.
    alpha = (histSize - 1) / inputRange;   // alpha expands current range to histsize range
    beta = (float)-minGray * (float)alpha;  // beta shifts current range so that minGray will go to 0
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
