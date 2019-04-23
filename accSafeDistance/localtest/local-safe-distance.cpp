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

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

#include <iostream>

using namespace cv;
using namespace std;
using namespace cluon;

static void help(const char* programName)
{
    cout <<
    "\nA program using pyramid scaling, Canny, contours and contour simplification\n"
    "to find squares in a list of images\n"
    "Returns sequence of squares detected on the image.\n"
    "Call:\n"
    "./" << programName << " [file_name (optional)]\n"
    "Using OpenCV version " << CV_VERSION << "\n" << endl;
}


int thresh = 50, N = 11;
// const char* wndname = "Square Detection Demo";

// helper function:
// finds a cosine of angle between vectors
// from pt0->pt1 and from pt0->pt2
double angle( Point pt1, Point pt2, Point pt0 )
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

// returns sequence of squares detected on the image.
void findSquares( const Mat& image, vector<vector<Point> >& squares )
{
    squares.clear();

    Mat pyr, timg, gray0(image.size(), CV_8U), gray;

    // down-scale and upscale the image to filter out the noise
    pyrDown(image, pyr, Size(image.cols/2, image.rows/2));
    pyrUp(pyr, timg, image.size());
    vector<vector<Point> > contours;

    // find squares in every color plane of the image
    for( int c = 0; c < 1; c++ )
    {
        int ch[] = {c, 0};
        mixChannels(&timg, 1, &gray0, 1, ch, 1);

        // try several threshold levels
        for( int l = 0; l < N; l++ )
        {
            // hack: use Canny instead of zero threshold level.
            // Canny helps to catch squares with gradient shading
            if( l == 0 )
            {
                // apply Canny. Take the upper threshold from slider
                // and set the lower to 0 (which forces edges merging)
                Canny(gray0, gray, 0, thresh, 5);
                // dilate canny output to remove potential
                // holes between edge segments
                dilate(gray, gray, Mat(), Point(-1,-1));
            }
            else
            {
                // apply threshold if l!=0:
                //     tgray(x,y) = gray(x,y) < (l+1)*255/N ? 255 : 0
                gray = gray0 >= (l+1)*255/N;
            }

            // find contours and store them all as a list
            findContours(gray, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);

            vector<Point> approx;
            // vector<Rect> boundRect( contours.size() );

            // test each contour
            for( size_t i = 0; i < contours.size(); i++ )
            {
                // approximate contour with accuracy proportional
                // to the contour perimeter
                // approxPolyDP(InputArray curve, OutputArray approxCurve, double epsilon, bool closed)
                approxPolyDP(contours[i], approx, arcLength(contours[i], true)*0.02, true);

                // square contours should have 4 vertices after approximation
                // relatively large area (to filter out noisy contours)
                // and be convex.
                // Note: absolute value (fabs) of an area is used because
                // area may be positive or negative - in accordance with the
                // contour orientation
               if( approx.size() == 4 &&
               fabs(contourArea(approx)) > 1000 &&
               fabs(contourArea(approx)) < 200000 &&
               isContourConvex(approx) ) {


               double maxCosine = 0;
               // double area = contourArea(approx);
               double length = arcLength(contours[i], true);

               for( int j = 2; j < 5; j++ )
               {
                  // find the maximum cosine of the angle between joint edges
                  double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
                  maxCosine = MAX(maxCosine, cosine);
               }

               // if cosines of all angles are small
               // (all angles are ~90 degree) then write quandrange
               // vertices to resultant sequence
               if( maxCosine < 0.2 ) {
                  // cout << "Is rectangle. \n";
                  // boundRect[i] = boundingRect( contours[i] );

                  squares.push_back(approx);
               }

               }
            }
        }
    }
}

double checkCarDistance(double area) {
   double speedlevel;
   if (area < 10000) {
      cout << "Too far away. Speed up. \n";
      speedlevel = 0.01;
   }
   if (area >= 10000 && area < 30000) {
      cout << "Catching up. Begin matching speed. \n";
      speedlevel = 0.005;
   }
   if (area >= 30000 && area < 40000) {
      // cout << "length: " << length << "\n";
      cout << "Optimal. Match Speed. \n";
      speedlevel = 0;
   }
   if (area >= 40000 && area < 60000) {
      cout << "Slow down. Almost crashing. \n";
      speedlevel = -0.005;
   }
   if (area >= 60000) {
      cout << "Stop. \n";
      speedlevel = -0.01;
   }
   return speedlevel;
}

double checkCarPosition(double centerX, int frame_center, int offset) {
   double position = 0;
   cout << "center X:       " << centerX << "\n";

   if (centerX < frame_center - offset) {
      cout << "      << car going left \n  ";
      position = 0.1;
   }
   if (centerX >= frame_center - offset && centerX < frame_center + offset) {
      cout << "      || car in front ||\n  ";
      position = 0;
   }
   if (centerX >= frame_center + offset) {
      cout << "      car going right >> \n";
      position = -0.1;
   }
   return position;
}

void countCars(Mat frame, vector<Rect >& squares) {
   int squareNum =  squares.size();
   std::string carcount = std::to_string(squareNum);
   // cout << "Detected      " << carcount << "cars. "<<"\n";
   putText(frame, carcount, Point(5,100), FONT_HERSHEY_DUPLEX, 1, Scalar(255,255,255), 2);

}

// the function draws all the squares in the image
Mat drawSquares( Mat& image, const vector<vector<Point> >& squares, int followcar, double *carResult ) // 0 = pink/other car, 1 = yellow/acc car
{
   int returnmsg; // -1 = speed down, 0 = nothing, 1 = speedup
    Scalar color = Scalar(255,0,0 );
    vector<Rect> boundRect( squares.size() );

    int group_thresh = 1;
   double merge_box_diff = 0.6; // how much rectangles have to overlap to merge

   cout << "before: " << boundRect.size();

    // for each square...
    for( size_t i = 0; i < squares.size(); i++ )
    {
        // const Point* p = &squares[i][0];
        // int n = (int)squares[i].size();
        // cout << "n: " << n << "\n";
        // polylines(image, &p, &n, 1, true, Scalar(0,255,0), 3, LINE_AA);

        // Code from http://answers.opencv.org/question/72237/measuring-width-height-of-bounding-box/
        boundRect[i] = boundingRect(squares[i]);
        rectangle(image, boundRect[i].tl(), boundRect[i].br(), color, 2 );

         // Make sure that the Rect is filled in any possible way
         int rect_x = boundRect[i].x;
         int rect_y = boundRect[i].y;
         int rect_width = boundRect[i].width;
         int rect_height = boundRect[i].height;
         double rect_area = boundRect[i].area();
         // Point2d center = boundRect[i].tl() + 0.5 * Point2d(boundRect[i].size()); //tl = top left, br = bot right

         double rect_centerX = rect_x + 0.5 * rect_width;
         double rect_centerY = rect_y + 0.5 * rect_height;
         int frame_center = 240;
         int offset = 20;


         // Now with those parameters you can calculate the 4 points
         Point top_left(rect_x,rect_y);
         Point top_right(rect_x + rect_width, rect_y);
         Point bot_left(rect_x, rect_y + rect_height);
         Point bot_right(rect_x + rect_width, rect_y + rect_height);



         if (followcar == 1) { // Yellow = if car is the one we are following, check position and distance
            double carDistance = 0;
            double carPosition = 0;

            carDistance = checkCarDistance(rect_area);
            carPosition = checkCarPosition(rect_centerX, frame_center, offset);
            carResult[0] = carDistance;
            carResult[1] = carPosition;
         }




         // cout << "rect_x:     " << rect_x << "\n";
         // cout << "rect_y:     " << rect_y << "\n";
         // cout << "rect_width:       " << rect_width << "\n";
         // cout << "rect_height:      " << rect_height << "\n";
         // cout << "area:       " << rect_area << "\n";

    }
    groupRectangles(boundRect, group_thresh, merge_box_diff);  //group overlapping rectangles
   cout << "         after: " << boundRect.size() << "\n";
   countCars(image, boundRect);

   return image;
}


int main(int argc, char** argv) {

   Mat frame;
   Mat frame_HSV;
   Mat frame_gray;
   Mat frame_threshold_pink;
   Mat frame_threshold_yellow;
   Mat finalFramePink;
   Mat finalFrameYellow;
   vector<vector<Point> > pinkSquares;
   vector<vector<Point> > yellowSquares;

   const int max_value_H = 360/2;
   const int max_value = 255;

// Pink
   int low_H_pink = 120;
   int low_S_pink = 10;
   int low_V_pink = 30;
   int high_H_pink = max_value_H;
   int high_S_pink = max_value;
   int high_V_pink = max_value;

// Yellow
   int low_H_yellow = 20;
   int low_S_yellow = 50;
   int low_V_yellow = 50;
   int high_H_yellow = 98;
   int high_S_yellow = 255;
   int high_V_yellow = 206;



   // Interface to a running OpenDaVINCI session; here, you can send and receive messages.
   static cluon::OD4Session od4(123,
     [](cluon::data::Envelope &&envelope) noexcept {
     if (envelope.dataType() == 2000) {
        HelloWorld receivedHello = cluon::extractMessage<HelloWorld>(std::move(envelope));
        std::cout << receivedHello.helloworld() << std::endl;
     }
     if (envelope.dataType() == 2001) {
        SpeedUp speedup = cluon::extractMessage<SpeedUp>(std::move(envelope));
        std::cout << speedup.speed() << std::endl;
     }
     if (envelope.dataType() == 2002) {
        SpeedDown speeddown = cluon::extractMessage<SpeedDown>(std::move(envelope));
        std::cout << speeddown.speed() << std::endl;
     }
   });

   // Capture the video stream from default or supplied capturing device.
   VideoCapture cap(argc > 1 ? atoi(argv[1]) : 0);

   while (true) {
   // get frame from the video
     cap >> frame;
   //   roi=selectROI("tracker",frame);
   //
     if(frame.empty()) {
         break;
         help(argv[0]);
     }

     HelloWorld helloworld;
     helloworld.helloworld("Hello world aaaaaaaaaa");
     od4.send(helloworld);

     SpeedUp speedup;
     SpeedDown speeddown;
     double carResult[2]; // slot 0 = distance, 1 = position

     carResult[0] = 0;
     carResult[1] = 0; //clear

      // Convert from BGR to HSV colorspace
      cvtColor(frame, frame_HSV, COLOR_BGR2HSV);
      // Detect the object based on HSV Range Values
      inRange(frame_HSV, Scalar(low_H_pink, low_S_pink, low_V_pink), Scalar(high_H_pink, high_S_pink, high_V_pink), frame_threshold_pink);
      inRange(frame_HSV, Scalar(low_H_yellow, low_S_yellow, low_V_yellow), Scalar(high_H_yellow, high_S_yellow, high_V_yellow), frame_threshold_yellow);

      // convert it into grayscale and blur it to get rid of the noise.
      // cvtColor( frame, frame_gray, COLOR_BGR2GRAY );
      // blur( frame_gray, frame_gray, Size(3,3) );

      findSquares(frame_threshold_pink, pinkSquares);
      finalFramePink = drawSquares(frame_threshold_pink, pinkSquares, 0, carResult);

      findSquares(frame_threshold_yellow, yellowSquares);
      finalFrameYellow = drawSquares(frame_threshold_yellow, yellowSquares, 1, carResult);

      if (carResult[0] < 0) { // slow down if speed is lower than 0
         speeddown.speed(carResult[0]);
         od4.send(speeddown);
      }
      else if (carResult[0] > 0) {
         speedup.speed(carResult[0]);
         od4.send(speedup);
      }


      // show image with the tracked object
      imshow("Pink", finalFramePink);
      imshow("Yellow", finalFrameYellow);

      // // show image with the tracked object
      // imshow("tracker",frame);

      int key = (char) waitKey(30);
      if ( key == 'q' || key == 27 ) {
          break;
      }
   }
    return 0;
}
