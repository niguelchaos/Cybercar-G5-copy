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
#include <cstring>

#include <iostream>

using namespace cv;
using namespace std;

static void help(const char* programName)
{
    cout <<
    "\nA program using pyramid scaling, Canny, contours and contour simplification\n"
    "to find squares in a list of images (pic1-6.png)\n"
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
static double angle( Point pt1, Point pt2, Point pt0 )
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

// returns sequence of squares detected on the image.
static void findSquares( const Mat& image, vector<vector<Point> >& squares )
{
    squares.clear();

    Mat pyr, timg, gray0(image.size(), CV_8U), gray;

    // down-scale and upscale the image to filter out the noise
    pyrDown(image, pyr, Size(image.cols/2, image.rows/2));
    pyrUp(pyr, timg, image.size());
    vector<vector<Point> > contours;

    // find squares in every color plane of the image
    for( int c = 0; c < 3; c++ )
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
               double area = contourArea(approx);
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

                  if (area < 10000) {
                     cout << "Too far away. Speed up. \n";
                  }
                  if (area >= 10000 && area < 30000) {
                     cout << "Catching up. Begin matching speed. \n";
                  }
                  if (area >= 30000 && area < 40000) {
                     // cout << "length: " << length << "\n";
                     cout << "Optimal. Match Speed. \n";
                  }
                  if (area >= 40000 && area < 60000) {
                     cout << "Slow down. Almost crashing. \n";
                  }
                  if (area >= 60000) {
                     cout << "Stop. \n";
                  }
                  squares.push_back(approx);
               }

               }
            }
        }
    }
}


// the function draws all the squares in the image
static Mat drawSquares( Mat& image, const vector<vector<Point> >& squares )
{
    Scalar color = Scalar(255,0,0 );
    vector<Rect> boundRect( squares.size() );

    // for each square...
    for( size_t i = 0; i < squares.size(); i++ )
    {
        // const Point* p = &squares[i][0];
        int n = (int)squares[i].size();
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

         double centerX = rect_x + 0.5 * rect_width;
         double centerY = rect_y + 0.5 * rect_height;


         // Now with those parameters you can calculate the 4 points
         Point top_left(rect_x,rect_y);
         Point top_right(rect_x + rect_width, rect_y);
         Point bot_left(rect_x, rect_y + rect_height);
         Point bot_right(rect_x + rect_width, rect_y + rect_height);

         // cout << "rect_x:     " << rect_x << "\n";
         // cout << "rect_y:     " << rect_y << "\n";
         // cout << "rect_width:       " << rect_width << "\n";
         // cout << "rect_height:      " << rect_height << "\n";
         // cout << "area:       " << rect_area << "\n";
         cout << "center X:       " << centerX << "\n";

         if (centerX < 300) {
            cout << "      << car going left \n  ";
         }
         if (centerX >= 300 && centerX < 340) {
            cout << "      || car in front ||\n  ";
         }
         if (centerX >= 340) {
            cout << "      car going right >> \n";
         }


    }

   return image;
}

int main(int argc, char** argv) {

   Mat frame;
   Mat finalFrame;
   vector<vector<Point> > squares;
   vector<Rect> boundRect;
   // Capture the video stream from default or supplied capturing device.
   VideoCapture cap(argc > 1 ? atoi(argv[1]) : 0);
   // Rect2d roi;
   // create a tracker object
   // Ptr<Tracker> tracker = Tracker::create( "KCF" );
   // // Until the user wants the program to exit do the following
   while (true) {
   // get frame from the video
     cap >> frame;
   //   roi=selectROI("tracker",frame);
   //
     if(frame.empty()) {
         break;
         help(argv[0]);
     }
   //   // initialize the tracker
   //   tracker->init(frame,roi);

   //
   //    // perform the tracking process
   //    printf("Start the tracking process, press ESC to quit.\n");
   //
   //    // stop the program if no more images
   //    if(frame.rows==0 || frame.cols==0)
   //    break;
   //    // update the tracking result
   //    tracker->update(frame,roi);
      // draw the tracked object
      // rectangle( frame, roi, Scalar( 255, 0, 0 ), 2, 1 );

      findSquares(frame, squares);
      finalFrame = drawSquares(frame, squares);

      // show image with the tracked object
      imshow("Rectangle Detector", finalFrame);
      // // show image with the tracked object
      // imshow("tracker",frame);

      int key = (char) waitKey(30);
      if ( key == 'q' || key == 27 ) {
          break;
      }
   }
    return 0;
}
