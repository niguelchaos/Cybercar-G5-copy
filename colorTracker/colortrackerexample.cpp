#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include <iostream>

// code copied from
// https://docs.opencv.org/3.4/da/d0c/tutorial_bounding_rects_circles.html
// https://docs.opencv.org/3.4.3/da/d97/tutorial_threshold_inRange.html
using namespace cv;
const int max_value_H = 360/2;
const int max_value = 255;
const String window_capture_name = "Video Capture";
const String window_detection_name = "Object Detection";
int low_H = 0, low_S = 0, low_V = 0;
int high_H = max_value_H, high_S = max_value, high_V = max_value;
Mat frame, frame_HSV, frame_threshold;

using namespace std;
Mat src_gray;
int thresh = 100;
RNG rng(12345);

void thresh_callback(int, void* );

// For a trackbar which controls the lower range, say for example hue value:
static void on_low_H_thresh_trackbar(int, void *)
{
    low_H = min(high_H-1, low_H);
    setTrackbarPos("Low H", window_detection_name, low_H);
}
// For a trackbar which controls the upper range, say for example hue value:
static void on_high_H_thresh_trackbar(int, void *)
{
    high_H = max(high_H, low_H+1);
    setTrackbarPos("High H", window_detection_name, high_H);
}
// saturation
static void on_low_S_thresh_trackbar(int, void *)
{
    low_S = min(high_S-1, low_S);
    setTrackbarPos("Low S", window_detection_name, low_S);
}
static void on_high_S_thresh_trackbar(int, void *)
{
    high_S = max(high_S, low_S+1);
    setTrackbarPos("High S", window_detection_name, high_S);
}
// value
static void on_low_V_thresh_trackbar(int, void *)
{
    low_V = min(high_V-1, low_V);
    setTrackbarPos("Low V", window_detection_name, low_V);
}
static void on_high_V_thresh_trackbar(int, void *)
{
    high_V = max(high_V, low_V+1);
    setTrackbarPos("High V", window_detection_name, high_V);
}

void thresh_callback(int, void* )
{
       // Use cv::Canny to detect edges in the images.
    Mat canny_output;
    Canny( frame_threshold, canny_output, thresh, thresh*2 );

    // Finds contours and saves them to the vectors contour and hierarchy.
    vector<vector<Point> > contours;
    findContours( canny_output, contours, RETR_TREE, CHAIN_APPROX_SIMPLE );


    vector<vector<Point> > contours_poly( contours.size() );
    vector<Rect> boundRect( contours.size() );
    vector<Point2f>centers( contours.size() );
    vector<float>radius( contours.size() );

    // For every found contour we now apply approximation to polygons with accuracy +-3
    // and stating that the curve must be closed.
    for( size_t i = 0; i < contours.size(); i++ )
    {
        approxPolyDP( contours[i], contours_poly[i], 3, true );

        // After that we find a bounding rect for every polygon and save it to boundRect.
        boundRect[i] = boundingRect( contours_poly[i] );

        // At last we find a minimum enclosing circle for every polygon and save it to center and radius vectors.
        minEnclosingCircle( contours_poly[i], centers[i], radius[i] );
    }

// We found everything we need, all we have to do is to draw.
// Create new Mat of unsigned 8-bit chars, filled with zeros. It will contain all the drawings we are going to make (rects and circles).
    Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );

// For every contour: pick a random color,
// draw the contour, the bounding rectangle
// and the minimal enclosing circle with it.
    for( size_t i = 0; i< contours.size(); i++ )
    {
        Scalar color = Scalar(260,99,85 );
        drawContours( drawing, contours_poly, (int)i, color );
        rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2 );
        // circle( drawing, centers[i], (int)radius[i], color, 2 );
    }
// Display the results: create a new window "Contours" and show everything we added to drawings on it.
   auto combinedFrame = drawing + frame;
   imshow( window_capture_name, combinedFrame );
}

//  MAIN STARTS HERE
int main(int argc, char* argv[]) {

   // Capture the video stream from default or supplied capturing device.
   VideoCapture cap(argc > 1 ? atoi(argv[1]) : 0);

   // Create a window to display the default frame and the threshold frame.
   namedWindow(window_capture_name);
   namedWindow(window_detection_name);

   const int max_thresh = 255;

   // Trackbars to set thresholds for HSV values
   createTrackbar("Low H", window_detection_name, &low_H, max_value_H, on_low_H_thresh_trackbar);
   createTrackbar("High H", window_detection_name, &high_H, max_value_H, on_high_H_thresh_trackbar);
   createTrackbar("Low S", window_detection_name, &low_S, max_value, on_low_S_thresh_trackbar);
   createTrackbar("High S", window_detection_name, &high_S, max_value, on_high_S_thresh_trackbar);
   createTrackbar("Low V", window_detection_name, &low_V, max_value, on_low_V_thresh_trackbar);
   createTrackbar("High V", window_detection_name, &high_V, max_value, on_high_V_thresh_trackbar);

   // Create a trackbar on the threshold window and assign a callback function to it.
   createTrackbar( "Canny thresh:", window_detection_name, &thresh, max_thresh, thresh_callback );

   // It is necessary to find the maximum and minimum value to avoid discrepancies such as the high value of threshold becoming less than the low value.

   // Until the user wants the program to exit do the following
   while (true) {
     cap >> frame;
     if(frame.empty())
     {
         break;
     }
      // Convert from BGR to HSV colorspace
      cvtColor(frame, frame_HSV, COLOR_BGR2HSV);
      // Detect the object based on HSV Range Values
      inRange(frame_HSV, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), frame_threshold);

      // convert it into grayscale and blur it to get rid of the noise.
      cvtColor( frame, src_gray, COLOR_BGR2GRAY );
      blur( src_gray, src_gray, Size(3,3) );

      // Show the frames
      imshow(window_capture_name, frame);
      // imshow(window_detection_name, frame_HSV );
      imshow("threshold", frame_threshold);

      // imshow( source_window, frame);

      // In general callback functions are used to react to some kind of signal,
      // in our case it's trackbar's state change.
      // Explicit one-time call of thresh_callback is necessary to display the "Contours" window simultaniously with the "Source" window.
      // Except i put the trackbar in the same window with threshold

      thresh_callback( 0, 0 );

      char key = (char) waitKey(30);
      if (key == 'q' || key == 27)
      {
         destroyWindow(window_capture_name);
         break;
      }
   }
    return 0;
}
