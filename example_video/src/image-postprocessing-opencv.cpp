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

    #include <opencv2/highgui/highgui.hpp>
    #include <opencv2/imgproc/imgproc.hpp>
    #include "opencv2/imgcodecs.hpp"

    #include <cstdint>
    #include <iostream>
    #include <memory>
    #include <mutex>

    using namespace cv;
    using namespace std;

    static void on_low_H_thresh_trackbar(int, void *);
    static void on_high_H_thresh_trackbar(int, void *);
    static void on_low_S_thresh_trackbar(int, void *);
    static void on_high_S_thresh_trackbar(int, void *);
    static void on_low_V_thresh_trackbar(int, void *) ;
    static void on_high_V_thresh_trackbar(int, void *);

    cv::Mat threshDrawing(cv::Mat img);

    const int max_value_H = 360/2;
    const int max_value = 255;

    const String window_detection_name = "Color thresholds";
// for color pink
    int low_H = 130;
    int high_H = 180;
    int low_S = 55;
    int high_S = 255;
    int low_V = 180;
    int high_V = 255;

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

        // Create a window to display the default frame and the threshold frame.
        //namedWindow(window_capture_name);
        namedWindow(window_detection_name);

        // const int max_thresh = 255;




        // Trackbars to set thresholds for HSV values
        createTrackbar("Low H", window_detection_name, &low_H, max_value_H, on_low_H_thresh_trackbar);
        createTrackbar("High H", window_detection_name, &high_H, max_value_H, on_high_H_thresh_trackbar);
        createTrackbar("Low S", window_detection_name, &low_S, max_value, on_low_S_thresh_trackbar);
        createTrackbar("High S", window_detection_name, &high_S, max_value, on_high_S_thresh_trackbar);
        createTrackbar("Low V", window_detection_name, &low_V, max_value, on_low_V_thresh_trackbar);
        createTrackbar("High V", window_detection_name, &high_V, max_value, on_high_V_thresh_trackbar);

            // Attach to the shared memory.
            std::unique_ptr<cluon::SharedMemory> sharedMemory{new cluon::SharedMemory{NAME}};
            if (sharedMemory && sharedMemory->valid()) {
                std::clog << argv[0] << ": Attached to shared memory '" << sharedMemory->name() << " (" << sharedMemory->size() << " bytes)." << std::endl;

                // Interface to a running OpenDaVINCI session; here, you can send and receive messages.
                cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

                // Endless loop; end the program by pressing Ctrl-C.
                while (od4.isRunning()) {
                    cv::Mat img;
                    cv::Mat cropped_img;
                    cv::Mat img_threshold;
                    cv::Mat img_hsv;

                    cv::Mat img_gray;
                    cv::Mat outlines;


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
                        img = wrapped.clone();
                    }
                    sharedMemory->unlock();

                    // TODO: Do something with the frame.
                    
                    

                    // Select ROI
                    img(Rect(Point(0, 100), Point(640, 400))).copyTo(cropped_img);

                    cvtColor(cropped_img, img_hsv, COLOR_RGB2HSV);
                    inRange(img_hsv, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), img_threshold);

                    cvtColor( cropped_img, img_gray, COLOR_RGB2GRAY);
                    GaussianBlur( img_gray, img_gray, Size(3,3),0,0 );
                    outlines = threshDrawing(img_threshold);
                    cropped_img = outlines + cropped_img;
                    // Example: Draw a red rectangle and display image.
                    cv::rectangle(img, cv::Point(50, 50), cv::Point(100, 100), cv::Scalar(0,0,255));

                    // Display image.
                    if (VERBOSE) {
                    cv::imshow("Color Threshold", img_threshold);
                        cv::imshow(sharedMemory->name().c_str(),cropped_img);
                        cv::waitKey(1);
                    }
                }
            }
            retCode = 0;
        }
        return retCode;
    }

    cv::Mat threshDrawing(cv::Mat img) {
        // Use cv::Canny to detect edges in the images.
        Mat canny_output;
        int thresh = 200;

        Canny(img, canny_output, thresh, thresh*2 );

        // Finds contours and saves them to the vectors contour and hierarchy.
        vector<vector<Point> > contours;
        findContours( canny_output, contours, RETR_TREE, CHAIN_APPROX_SIMPLE );

// 
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
            // minEnclosingCircle( contours_poly[i], centers[i], radius[i] );
        }
        

    // We found everything we need, all we have to do is to draw.
    // Create new Mat of unsigned 8-bit chars, filled with zeros. It will contain all the drawings we are going to make (rects and circles).
        Mat drawing = Mat::zeros( canny_output.size(), CV_8UC4 );

    // For every contour: pick a random color,
    // draw the contour, the bounding rectangle
    // and the minimal enclosing circle with it.
        for( size_t i = 0; i< contours.size(); i++ )
        {
            Scalar color = Scalar(0,0,255 );
            drawContours( drawing, contours_poly, (int)i, color );
            rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), color, 2 );
            // circle( drawing, centers[i], (int)radius[i], color, 2 );
            putText(drawing, "Car", boundRect[i].tl(), 1, 1.0, color);
        }
    // Display the results: create a new window "Contours" and show everything we added to drawings on it.
    // auto combinedFrame = drawing + frame;
    // imshow( window_capture_name, combinedFrame );
    return drawing;
    }

    // For a trackbar which controls the lower range, say for example hue value:
    static void on_low_H_thresh_trackbar(int, void *) {
        low_H = min(high_H-1, low_H);
        setTrackbarPos("Low H", window_detection_name, low_H);
    }
    // For a trackbar which controls the upper range, say for example hue value:
    static void on_high_H_thresh_trackbar(int, void *) {
        high_H = max(high_H, low_H+1);
        setTrackbarPos("High H", window_detection_name, high_H);
    }
    // saturation
    static void on_low_S_thresh_trackbar(int, void *) {
        low_S = min(high_S-1, low_S);
        setTrackbarPos("Low S", window_detection_name, low_S);
    }
    static void on_high_S_thresh_trackbar(int, void *) {
        high_S = max(high_S, low_S+1);
        setTrackbarPos("High S", window_detection_name, high_S);
    }
    // value
    static void on_low_V_thresh_trackbar(int, void *) {
        low_V = min(high_V-1, low_V);
        setTrackbarPos("Low V", window_detection_name, low_V);
    }
    static void on_high_V_thresh_trackbar(int, void *) {
        high_V = max(high_V, low_V+1);
        setTrackbarPos("High V", window_detection_name, high_V);
    }
