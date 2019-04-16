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
        // #include <opencv2/tracking.hpp>

        #include <cstdint>
        #include <iostream>
        #include <memory>
        #include <mutex>

        using namespace std;
        using namespace cv;

        static Mat drawSquares( Mat& image, const vector<vector<Point> >& squares );
        static void findSquares( const Mat& image, vector<vector<Point> >& squares );
        static double angle( Point pt1, Point pt2, Point pt0 );
        void countCars(Mat frame, vector<vector<Point> >& squares);

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

                    // Interface to a running OpenDaVINCI session; here, you can send and receive messages.
                    cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

                    // Endless loop; end the program by pressing Ctrl-C.
                while (od4.isRunning()) {
                    Mat frame;
                    Mat frame_HSV;
                    Mat frame_gray;
                    Mat cropped_frame;
                    Mat frame_threshold_pink;
                    Mat frame_threshold_green;
                    Mat frame_threshold_red;
                    Mat finalFramePink;
                    Mat finalFrameGreen;
                    Mat finalFrameRed;
                    vector<vector<Point> > pinkSquares;
                    vector<vector<Point> > greenSquares;
                    vector<vector<Point> > redSquares;

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


                    // Pink
                    int low_H_pink = 130;
                    int low_S_pink = 55;
                    int low_V_pink = 100;
                    int high_H_pink = max_value_H;
                    int high_S_pink = max_value;
                    int high_V_pink = max_value;

                    // Green
                    int low_H_green = 42;
                    int low_S_green = 18;
                    int low_V_green= 102;
                    int high_H_green= 92;
                    int high_S_green= 182;
                    int high_V_green= 255;
                    
                    // Red
                    int low_H_red = 154;
                    int low_S_red = 60;
                    int low_V_red= 150;
                    int high_H_red= 180;
                    int high_S_red= max_value;
                    int high_V_red= max_value;

                    frame(Rect(Point(100, 150), Point(580, 400))).copyTo(cropped_frame);

                    // Convert from BGR to HSV colorspace
                    cvtColor(cropped_frame, frame_HSV, COLOR_RGB2HSV);
                    // Detect the object based on HSV Range Values
                    inRange(frame_HSV, Scalar(low_H_pink, low_S_pink, low_V_pink), Scalar(high_H_pink, high_S_pink, high_V_pink), frame_threshold_pink);
                    inRange(frame_HSV, Scalar(low_H_green, low_S_green, low_V_green), Scalar(high_H_green, high_S_green, high_V_green), frame_threshold_green);
                    inRange(frame_HSV, Scalar(low_H_red, low_S_red, low_V_red), Scalar(high_H_red, high_S_red, high_V_red), frame_threshold_red);

                    findSquares(frame_threshold_pink, pinkSquares);
                    finalFramePink = drawSquares(frame_threshold_pink, pinkSquares);

                    findSquares(frame_threshold_green, greenSquares);
                    finalFrameGreen = drawSquares(frame_threshold_green, greenSquares);
                    
                    findSquares(frame_threshold_red, redSquares);
                    finalFrameRed = drawSquares(frame_threshold_red, redSquares);

                    countCars(finalFramePink, pinkSquares);
                    countCars(finalFrameGreen, greenSquares);
                    countCars(finalFrameRed, redSquares);


                    // show image with the tracked object
                    // Example: Draw a red rectangle and display image.
                    // cv::rectangle(img, cv::Point(50, 50), cv::Point(100, 100), cv::Scalar(0,0,255));

                    // Display image.
                    if (VERBOSE) {
                    imshow("Pink", finalFramePink);
                    imshow("Green", finalFrameGreen);
                    imshow("Red", finalFrameRed);
                        cv::waitKey(1);
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

        /**
        * Helper function to display text in the center of a contour
        */
        // void setLabel(cv::Mat& im, const std::string label, std::vector& contour) {
        //     int fontface = cv::FONT_HERSHEY_SIMPLEX;
        //     double scale = 0.4;
        //     int thickness = 1;
        //     int baseline = 0;
        //
        //     cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);
        //     cv::Rect r = cv::boundingRect(contour);
        //
        //     cv::Point pt(r.x + ((r.width - text.width) / 2), r.y + ((r.height + text.height) / 2));
        //     cv::rectangle(im, pt + cv::Point(0, baseline), pt + cv::Point(text.width, -text.height), CV_RGB(255,255,255), CV_FILLED);
        //     cv::putText(im, label, pt, fontface, scale, CV_RGB(0,0,0), thickness, 8);
        // }

        // returns sequence of squares detected on the image.
        static void findSquares( const Mat& image, vector<vector<Point> >& squares )
        {
        int thresh = 50, N = 11;
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

                    // test each contour
                    for( size_t i = 0; i < contours.size(); i++ )
                    {
                        // approximate contour with accuracy proportional
                        // to the contour perimeter
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
                    for( int j = 2; j < 5; j++ )
                    {
                        // find the maximum cosine of the angle between joint edges
                        double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
                        maxCosine = MAX(maxCosine, cosine);
                    }

                    // if cosines of all angles are small
                    // (all angles are ~90 degree) then write quandrange
                    // vertices to resultant sequence
                    if( maxCosine < 0.25 ) {
                        // cout << "Is rectangle. \n";
                        if (area < 10000) {
                            cout << "Too far away. Speed up. \n";
                        }
                        if (area >= 10000 && area < 30000) {
                            cout << "Catching up. Begin matching speed. \n";
                        }
                        if (area >= 30000 && area < 40000) {
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
        Scalar color = Scalar(255,0,0);
        vector<Rect> boundRect( squares.size() );
        const int max_value_H = 360/2;
                    const int max_value = 255;

            for( size_t i = 0; i < squares.size(); i++ )
            {
                const Point* p = &squares[i][0];
                int n = (int)squares[i].size();
                polylines(image, &p, &n, 1, true, Scalar(0,255,0), 3, LINE_AA);

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
                Point bot_left(rect_x, rect_y + rect_height);int low_H_pink = 130;
                int low_S_pink = 55;
                int low_V_pink = 100;
                int high_H_pink = max_value_H;
                int high_S_pink = max_value;
                int high_V_pink = max_value;
                Point bot_right(rect_x + rect_width, rect_y + rect_height);

                // cout << "rect_x:     " << rect_x << "\n";
                // cout << "rect_y:     " << rect_y << "\n";
                // cout << "rect_width:       " << rect_width << "\n";
                // cout << "rect_height:      " << rect_height << "\n";
                // cout << "area:       " << rect_area << "\n";
                cout << "center X:       " << rect_centerX << "\n";

                if (rect_centerX < frame_center - offset) {
                cout << "      << car going left \n  ";
                }
                if (rect_centerX >= frame_center - offset && rect_centerX < frame_center + offset) {
                cout << "      || car in front ||\n  ";
                }
                if (rect_centerX >= frame_center + offset) {
                cout << "      car going right >> \n";
                }
            }

        return image;
        }

        void countCars(Mat frame, vector<vector<Point> >& squares) {
        int squareNum =  squares.size();
        std::string carcount = std::to_string(squareNum);
        cout << "Detected      " << carcount << "cars. "<<"\n";
        putText(frame, carcount, Point(5,100), FONT_HERSHEY_DUPLEX, 1, Scalar(0,0,255), 2);

        }
