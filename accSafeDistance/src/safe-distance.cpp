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
                        #include <opencv2/videoio.hpp>
                        //#include <opencv2/dnn.hpp>
                        #include <cstring>
                        #include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <stdio.h>
                        
                        #include <cstdint>
                        #include <iostream>
                        #include <memory>
                        #include <mutex>
                        
                        using namespace std;
                        using namespace cv;
                        //defining variables for stop sign
                        String stopSignCascadeName;
                        CascadeClassifier stopSignCascade;
                        
                        void detectAndDisplayStopSign( Mat frame );
                        
                        
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
                                        
                            //Loading the haar cascade
                            //"../stopSignClassifier.xml" because the build file is in another folder, necessary to build for testing
                            stopSignCascadeName = "/usr/bin/stopSignClassifier.xml";
                            if(!stopSignCascade.load(stopSignCascadeName)){printf("--(!)Error loading stopsign cascade\n"); return -1; };
                                       
                                        
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
                                            
                                            // TODO: Do something with the frame.
                               
                                            
                                            frame(Rect(Point(100, 150), Point(580, 400))).copyTo(cropped_frame);
                                            
                                             // Method for detecting stop sign with haar cascade
                                            detectAndDisplayStopSign(frame);
                                            
                                           
                                            
                                            // show image with the tracked object
                                            // Example: Draw a red rectangle and display image.
                                            // cv::rectangle(img, cv::Point(50, 50), cv::Point(100, 100), cv::Scalar(0,0,255));
                                            
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
                        
                     
                        
                    //Haar cascade for Stop sign copied and modified from 
                    //https://docs.opencv.org/3.4.1/db/d28/tutorial_cascade_classifier.html
                    //Classifier gotten from : https://github.com/markgaynor/stopsigns
                    void detectAndDisplayStopSign( Mat frame )
                        {
                                        
                            std::vector<Rect> stopsigns;
                            Mat frame_gray;
                            cvtColor( frame, frame_gray, COLOR_BGR2GRAY );
                            equalizeHist( frame_gray, frame_gray );
                            //-- Detect stop signs
                            stopSignCascade.detectMultiScale(frame_gray, stopsigns, 1.1, 2, 0|CASCADE_SCALE_IMAGE, Size(60, 60));
                            for (size_t i = 0; i < stopsigns.size(); i++)
                            {
                                Point center( stopsigns[i].x + stopsigns[i].width/2, stopsigns[i].y + stopsigns[i].height/2 );
                                //Draw a circle when recognized
                                ellipse( frame, center, Size( stopsigns[i].width/2, stopsigns[i].height/2 ), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0 );
                                Mat faceROI = frame_gray( stopsigns[i] );
                            }
                            // -- Opens a new window with the Stop sign recognition on
                            imshow( "stopSign", frame );
                            
                        }
                        
