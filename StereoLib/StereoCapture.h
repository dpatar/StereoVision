#ifndef STEREOCAPTURE_H
#define STEREOCAPTURE_H

#include <iostream>
#include <string>
#include <thread>
#include <mutex>
#include <unistd.h>
#include <condition_variable>
#include <chrono>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

class StereoCapture
{
    private:

        cv::Mat currLeftFrame, currRightFrame;
        std::mutex leftMutex, rightMutex, readyMutex;
        std::condition_variable cv;
        std::thread leftCamThread, rightCamThread;
        bool leftReady, rightReady, endProcess;
        std::string lCamID, rCamID;

        static void leftCamReader(StereoCapture* stcapObj);
        static void rightCamReader(StereoCapture* stcapObj);
    
    public:
         StereoCapture(std::string lID="/dev/video4", std::string rID="/dev/video2");
         ~StereoCapture();
         bool getFrames(cv::Mat &leftframe, cv::Mat &rightFrame);
};

#endif