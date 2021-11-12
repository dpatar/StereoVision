#include "StereoCapture.h"

StereoCapture::StereoCapture(std::string lID, std::string rID)
                :
                leftReady(false),
                rightReady(false),
                lCamID(lID),
                rCamID(rID),
                endProcess(false)
{
    this->leftCamThread = std::thread(leftCamReader, this);
    this->rightCamThread = std::thread(rightCamReader, this);
}

StereoCapture::~StereoCapture()
{
    this->endProcess = true;
    this->leftCamThread.join();
    this->rightCamThread.join();
}

bool StereoCapture::getFrames(cv::Mat &leftframe, cv::Mat &rightFrame)
{   
    std::unique_lock<std::mutex> ul(this->readyMutex);
    if(cv.wait_for(ul, std::chrono::seconds(5), [this] {return (leftReady && rightReady) ? true : false; }))
    {
        this->leftMutex.lock();
        this->rightMutex.lock();
        leftframe = this->currLeftFrame.clone();
        rightFrame = this->currRightFrame.clone();
        this->leftMutex.unlock();
        this->rightMutex.unlock();
        return true;
    }
    else{
        std::cerr << "Camera Initilization Timeout" << std::endl;
        return false;
    }
}

void StereoCapture::leftCamReader(StereoCapture* stcapObj)
{
    cv::VideoCapture cap(stcapObj->lCamID);
    if (!cap.isOpened())
    {
        std::cerr << "can not open camera with ID " << stcapObj->lCamID <<  std::endl;
        stcapObj->endProcess = true;
    }
    else{
        // Init
        cap.set(cv::CAP_PROP_BUFFERSIZE, 2);
        cv::Mat frame;
        cap >> frame;
        stcapObj->leftMutex.lock();
        stcapObj->currLeftFrame = frame.clone();
        stcapObj->leftMutex.unlock();
        std::unique_lock<std::mutex> ul(stcapObj->readyMutex);
        stcapObj->leftReady = true;
        ul.unlock();
        stcapObj->cv.notify_one();
        // Reader Loop
        while(!stcapObj->endProcess) // change inf loop
        {
            cap >> frame;
            stcapObj->leftMutex.lock();
            stcapObj->currLeftFrame = frame.clone();
            stcapObj->leftMutex.unlock();
            usleep(100000); // sleep for 0.1 sec
        }
    }
    cap.release();
}

void StereoCapture::rightCamReader(StereoCapture* stcapObj)
    {
        cv::VideoCapture cap(stcapObj->rCamID);
        if (!cap.isOpened())
        {
            std::cerr << "can not open camera with ID " << stcapObj->rCamID <<  std::endl;
            stcapObj->endProcess = true;
        }
        else{
            //Init
            cap.set(cv::CAP_PROP_BUFFERSIZE, 2);
            cv::Mat frame;
            cap >> frame;
            stcapObj->rightMutex.lock();
            stcapObj->currRightFrame = frame.clone();
            stcapObj->rightMutex.unlock();
            std::unique_lock<std::mutex> ul(stcapObj->readyMutex);
            stcapObj->rightReady = true;
            ul.unlock();
            stcapObj->cv.notify_one();
            //Reading Loop
            while(!stcapObj->endProcess) // change inf loop
            {
                cap >> frame;
                stcapObj->rightMutex.lock();
                stcapObj->currRightFrame = frame.clone();
                stcapObj->rightMutex.unlock();
                //usleep(100000); // sleep for 0.1 sec
                usleep(75000); // sleep for 0.075 sec
            }
        }
        cap.release();
    }