#ifndef DISPARITY_H
#define DISPARITY_H

#include "StereoCapture.h"
#include "Calibrator.h"

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/ximgproc/disparity_filter.hpp>
#include <opencv2/cudastereo.hpp>


class Disparity
{
    public:
    int numDisparities = 9*16;
    int blockSize = 5;
    int preFilterCap = 31;
    int minDisparity = 4;
    int uniquenessRatio = 15;
    int speckleRange = 2;
    int speckleWindowSize = 150;
    int disp12MaxDiff = 20;
    int P1 = 200;
    int P2 = 800;
    int lambda = 8000;
    int sigma = 3;
    cv::Mat lgray, rgray, dispR;

    cv::Ptr<cv::StereoSGBM> stereo = cv::StereoSGBM::create();
    cv::Ptr<cv::ximgproc::DisparityWLSFilter>  wls_filter;
    cv::Ptr<cv::StereoMatcher> right_matcher;

    Disparity();
    void getDisparity(cv::Mat &lFrame, cv::Mat &rFrame, cv::Mat &disp, cv::Mat &dispFiltered);
    void adjustDisparityParams();
    void readParams();
    void writeParams() const;
};

static void on_trackbar1( int, void* dispObj);
static void on_trackbar2( int, void* dispObj);
static void on_trackbar3( int, void* dispObj);
static void on_trackbar4( int, void* dispObj);
static void on_trackbar5( int, void* dispObj);
static void on_trackbar6( int, void* dispObj);
static void on_trackbar7( int, void* dispObj);
static void on_trackbar8( int, void* dispObj);
static void on_trackbar9( int, void* dispObj);
static void on_trackbar10( int, void* dispObj);
static void on_trackbar11( int, void* dispObj);

#endif