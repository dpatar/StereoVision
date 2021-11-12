#include "Disparity.h"

Disparity::Disparity()
    :
    stereo()
{   
    //readParams
    this->stereo = cv::StereoSGBM::create(this->minDisparity, this->numDisparities, this->blockSize, this->P1, this->P2,
                                        this->disp12MaxDiff, this->preFilterCap, this->uniquenessRatio,this->speckleWindowSize,
                                        this->speckleRange);
    this->wls_filter = cv::ximgproc::createDisparityWLSFilter(this->stereo);
    this->right_matcher = cv::ximgproc::createRightMatcher(this->stereo);
}

void Disparity::getDisparity(cv::Mat &lFrame, cv::Mat &rFrame, cv::Mat &disp, cv::Mat &dispFiltered)
{
    cv::cvtColor(lFrame, this->lgray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(rFrame, this->rgray, cv::COLOR_BGR2GRAY);
    this->wls_filter = cv::ximgproc::createDisparityWLSFilter(this->stereo);
    this->right_matcher = cv::ximgproc::createRightMatcher(this->stereo);
    this->stereo->compute(lgray, rgray, disp);
    right_matcher->compute(rgray, lgray, this->dispR);
    wls_filter->setLambda(this->lambda);
    wls_filter->setSigmaColor(this->sigma);
    wls_filter->filter(disp, lFrame, dispFiltered, this->dispR);
    disp.convertTo(disp, CV_32F, 1.0);
    disp = (disp/16.0f - (float) minDisparity) / ((float)numDisparities);
    cv::ximgproc::getDisparityVis(dispFiltered, dispFiltered,1);
}

void Disparity::adjustDisparityParams()
{
    // Initialize
    StereoCapture steCap;
    StereoCalib steCalib = StereoCalib();
    cv::Mat lFrame, rFrame, lrFrame, lProcessed, rProcessed;;
    cv::Ptr<cv::StereoBM> bm = cv::StereoBM::create(64,5);
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create();
    cv::Mat lgray, rgray, disp, disp_filtered;

    cv::namedWindow("disparity",cv::WINDOW_NORMAL);
    cv::resizeWindow("disparity", 600,600); 
    cv::createTrackbar("numDisparities", "disparity", &this->numDisparities, 16, on_trackbar1, this);
    cv::createTrackbar("blockSize", "disparity", &this->blockSize, 50, on_trackbar2, this);
    cv::createTrackbar("P1", "disparity", &this->P1, 400, on_trackbar3, this);
    cv::createTrackbar("P2", "disparity", &this->P2, 1000, on_trackbar4, this);
    cv::createTrackbar("preFilterCap", "disparity", &this->preFilterCap, 62, on_trackbar5, this);
    cv::createTrackbar("lambda", "disparity", &this->lambda, 32000, on_trackbar6, this);
    cv::createTrackbar("uniquenessRatio", "disparity", &this->uniquenessRatio, 100, on_trackbar7, this);
    cv::createTrackbar("speckleRange", "disparity", &this->speckleRange, 100, on_trackbar8, this);
    cv::createTrackbar("speckleWindowSize", "disparity", &this->speckleWindowSize, 200, on_trackbar9, this);
    cv::createTrackbar("disp12MaxDiff", "disparity", &this->disp12MaxDiff, 25, on_trackbar10, this);
    cv::createTrackbar("minDisparity", "disparity", &this->minDisparity, 25, on_trackbar11, this);

    // Loop
    for(;;){

        // Successfully get frames
        if (steCalib.stereoUndisRemap(steCap, lFrame, rFrame, lProcessed, rProcessed))
        {
            this->getDisparity(lProcessed, rProcessed, disp, disp_filtered);
            cv::hconcat(lProcessed, rProcessed, lrFrame);
            cv::imshow("Left Right Image", lrFrame);
            cv::imshow("Dis", disp);
            cv::imshow("filtered", disp_filtered);
            char key = (char)cv::waitKey(30);
            if (key == 27)
                break;
        }
        else
        {
            break;
        }
    }
}

static void on_trackbar1( int, void* dispObj)
{
    Disparity* disp = (Disparity*)dispObj;
    disp->stereo->setNumDisparities(disp->numDisparities*16);
    disp->numDisparities = disp->numDisparities*16;
}
static void on_trackbar2( int, void* dispObj)
{
    Disparity* disp = (Disparity*)dispObj;
    disp->stereo->setBlockSize(disp->blockSize*2+5);
    disp->blockSize = disp->blockSize*2+5;
}
static void on_trackbar3( int, void* dispObj)
{
    Disparity* disp = (Disparity*)dispObj;
    disp->stereo->setP1(disp->P1);
}
static void on_trackbar4( int, void* dispObj)
{
    Disparity* disp = (Disparity*)dispObj;
    disp->stereo->setP2(disp->P2);
}
static void on_trackbar5( int, void* dispObj)
{
    Disparity* disp = (Disparity*)dispObj;
    disp->stereo->setPreFilterCap(disp->preFilterCap);
}
static void on_trackbar6( int, void* dispObj)
{
    Disparity* disp = (Disparity*)dispObj;
    disp->wls_filter->setLambda(disp->lambda);
}
static void on_trackbar7( int, void* dispObj)
{
    Disparity* disp = (Disparity*)dispObj;
    disp->stereo->setUniquenessRatio(disp->uniquenessRatio);
}
static void on_trackbar8( int, void* dispObj)
{
    Disparity* disp = (Disparity*)dispObj;
    disp->stereo->setSpeckleRange(disp->speckleRange);
}
static void on_trackbar9( int, void* dispObj)
{
    Disparity* disp = (Disparity*)dispObj;
    disp->stereo->setSpeckleWindowSize(disp->speckleWindowSize * 2);
    disp->speckleWindowSize = disp->speckleWindowSize * 2;
}
static void on_trackbar10( int, void* dispObj)
{
    Disparity* disp = (Disparity*)dispObj;
    disp->stereo->setDisp12MaxDiff(disp->disp12MaxDiff);
}
static void on_trackbar11( int, void* dispObj)
{
    Disparity* disp = (Disparity*)dispObj;
    disp->stereo->setMinDisparity(disp->minDisparity);
}
