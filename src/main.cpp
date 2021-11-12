#include <StereoCapture.h>
#include <Calibrator.h>
#include <Disparity.h>


void main_loop()
{
    // Initialize
    StereoCapture steCap;
    StereoCalib steCalib = StereoCalib();
    Disparity d = Disparity();
    cv::Mat lFrame, rFrame, lrFrame, lrIn;
    cv::Mat disp, disp_filtered;
    cv::Mat lProcessed, rProcessed;
    // Loop
    for(;;){
        // Successfully get frames
        if (steCalib.stereoUndisRemap(steCap, lFrame, rFrame, lProcessed, rProcessed))
        {
            d.getDisparity(lProcessed, rProcessed, disp, disp_filtered);
            cv::hconcat(lProcessed, rProcessed, lrFrame);
            cv::imshow("Left Right Image", lrFrame);
            cv::imshow("Dis", disp);
            cv::imshow("filtered", disp_filtered);
            char key = (char)cv::waitKey(30);
            if (key == 27)
                break;
            else if (key == 32) // Space
            {
                
                cv::hconcat(lFrame, rFrame, lrIn);
                cv::imwrite("..//visual//rawInput.jpg", lrIn);
                cv::imwrite("..//visual//undistorted.jpg", lrFrame);
                cv::imwrite("..//visual//disparity.jpg", disp*256);
                cv::imwrite("..//visual//disparity_filtered.jpg", disp_filtered);
            }
        }
        else
        {
            break;
        }
    }
}

void vidRecorder()
{
    StereoCapture steCap;
    cv::Mat lFrame, rFrame;
    steCap.getFrames(lFrame, rFrame);
    int f_width = lFrame.cols;
    int f_height = lFrame.rows;
    cv::VideoWriter videoLeft("left6.avi", cv::VideoWriter::fourcc('M','J','P','G'), 15, cv::Size(f_width,f_height));
    cv::VideoWriter videoRight("right6.avi", cv::VideoWriter::fourcc('M','J','P','G'), 15, cv::Size(f_width,f_height));
    
    for(;;){
        if(steCap.getFrames(lFrame, rFrame))
        {
            videoLeft.write(lFrame);
            videoRight.write(rFrame);
            cv::imshow("", lFrame);
            char key = (char)cv::waitKey(30);
            if (key == 27)
                break;
        }
    }
    videoLeft.release();
    videoRight.release();
}

int main()
{   
    main_loop();
    //vidRecorder();
    cv::waitKey(0);
    return 0;
}