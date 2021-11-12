#ifndef CALIBRATOR_H
#define CALIBRATOR_H

#include "StereoCapture.h"
#include <opencv2/highgui.hpp>  //namedWindow imgshow waitkey
#include <opencv2/imgproc.hpp>  //cvtColor
#include <opencv2/calib3d.hpp>

class Calibrator
{
public:
    std::string imgsPath, camName;
    cv::Mat cameraMatrix, distCoeffs, R, T, new_cam_mat;
    cv::Mat map1, map2;
    cv::Size imSize;
    std::vector<std::vector<cv::Point3f>> objPoints;
    std::vector<std::vector<cv::Point2f>> imgPoints;
public:
    Calibrator(std::string cam_name="left", std::string path="../calib_imgs/");
    void writeCalibToFile() const;
    bool readCalibFile();
    void produceCalibParams();
    void static produceCalibImgs();
};

class StereoCalib
{
    public:
    cv::Mat Rot, Trans, Essen, Fund;
    cv::Mat rect_l, rect_r, proj_l, proj_r, Q;
    cv::Mat l_ste_map1, l_ste_map2, r_ste_map1, r_ste_map2;
    cv::Mat mapl1, mapl2, mapr1, mapr2;
    cv::Mat lPreProc, rPreProc;
    
    StereoCalib();
    void calibStereo(Calibrator &left, Calibrator &right);
    bool stereoUndisRemap(StereoCapture &steCap, cv::Mat &left_f, cv::Mat &right_f, cv::Mat &lout, cv::Mat &rout);
    void stereoUndisRemap(cv::Mat &left_f, cv::Mat &right_f, cv::Mat &lout, cv::Mat &rout);
    void writeStereoParams() const;
    bool readStereoParams();
    static void remapping(cv::Mat &in, cv::Mat &out, cv::Mat &map1, cv::Mat &map2);
};


#endif