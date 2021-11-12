#include "Calibrator.h"


Calibrator::Calibrator(std::string cam_name, std::string path)
                    :                    
                    camName(cam_name),
                    imgsPath(path)
{
    if(!this->readCalibFile())
    {   
        // Assuming calib images available
        this->produceCalibParams();
    }
    cv::initUndistortRectifyMap(this->cameraMatrix, this->distCoeffs, cv::Mat(), 
                                cv::getOptimalNewCameraMatrix(this->cameraMatrix, this->distCoeffs, this->imSize, 1, this->imSize, 0),
                                this->imSize, CV_16SC2, this->map1, this->map2);
}

void Calibrator::writeCalibToFile() const
{
    std::string filePath = this->imgsPath + this->camName + ".xml";
    cv::FileStorage fs (filePath, cv::FileStorage::WRITE);
    fs << "cameraMatrix" << this->cameraMatrix;
    fs << "distCoeffs" << this->distCoeffs;
    fs << "R" << this->R;
    fs << "T" << this->T;
    fs << "imSize" << this->imSize;
    fs << "newCamMat" << this->new_cam_mat;
    fs << "objPoints" << this->objPoints;
    fs << "imgPoints" << this->imgPoints;
    fs.release();
}

bool Calibrator::readCalibFile()
{   
    std::string filePath = this->imgsPath + this->camName + ".xml";
    cv::FileStorage fs (filePath, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
      std::cerr << "failed to open " << filePath << '\n';
      fs.release();
      return 0;
    }
    else
    {
        fs["cameraMatrix"] >> this->cameraMatrix;
        fs["distCoeffs"] >> this->distCoeffs;
        fs["R"] >> this->R;
        fs["T"] >> this->T;
        fs["imSize"] >> this->imSize;
        fs["newCamMat"] >> this->new_cam_mat;
        fs["objPoints"] >> this->objPoints;
        fs["imgPoints"] >> this->imgPoints;
        fs.release();
        return 1;
    }
}

void Calibrator::produceCalibParams()
{
    int CHECKBOARD[2]{6,9};
    std::vector<std::vector<cv::Point2f>> imgPoints;
    std::vector<cv::Point3f> objp;

    for(int i{0}; i<CHECKBOARD[1]; i++)
    {
        for(int j{0};  j<CHECKBOARD[0]; j++)
        {
            objp.push_back(cv::Point3f(j*2.5,i*2.5,0));
        }
    }

    std::vector<cv::String> images;
    std::string path = this->imgsPath + this->camName + "/*.jpg";
    cv::glob(path, images);
    cv::Mat frame, gray;
    std::vector<cv::Point2f> corner_pts;
    bool success;
    // Use only the ones with chessboard
    for(int i{0}; i<images.size(); i++)
    {
        frame = cv::imread(images[i]);
        cv::cvtColor(frame,gray, cv::COLOR_BGR2GRAY);
        success = cv::findChessboardCorners(gray, cv::Size(CHECKBOARD[0], CHECKBOARD[1]), corner_pts,
                                            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);

        if(success)
        {
            cv::TermCriteria crit(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);
            cv::cornerSubPix(gray,corner_pts, cv::Size(11,11), cv::Size(-1,-1), crit);
            cv::drawChessboardCorners(frame, cv::Size(CHECKBOARD[0], CHECKBOARD[1]), corner_pts, success);
            this->objPoints.push_back(objp);
            this->imgPoints.push_back(corner_pts);
        }
        // cv::imshow("Image", frame);
        // cv::waitKey(0);
    }
    this->imSize = cv::Size(gray.cols, gray.rows);
    // cv::destroyAllWindows();

    cv::calibrateCamera(this->objPoints, this->imgPoints, this->imSize,
                        this->cameraMatrix, this->distCoeffs, this->R, this->T);
    this->new_cam_mat = cv::getOptimalNewCameraMatrix(this->cameraMatrix, this->distCoeffs, this->imSize, 1, this->imSize, 0);
    writeCalibToFile();
}

void Calibrator::produceCalibImgs()
{
    StereoCapture steCap;
    cv::Mat left_Frame, right_Frame;
    cv::namedWindow("Left Image", cv::WINDOW_NORMAL);
    cv::namedWindow("Right Image", cv::WINDOW_NORMAL);
    int savedFrameID = 0;
    std::string l_fileName, r_fileName;
    for(;;){
        // Successfully get frames
        if (steCap.getFrames(left_Frame,right_Frame))
        {
            cv::imshow("Left Image", left_Frame);
            cv::imshow("Right Image", right_Frame);
            char key = (char)cv::waitKey(30);
            if (key == 27)
                break;
            else if (key == 32) // Space
            {
                l_fileName = "..//calib_imgs//left//" + std::to_string(savedFrameID) + ".jpg";
                cv::imwrite(l_fileName, left_Frame);
                r_fileName = "..//calib_imgs//right//" + std::to_string(savedFrameID) + ".jpg";
                cv::imwrite(r_fileName, right_Frame);
                std::cout << std::to_string(savedFrameID) << " Img saved"<< std::endl;
                savedFrameID++ ;
            }
        }
        else
        {
            break;
        }
    }
}

StereoCalib::StereoCalib()
{
    if(!this->readStereoParams())
    {
        Calibrator lcalib("left"), rcalib("right");
        this->calibStereo(lcalib, rcalib);
    }
}

void StereoCalib::calibStereo(Calibrator &left, Calibrator &right)
{
    cv::stereoCalibrate(left.objPoints, left.imgPoints, right.imgPoints, left.new_cam_mat, left.distCoeffs,
                        right.new_cam_mat, right.distCoeffs, left.imSize, this->Rot, this->Trans, this->Essen, this->Fund,
                        cv::CALIB_FIX_INTRINSIC, cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 30, 1e-6));
    
    cv::stereoRectify(left.new_cam_mat, left.distCoeffs, right.new_cam_mat, right.distCoeffs, left.imSize, this->Rot, this->Trans,
                    this->rect_l, this->rect_r, this->proj_l, this->proj_r, this->Q, 1);
    
    cv::initUndistortRectifyMap(left.new_cam_mat, left.distCoeffs, this->rect_l, this->proj_l, left.imSize, CV_16SC2, this->mapl1, this->mapl2);
    cv::initUndistortRectifyMap(right.new_cam_mat, right.distCoeffs, this->rect_r, this->proj_r, right.imSize, CV_16SC2, this->mapr1, this->mapr2);
    writeStereoParams();
}

bool StereoCalib::stereoUndisRemap(StereoCapture &steCap, cv::Mat &left_f, cv::Mat &right_f, cv::Mat &lout, cv::Mat &rout)
{   
    bool succ = steCap.getFrames(left_f, right_f);
    if(succ)
    {
        std::thread mapping1 = std::thread(StereoCalib::remapping, std::ref(left_f), std::ref(lout), std::ref(this->mapl1), std::ref(this->mapl2));
        std::thread mapping2 = std::thread(StereoCalib::remapping, std::ref(right_f), std::ref(rout), std::ref(this->mapr1), std::ref(this->mapr2));
        mapping1.join();
        mapping2.join();
        // cv::remap(left_f, left_f, this->mapl1, this->mapl2, cv::INTER_LINEAR);
        // cv::remap(right_f, right_f, this->mapr1, this->mapr2, cv::INTER_LINEAR);
    }
    return succ;
}

void StereoCalib::stereoUndisRemap(cv::Mat &left_f, cv::Mat &right_f,  cv::Mat &lout, cv::Mat &rout)
{       
    std::thread mapping1 = std::thread(StereoCalib::remapping, std::ref(left_f), std::ref(lout), std::ref(this->mapl1), std::ref(this->mapl2));
    std::thread mapping2 = std::thread(StereoCalib::remapping, std::ref(right_f), std::ref(rout), std::ref(this->mapr1), std::ref(this->mapr2));
    mapping1.join();
    mapping2.join();
    // cv::remap(left_f, left_f, this->mapl1, this->mapl2, cv::INTER_LINEAR);
    // cv::remap(right_f, right_f, this->mapr1, this->mapr2, cv::INTER_LINEAR);
    
}

void StereoCalib::remapping(cv::Mat &in, cv::Mat &out, cv::Mat &map1, cv::Mat &map2)
{
    cv::bilateralFilter(in, out, 9, 175, 175);
    cv::remap(out, out, map1, map2, cv::INTER_LINEAR);
}

void StereoCalib::writeStereoParams() const
{
    std::string filePath = "../calib_imgs/stereoParam.xml";
    cv::FileStorage fs(filePath, cv::FileStorage::WRITE);
    fs << "mapl1" << this->mapl1;
    fs << "mapl2" << this->mapl2;
    fs << "mapr1" << this->mapr1;
    fs << "mapr2" << this->mapr2;
    fs.release();
}

bool StereoCalib::readStereoParams()
{
    std::string filePath = "../calib_imgs/stereoParam.xml";
    cv::FileStorage fs(filePath, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        std::cerr << "failed to open " << filePath << '\n';
        fs.release();
        return 0;
    }
    else
    {
        fs["mapl1"] >> this->mapl1;
        fs["mapl2"] >> this->mapl2;
        fs["mapr1"] >> this->mapr1;
        fs["mapr2"] >> this->mapr2;
        fs.release();
        return 1;
    }
}