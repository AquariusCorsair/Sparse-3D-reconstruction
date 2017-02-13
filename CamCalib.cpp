#include "CamCalib.h"

#include <stdio.h>
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <string>
#include <dirent.h>

using namespace std;
using namespace cv;

CamCalib::CamCalib(){
	 errCC=0;
	 k = Mat::eye(3, 3, CV_64FC1);
	 distortion=Mat::zeros(8, 1, CV_64F);

	 apertureWidth=0;
	 apertureHeight=0;
	 fovx=0;
	 fovy=0;
	 focalLength=0;
	 aspectRatio=0;
	 principalPoint=Point2f(float(0),float(0));
}

CamCalib::~CamCalib(){}

void CamCalib::loadFile(string filePath){
	FileStorage fs(filePath, FileStorage::READ);
	fs["reprojectionError"] >> errCC;
	fs["cameraMatrix"] >> k;
	fs["distortion"] >> distortion;

	fs["apertureWeight"] >> apertureWidth;
	fs["apertureHeight"] >> apertureHeight;
	fs["fovx"] >> fovx;
	fs["fovy"] >> fovy;			
	fs["focalLength"] >> focalLength;
	fs["aspectRatio"] >> aspectRatio;			
	fs["principalPoint"] >> principalPoint;
	fs.release();
}

void CamCalib::writeFile(string filePath){
	FileStorage fs(filePath, FileStorage::WRITE);
	fs << "reprojectionError" << errCC;
	fs << "cameraMatrix" << k;
	fs << "distortion" << distortion;

	fs << "apertureWidth" << apertureWidth;		//Physical width in mm of the sensor.
	fs << "apertureHeight" << apertureHeight;
	fs << "fovx" << fovx;//Output field of view in degrees along the horizontal sensor axis.
	fs << "fovy" << fovy;
	fs << "focalLength" << focalLength;			//Focal length of the lens in mm
	fs << "aspectRatio" << aspectRatio;			//fy/fx
	fs << "principalPoint" << principalPoint;	//Principal point in mm.
	fs.release();
}

vector<Mat> CamCalib::load2gray(string dir){
	vector<cv::Mat> imgs;
	vector<cv::String> fn;
	cv::glob(dir,fn,true); // recurse
	for (size_t k=0; k<fn.size(); ++k)
	{
	    cv::Mat im = cv::imread(fn[k], CV_LOAD_IMAGE_GRAYSCALE);
	    if (im.empty()) continue; //only proceed if sucsessful
	    imgs.push_back(im);
	}
	cout<<"imgs number for calibration:" << imgs.size()<<endl;

	return imgs;
}

vector<vector<Point2f> > CamCalib::conerDetect(vector<Mat> imgs, Size boardSize){
	vector<vector<Point2f> > imgCorners;
	vector<Point2f> corners;

	for (int i = 0; i < imgs.size();i++)
	{
		bool found;
		found = cv::findChessboardCorners(imgs[i], boardSize, corners, CV_CALIB_CB_ADAPTIVE_THRESH + CV_CALIB_CB_FILTER_QUADS);
		imgCorners.push_back(corners);
		cornerSubPix(imgs[i], corners, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
		cout<<" corner number "<<corners.size()<<endl;
		// drawChessboardCorners(imgs[i], boardSize, corners, found);
		// imshow("corners", imgs[i]);
		// waitKey(0);
	 }

	return imgCorners;
}

double CamCalib::calib(bool recalib, string readPath, string writePath = "", cv::Size boardSize = Size(7,5), float squareSize = 1.f){
	if(! recalib){
		loadFile(readPath);
		return errCC;
	}

	//load chessboard images within folder <readPath> and convert them to grayscale
	vector<Mat> imgs = load2gray(readPath);
	// detect the imgCorners for all the imgs
	vector<vector<Point2f> > imgCorners = conerDetect(imgs, boardSize);
	// generate virtual objCorners using template obj 
	vector<Point3f> obj;
	for (int i = 0; i < boardSize.height; i++)
		for (int j = 0; j < boardSize.width; j++)
			obj.push_back(Point3f(float(j*squareSize),float(i*squareSize), 0));
	vector<vector<Point3f> > objCorners (imgs.size(),obj);
	// calibrate camera characters using virtual objCorners and imgCorners, and print the Reprojection error
	vector<Mat> R,t;
	errCC = calibrateCamera(objCorners, imgCorners, imgs[0].size(), k, distortion, R, t, CV_CALIB_ZERO_TANGENT_DIST | CALIB_FIX_K4 | CALIB_FIX_K5);
	std::cout << " Reprojection error of camera calibration:" << errCC << std::endl;
	//Computes useful camera characteristics from the camera matrix.
	cv::calibrationMatrixValues(k, imgs[0].size(), apertureWidth, apertureHeight, fovx, fovy, focalLength, principalPoint, aspectRatio);
	// store result into a file
	writeFile(writePath);

	return errCC;
}