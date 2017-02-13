#ifndef __CamCalib__
#define __CamCalib__

#include <stdio.h>
#include <cstdlib>
#include <string>
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/ccalib.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"

class CamCalib
{
private:
	/*Main camera characteristics of calibration*/
	double errCC;//reprojectionError
	cv::Mat k, distortion; //cameraMatrix

	/*Other useful camera characteristics computed using the camera matrix:
	(apertureWidth, apertureHeight): Physical (width,height) of the sensor. (mm)
	(fovx,fovy): Output field of view in degrees along the (horizontal,vertical) sensor axis. (degree)
	aspectRatio: fy/fx.
	focalLength: Focal length of the lens. (mm)
	principalPoint: principalPoint. (mm) */
	double 	apertureWidth, apertureHeight;
	double  fovx, fovy, focalLength,aspectRatio;
	cv::Point2d principalPoint;

	void loadFile(std::string filePath);
	void writeFile(std::string filePath);

public:
	/*initiate all the private parameters with default values*/
	CamCalib();

	/*load all the chessboard imgs inside of a directory, and turn them into grayscala*/
	std::vector<cv::Mat> load2gray(std::string dir);

	/*chess board coner detection*/
	std::vector<std::vector<cv::Point2f> > conerDetect(std::vector<cv::Mat> imgs, cv::Size boardSize);

	/*Parameters: If recalib == True, readPath for input pics are need;
	else readPath <XXX.yml> is needed to initiate the Object, and writePath is neede for output file.
	Return: reprojectionError is returned */
	double calib(bool recalib, std::string readPath, std::string writePath, cv::Size boardSize, float squareSize);

	double getFocalLength(){return focalLength;}
	cv::Point2d getPrincipalPoint(){return principalPoint;}
	cv::Mat getCameraMatrix(){return k;}
	cv::Mat getDistortion(){return distortion;}

	~CamCalib();

};

#endif 
