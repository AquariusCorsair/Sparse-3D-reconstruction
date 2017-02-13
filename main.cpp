#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

#include "CamCalib.h"

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/opencv_modules.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/stitching/detail/autocalib.hpp"
#include "opencv2/stitching/detail/blenders.hpp"
#include "opencv2/stitching/detail/timelapsers.hpp"
#include "opencv2/stitching/detail/camera.hpp"
#include "opencv2/stitching/detail/exposure_compensate.hpp"
#include "opencv2/stitching/detail/matchers.hpp"
#include "opencv2/stitching/detail/motion_estimators.hpp"
#include "opencv2/stitching/detail/seam_finders.hpp"
#include "opencv2/stitching/detail/util.hpp"
#include "opencv2/stitching/detail/warpers.hpp"
#include "opencv2/stitching/warpers.hpp"

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;
using namespace cv::detail;

vector<Mat> load2gray(string dir){
	vector<cv::Mat> imgs;
	vector<cv::String> fn;
	cv::glob(dir,fn,true); // recurse
	for (size_t k=0; k<fn.size(); ++k)
	{
	    cv::Mat im = cv::imread(fn[k], CV_LOAD_IMAGE_GRAYSCALE);
	    if (im.empty()) continue; //only proceed if sucsessful
	    imgs.push_back(im);
	}
	cout<<"imgs total number:" << imgs.size()<<endl;

	return imgs;
}

ImageFeatures surfFeature(double hessThresh, Mat img, string dir){
	ImageFeatures features;

	Ptr<FeaturesFinder> finder = makePtr<SurfFeaturesFinder>(hessThresh);
	(*finder)(img, features);
	finder->collectGarbage();//Frees unused memory allocated before if there is any.
	Mat drawImg;
	drawKeypoints(img, features.keypoints, drawImg);
	cv::imwrite(dir, drawImg);
}


void featureTrack(vector<Mat> imgs, int maxFeatureCount, CamCalib cam){
	const int N = imgs.size();
	vector<Point2f> points[N];
	stringstream ss;

	//track keypoints between two images using opoticalFlow
	for(int i=1; i<N; i++){
		goodFeaturesToTrack(imgs[i-1], points[i-1], maxFeatureCount, 0.01, 10, Mat(), 3, 0, 0.04);

		//refine the coner location in points[i-1]
		Size subPixWinSize(10, 10);
		TermCriteria termcrit(TermCriteria::COUNT | TermCriteria::EPS, 20, 0.03);
		cornerSubPix(imgs[i-1], points[i-1], subPixWinSize, Size(-1, -1), termcrit);

		//calculate optical flow 
		Size winSize(31, 31);
		vector<uchar> status;
		vector<float> err;
		calcOpticalFlowPyrLK(imgs[i-1], imgs[i], points[i-1], points[i], status, err, winSize, 3, termcrit, 0, 0.001);
		if(points[i-1].size()!= points[i].size()) cout<<"point losing in opticalFlow"<<endl;
		
		// draw an arrow line on each detected flow
		for (int j = 0; j < points[i].size();j++)
		{ 
			cv::arrowedLine(imgs[i], points[i-1][j], points[i][j], 255);//draw white arrows
		}
		ss.str("");
		ss<<i;
		std::string name = "../output/flowToImg" + ss.str() + ".PNG";
		imwrite(name , imgs[i])	;//store tracking img
		// stiching two imgs 
		std::swap(points[i-1], points[i]);
		cv::swap(imgs[i-1], imgs[i]);
	}

	//3D traiangulation
	Mat RFst = (Mat_<double>(3, 3) << 1, 0, 0, 0, 1,0, 0, 0, 1);//R of right camera in reference to the leftest camera
	Mat tFst = (Mat_<double>(3, 1) << 0, 0, 0);
	cv::Mat points4DAve(4, points[0].size(), CV_64FC1);
	for (int i = 1; i < N; i++)
	{ 
		cout<<"reconstructing the "<<i<<"th img..."<<endl;
		//Find essential matrix from corresponding point pairs and intrinsic matrix	
		Mat E,mask;
		E = findEssentialMat(points[i-1], points[i], cam.getFocalLength(), cam.getPrincipalPoint(), RANSAC, 0.999, 1.0, mask);
		//decompose E using svd decomposition and narrow down to one set of 1R2 1t2 using cheirality check
		Mat_<double> R(Mat_<double>(3, 3));
		Mat_<double> t(Mat_<double>(3, 1));
		cv::recoverPose(E, points[i-1], points[i], R, t, cam.getFocalLength(), cam.getPrincipalPoint());//using cheirality check
		//construct extrinsic matrix for both cameras
		Mat P1 = (Mat_<double>(3, 4) << 1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0);
		Mat P2 = (Mat_<double>(3, 4) << R(0, 0), R(0, 1), R(0, 2), t(0),
			R(1, 0), R(1, 1), R(1, 2), t(1),
			R(2, 0), R(2, 1), R(2, 2), t(2));

		//remove lens distortion
		cv::undistortPoints(points[i-1], points[i-1], cam.getCameraMatrix(), cam.getDistortion());
		cv::undistortPoints(points[i], points[i], cam.getCameraMatrix(), cam.getDistortion());
		//construct projection matrix(intrinsic * extrinsic)
		P1 = cam.getCameraMatrix()*P1;
		P2 = cam.getCameraMatrix()*P2;
		P1.convertTo(P1, CV_64FC1);
		P2.convertTo(P2, CV_64FC1);
		//start triangulation
		cv::Mat points4D(4, points[i-1].size(), CV_64FC1);//4D in homo coordinates
		cv::Mat points3DAvg(3, points[i-1].size(), CV_64FC1);
		cv::triangulatePoints(P1, P2, points[i-1], points[i], points4D);

		//construct extrinsic matrix(camera poses) for the left camera in reference to the first one 
		tFst = tFst + (RFst.t())*t;	//1t3=1t2+2R1*2t3=1t2+inv(1R2)*2t3=1t2+1R2.transpose*2t3
		RFst = RFst*R;				//1R3=1R2*2R3
		Mat P2Fst(3, 4, CV_64FC1);
		P2Fst.col(0) = RFst.col(0);
		P2Fst.col(1) = RFst.col(1);
		P2Fst.col(2) = RFst.col(2);
		P2Fst.col(3) = tFst;
		//points3DAvg = points3DAvg + P2Fst*points4D;

		// store 3D cloud data in files
		ss.str("");
		ss<<i;
		FileStorage fs3Dx("../output/PointsCloudX"+ss.str()+".txt", FileStorage::WRITE);
		std::string XS = "X";
		fs3Dx << XS << points4D.row(0);
		fs3Dx.release();

		FileStorage fs3Dy("../output/PointsCloudY"+ss.str()+".txt", FileStorage::WRITE);
		std::string YS = "Y" ;
		fs3Dy << YS << points4D.row(1); 
		fs3Dy.release();

		FileStorage fs3Dz("../output/PointsCloudZ"+ss.str()+".txt", FileStorage::WRITE);
		std::string ZS = "Z";
		fs3Dz << ZS << points4D.row(2); 
		fs3Dz.release();
		
	}
}

int main(int argc, char** argv)
{
	//CalibrateCamera
	CamCalib cc;
	cc.calib(false,"../output/cam.yml","../output/cam.yml",cv::Size(7,5),1.f);

	//load homography imgs
	vector<Mat> imgs = load2gray("../input/restore");

	//use (SURF)FeatureFinder to find keypoints in image, and store img in dir
	ImageFeatures features = surfFeature(300, imgs[0], "../output/img1SURFfeatures.png");

	/*track keypoints between two images using opoticalFlow 
	/and 3D reconstruct, using matching features between two imgs and camera characters*/
	featureTrack(imgs, 250, cc);
	//output cloud points are stored in "../output/PointsCloud*.txt"
	
	
	return 0;
}
