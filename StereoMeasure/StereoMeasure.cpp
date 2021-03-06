// StereoMeasure.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include <string>
#include <cstdio>
#include <iostream>
#include <opencv2\opencv.hpp>

using namespace std;
using namespace cv;

Mat XYZ;
Rect vroi;

Mat R, T, E, F;
Mat R1, R2, P1, P2, Q;

int mx, my;

void detectDistance(cv::Mat& pointCloud)
{
	if (pointCloud.empty())
	{
		return;
	}

	// 提取深度图像
	vector<cv::Mat> xyzSet;
	split(pointCloud, xyzSet);
	cv::Mat depth;
	xyzSet[2].copyTo(depth);

	// 根据深度阈值进行二值化处理
	double maxVal = 0, minVal = 0;
	cv::Mat depthThresh = cv::Mat::zeros(depth.rows, depth.cols, CV_8UC1);
	cv::minMaxLoc(depth, &minVal, &maxVal);
	double thrVal = minVal * 1.5;
	threshold(depth, depthThresh, thrVal, 255, CV_THRESH_BINARY_INV);
	depthThresh.convertTo(depthThresh, CV_8UC1);

	double  distance = depth.at<float>(my, mx);
	cout << " " << distance << "CM" << endl;

}

void on_mouse(int event, int x, int y, int flags, void *ustc)
{
	mx = x;
	my = y;
	if (event == CV_EVENT_LBUTTONUP) {
		cout << mx << " " << my << " ";
		detectDistance(XYZ);
	}
}


int getPointClouds(cv::Mat& disparity, cv::Mat& pointClouds)
{
	if (disparity.empty())
	{
		return 0;
	}

	reprojectImageTo3D(disparity, pointClouds, Q, true);

	pointClouds *= 1.6;

	for (int y = 0; y < pointClouds.rows; ++y)
	{
		for (int x = 0; x < pointClouds.cols; ++x)
		{
			cv::Point3f point = pointClouds.at<cv::Point3f>(y, x);
			point.y = -point.y;
			pointClouds.at<cv::Point3f>(y, x) = point;
		}
	}



	return 1;
}



int main() {

	VideoCapture lCamera, RCamera;
	
	int lid, rid;

	FileStorage fs;

	fs.open("..\\CameraID.yml", FileStorage::READ);

	if (!fs.isOpened())
		return -1;
	fs["LID"] >> lid;
	fs["RID"] >> rid;

	lCamera.open(lid);
	RCamera.open(rid);

	if (!lCamera.isOpened()) { cout << "No left camera!" << endl; return -1; }
	if (!RCamera.isOpened()) { cout << "No right camera!" << endl; return -1; }

	Mat cameraMatrix[2], distCoeffs[2];

	fs.open("..\\intrinsics.yml", FileStorage::READ);
	if (fs.isOpened())
	{
		fs["cameraMatrixL"] >> cameraMatrix[0];
		fs["cameraDistcoeffL"] >> distCoeffs[0];
		fs["cameraMatrixR"] >> cameraMatrix[1];
		fs["cameraDistcoeffR"] >> distCoeffs[1];
		fs.release();
	}
	else
		cout << "Error: can not load the intrinsic parameters\n";


	Rect validRoi[2];
	Size imageSize(lCamera.get(CAP_PROP_FRAME_WIDTH), lCamera.get(CAP_PROP_FRAME_HEIGHT));

	fs.open("..\\extrinsics.yml", FileStorage::READ);
	if (fs.isOpened())
	{
		fs["R"] >> R;
		fs["T"] >> T;
		fs["Rl"] >> R1;
		fs["Rr"] >> R2;
		fs["Pl"] >> P1;
		fs["Pr"] >> P2;
		fs["Q"] >> Q;
		fs.release();
	}
	else
		cout << "Error: can not load the extrinsic parameters\n";

	stereoRectify(cameraMatrix[0], distCoeffs[0],
		cameraMatrix[1], distCoeffs[1],
		imageSize, R, T, R1, R2, P1, P2, Q,
		CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);

	cout << "Tx:" << -1 / Q.at<double>(3, 2) << endl;

	// OpenCV can handle left-right
	// or up-down camera arrangements
	bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));

	// COMPUTE AND DISPLAY RECTIFICATION
	Mat rmap[2][2];
	// IF BY CALIBRATED (BOUGUET'S METHOD)

	//Precompute maps for cv::remap()
	initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
	initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

	Mat canvas;
	double sf;
	int w, h;
	if (!isVerticalStereo)
	{
		sf = 600. / MAX(imageSize.width, imageSize.height);
		w = cvRound(imageSize.width*sf);
		h = cvRound(imageSize.height*sf);
		canvas.create(h, w * 2, CV_8UC3);
	}
	else
	{
		sf = 300. / MAX(imageSize.width, imageSize.height);
		w = cvRound(imageSize.width*sf);
		h = cvRound(imageSize.height*sf);
		canvas.create(h * 2, w, CV_8UC3);
	}

	Mat lSrc, rSrc, lImg, rImg;

	int ndisparities = 16 * 5;   /**< Range of disparity */
	int SADWindowSize = 31; /**< Size of the block window. Must be odd */
	Ptr<StereoBM> sbm = StereoBM::create(ndisparities, SADWindowSize);
	//    sbm->setMinDisparity(0);
	//    sbm->setNumDisparities(64);
	//    sbm->setTextureThreshold(10);
	//    sbm->setDisp12MaxDiff(-1);
	//    sbm->setPreFilterCap(31);
	//    sbm->setUniquenessRatio(25);
	//    sbm->setSpeckleRange(32);
	//    sbm->setSpeckleWindowSize(100);


	Ptr<StereoSGBM> sgbm = StereoSGBM::create(0, 64, 7,
		10 * 7 * 7,
		40 * 7 * 7,
		1, 63, 10, 100, 32, StereoSGBM::MODE_SGBM);


	Mat rimg, cimg;
	Mat Mask;
	while (true)
	{
		lCamera >> lSrc;
		RCamera >> rSrc;

		if (lSrc.empty() || rSrc.empty())
			continue;

		remap(lSrc, rimg, rmap[0][0], rmap[0][1], INTER_LINEAR);
		rimg.copyTo(cimg);
		Mat canvasPart1 = !isVerticalStereo ? canvas(Rect(w * 0, 0, w, h)) : canvas(Rect(0, h * 0, w, h));
		resize(cimg, canvasPart1, canvasPart1.size(), 0, 0, INTER_AREA);
		Rect vroi1(cvRound(validRoi[0].x*sf), cvRound(validRoi[0].y*sf),
			cvRound(validRoi[0].width*sf), cvRound(validRoi[0].height*sf));

		remap(rSrc, rimg, rmap[1][0], rmap[1][1], INTER_LINEAR);
		rimg.copyTo(cimg);
		Mat canvasPart2 = !isVerticalStereo ? canvas(Rect(w * 1, 0, w, h)) : canvas(Rect(0, h * 1, w, h));
		resize(cimg, canvasPart2, canvasPart2.size(), 0, 0, INTER_AREA);
		Rect vroi2 = Rect(cvRound(validRoi[1].x*sf), cvRound(validRoi[1].y*sf),
			cvRound(validRoi[1].width*sf), cvRound(validRoi[1].height*sf));

		vroi = vroi1&vroi2;

		lImg = canvasPart1(vroi).clone();
		rImg = canvasPart2(vroi).clone();

		//标记区域
		rectangle(canvasPart1, vroi1, Scalar(0, 0, 255), 3, 8);
		rectangle(canvasPart2, vroi2, Scalar(255, 0, 0), 3, 8);
		rectangle(canvasPart1, vroi, Scalar(0, 255, 0), 3, 8);

		if (!isVerticalStereo)
			for (int j = 0; j < canvas.rows; j += 32)
				line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);
		else
			for (int j = 0; j < canvas.cols; j += 32)
				line(canvas, Point(j, 0), Point(j, canvas.rows), Scalar(0, 255, 0), 1, 8);

		imshow("Rectified", canvas);
		setMouseCallback("Rectified", on_mouse, 0);

		if (lImg.empty() || rImg.empty())
		{
			cout << " --(!) Error reading images " << endl;
			return -1;
		}

		cvtColor(lImg, lImg, CV_BGR2GRAY);
		cvtColor(rImg, rImg, CV_BGR2GRAY);

		Mat imgDisparity16S = Mat(lImg.rows, lImg.cols, CV_16S);
		Mat imgDisparity8U = Mat(lImg.rows, lImg.cols, CV_8UC1);

		sbm->compute(lImg, rImg, imgDisparity16S);

		imgDisparity16S.convertTo(imgDisparity8U, CV_8UC1, 255.0 / 1000.0);
		compare(imgDisparity16S, 0, Mask, CMP_GE);
		applyColorMap(imgDisparity8U, imgDisparity8U, COLORMAP_HSV);
		Mat disparityShow;
		imgDisparity8U.copyTo(disparityShow, Mask);
		getPointClouds(imgDisparity16S, XYZ);
		imshow("BM算法视差图", disparityShow);
		setMouseCallback("BM算法视差图", on_mouse, 0);


		char c = waitKey(1);
		if (c == 27)
			break;
	}
	return 0;
}