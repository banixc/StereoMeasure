// Calibration.cpp : 定义控制台应用程序的入口点。
//
#include "stdafx.h"
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

const int boardWidth = 7;								//横向的角点数目
const int boardHeight = 6;								//纵向的角点数据
const int boardCorner = boardWidth * boardHeight;		//总的角点数据
const int frameNumber = 10;								//相机标定时需要采用的图像帧数
const int squareSize = 20;								//标定板黑白格子的大小 单位mm
const Size boardSize = Size(boardWidth, boardHeight);	

int id;													//摄像机序号

Mat intrinsic;											//相机内参数
Mat distortion_coeff;									//相机畸变参数
vector<Mat> rvecs;									    //旋转向量
vector<Mat> tvecs;										//平移向量
vector<vector<Point2f>> corners;						//各个图像找到的角点的集合 和objRealPoint 一一对应
vector<vector<Point3f>> objRealPoint;					//各副图像的角点的实际物理坐标集合


vector<Point2f> corner;									//某一副图像找到的角点


/*计算标定板上模块的实际物理坐标*/
void calRealPoint(vector<vector<Point3f>>& obj, int boardwidth, int boardheight, int imgNumber, int squaresize)
{
	vector<Point3f> imgpoint;
	for (int rowIndex = 0; rowIndex < boardheight; rowIndex++)
	{
		for (int colIndex = 0; colIndex < boardwidth; colIndex++)
		{
			imgpoint.push_back(Point3f(rowIndex * squaresize, colIndex * squaresize, 0));
		}
	}
	for (int imgIndex = 0; imgIndex < imgNumber; imgIndex++)
	{
		obj.push_back(imgpoint);
	}
}

/*设置相机的初始参数 也可以不估计*/
void guessCameraParam(void)
{
	/*分配内存*/
	intrinsic.create(3, 3, CV_64FC1);
	distortion_coeff.create(5, 1, CV_64FC1);

	/*
	fx 0 cx
	0 fy cy
	0 0  1
	*/
	/*
	intrinsic.at<double>(0, 0) = 256.8093262;   //fx		
	intrinsic.at<double>(0, 2) = 160.2826538;   //cx
	intrinsic.at<double>(1, 1) = 254.7511139;   //fy
	intrinsic.at<double>(1, 2) = 127.6264572;   //cy

	intrinsic.at<double>(0, 1) = 0;
	intrinsic.at<double>(1, 0) = 0;
	intrinsic.at<double>(2, 0) = 0;
	intrinsic.at<double>(2, 1) = 0;
	intrinsic.at<double>(2, 2) = 1;
	*/
	/*
	k1 k2 p1 p2 p3
	*/
	/*
	distortion_coeff.at<double>(0, 0) = -0.193740;  //k1
	distortion_coeff.at<double>(1, 0) = -0.378588;  //k2
	distortion_coeff.at<double>(2, 0) = 0.028980;   //p1
	distortion_coeff.at<double>(3, 0) = 0.008136;   //p2
	distortion_coeff.at<double>(4, 0) = 0;		  //p3
	*/
}

void outputCameraParam(void)
{
	/*输出数据*/
	char filename[64];
	sprintf_s(filename,64,"..\\camera%d.yml",id);
	FileStorage fs(filename, FileStorage::WRITE);
	if (fs.isOpened())
	{
		fs << "fx" << intrinsic.at<double>(0, 0) << "fy" << intrinsic.at<double>(1, 1)
			<< "cx" << intrinsic.at<double>(0, 2) << "cy" << intrinsic.at<double>(1, 2)
			<< "k1" << distortion_coeff.at<double>(0, 0)
			<< "k2" << distortion_coeff.at<double>(1, 0)
			<< "p1" << distortion_coeff.at<double>(2, 0) 
			<< "p2" << distortion_coeff.at<double>(3, 0)
			<< "p3" << distortion_coeff.at<double>(4, 0);
		fs.release();
	}
	else {
		cout << "无法打开文件: " << filename << endl;
		cout << "fx :" << intrinsic.at<double>(0, 0) << endl << "fy :" << intrinsic.at<double>(1, 1) << endl;
		cout << "cx :" << intrinsic.at<double>(0, 2) << endl << "cy :" << intrinsic.at<double>(1, 2) << endl;

		cout << "k1 :" << distortion_coeff.at<double>(0, 0) << endl;
		cout << "k2 :" << distortion_coeff.at<double>(1, 0) << endl;
		cout << "p1 :" << distortion_coeff.at<double>(2, 0) << endl;
		cout << "p2 :" << distortion_coeff.at<double>(3, 0) << endl;
		cout << "p3 :" << distortion_coeff.at<double>(4, 0) << endl;
	}
}


int main()
{
	Mat rgbImage, grayImage;
	int goodFrameCount = 0;

	cout << "请输入要标定的摄像头序列号:";
	cin >> id;

	VideoCapture capture(id);

	while (!capture.isOpened()) {
		std::cout << "无法打开该序列号的摄像头，请重试:";
		cin >> id;
		capture.open(id);
	}

	int imageWidth = capture.get(CAP_PROP_FRAME_WIDTH);
	int imageHeight = capture.get(CAP_PROP_FRAME_HEIGHT);

	cout << "按下 C 来抓取一张图片\n按下 ESC 退出程序" << endl;

	while (goodFrameCount < frameNumber) {
		capture >> rgbImage;
		cvtColor(rgbImage, grayImage, CV_BGR2GRAY);
		imshow("GRAY", grayImage);

		char c = waitKey(10);

		if (c == 27) //退出
			return 0;
		else if (c == 'c') {//拍照并检测
			bool isFind = findChessboardCorners(grayImage, boardSize, corner,
				CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
				+ CALIB_CB_FAST_CHECK);
			if (isFind == true)	//所有角点都被找到 说明这幅图像是可行的
			{
				/*
				Size(5,5) 搜索窗口的一半大小
				Size(-1,-1) 死区的一半尺寸
				TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 20, 0.1)迭代终止条件
				*/
				cornerSubPix(grayImage, corner, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 20, 0.1));
				drawChessboardCorners(rgbImage, boardSize, corner, isFind);
				imshow("棋盘格", rgbImage);
				corners.push_back(corner);

				goodFrameCount++;
				cout << "第 " << goodFrameCount << "/" << frameNumber <<" 张图片已采集" << endl;
			}
			else
			{
				cout << "无法找到所有角点，请重试！" << endl;
			}
		}
	}


	/*
	图像采集完毕 接下来开始摄像头的校正
	calibrateCamera()
	输入参数 objectPoints  角点的实际物理坐标
	imagePoints   角点的图像坐标
	imageSize	   图像的大小
	输出参数
	cameraMatrix  相机的内参矩阵
	distCoeffs	   相机的畸变参数
	rvecs		   旋转矢量(外参数)
	tvecs		   平移矢量(外参数）
	*/

	/*设置实际初始参数 根据calibrateCamera来 如果flag = 0 也可以不进行设置*/
	guessCameraParam();

	/*计算实际的校正点的三维坐标*/
	calRealPoint(objRealPoint, boardWidth, boardHeight, frameNumber, squareSize);

	/*标定摄像头*/
	calibrateCamera(objRealPoint, corners, Size(imageWidth, imageHeight), intrinsic, distortion_coeff, rvecs, tvecs, 0);

	/*保存并输出参数*/
	outputCameraParam();

	destroyAllWindows();

	/*显示畸变校正效果*/
	Mat cImage;
	while (true) {
		capture >> rgbImage;
		undistort(rgbImage, cImage, intrinsic, distortion_coeff);
		imshow("修正后的图像", cImage);
		char c = waitKey(1);
		if (c == 27)
			break;
	}
	return 0;
}
