// StereoCalibration.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include <opencv2/opencv.hpp>
#include <iostream>

//在进行双目摄像头的标定之前，最好事先分别对两个摄像头进行单目视觉的标定 
//分别确定两个摄像头的内参矩阵，然后再开始进行双目摄像头的标定
//在此例程中是先对两个摄像头进行单独标定(见上一篇单目标定文章)，然后在进行立体标定


using namespace std;
using namespace cv;

const int imageWidth = 640;								//摄像头的分辨率
const int imageHeight = 480;
const int boardWidth = 7;								//横向的角点数目
const int boardHeight = 7;								//纵向的角点数据
const int boardCorner = boardWidth * boardHeight;		//总的角点数据
const int frameNumber = 10;								//相机标定时需要采用的图像帧数
const int squareSize = 20;								//标定板黑白格子的大小 单位mm
const Size boardSize = Size(boardWidth, boardHeight);	//
Size imageSize = Size(imageWidth, imageHeight);

int lid, rid;
bool saved = false;

Mat R, T, E, F;											//R 旋转矢量 T平移矢量 E本征矩阵 F基础矩阵
vector<Mat> rvecs;									    //旋转向量
vector<Mat> tvecs;										//平移向量
vector<vector<Point2f>> imagePointL;				    //左边摄像机所有照片角点的坐标集合
vector<vector<Point2f>> imagePointR;					//右边摄像机所有照片角点的坐标集合
vector<vector<Point3f>> objRealPoint;					//各副图像的角点的实际物理坐标集合


vector<Point2f> cornerL;								//左边摄像机某一照片角点坐标集合
vector<Point2f> cornerR;								//右边摄像机某一照片角点坐标集合

Mat rgbImageL, grayImageL;
Mat rgbImageR, grayImageR;

Mat Rl, Rr, Pl, Pr, Q;									//校正旋转矩阵R，投影矩阵P 重投影矩阵Q (下面有具体的含义解释）	
Mat mapLx, mapLy, mapRx, mapRy;							//映射表
Rect validROIL, validROIR;								//图像校正之后，会对图像进行裁剪，这里的validROI就是指裁剪之后的区域
Mat cameraMatrixL, distCoeffL, cameraMatrixR, distCoeffR;


// 从文件中获取相机标定参数
bool loadCameraParams(void) {

	char filename[64];
	double fx, cx, fy, cy, k1, k2, p1, p2, p3;
	FileStorage fs;

	sprintf_s(filename, 64, "..\\Camera%d.yml", lid);
	fs.open(filename, FileStorage::READ);

	if (!fs.isOpened())
		return false;

	fs["fx"] >> fx;
	fs["cx"] >> cx;
	fs["fy"] >> fy;
	fs["cy"] >> cy;
	fs["k1"] >> k1;
	fs["k2"] >> k2;
	fs["p1"] >> p1;
	fs["p2"] >> p2;
	fs["p3"] >> p3;

	/*
	事先标定好的相机的内参矩阵
	fx 0 cx
	0 fy cy
	0 0  1
	*/

	cameraMatrixL = (Mat_<double>(3, 3) << fx, 0, cx,
		0, fy, cy,
		0, 0, 1);
	distCoeffL = (Mat_<double>(5, 1) << k1, k2, p1, p2, p3);

	sprintf_s(filename, 64, "..\\Camera%d.yml", rid);
	fs.open(filename, FileStorage::READ);

	if (!fs.isOpened())
		return false;

	fs["fx"] >> fx;
	fs["cx"] >> cx;
	fs["fy"] >> fy;
	fs["cy"] >> cy;
	fs["k1"] >> k1;
	fs["k2"] >> k2;
	fs["p1"] >> p1;
	fs["p2"] >> p2;
	fs["p3"] >> p3;

	cameraMatrixR = (Mat_<double>(3, 3) << fx, 0, cx,
		0, fy, cy,
		0, 0, 1);
	distCoeffR = (Mat_<double>(5, 1) << k1, k2, p1, p2, p3);
	return true;
}

/*计算标定板上模块的实际物理坐标*/
void calRealPoint(vector<vector<Point3f>>& obj, int boardwidth, int boardheight, int imgNumber, int squaresize)
{
	//	Mat imgpoint(boardheight, boardwidth, CV_32FC3,Scalar(0,0,0));
	vector<Point3f> imgpoint;
	for (int rowIndex = 0; rowIndex < boardheight; rowIndex++)
	{
		for (int colIndex = 0; colIndex < boardwidth; colIndex++)
		{
			//	imgpoint.at<Vec3f>(rowIndex, colIndex) = Vec3f(rowIndex * squaresize, colIndex*squaresize, 0);
			imgpoint.push_back(Point3f(rowIndex * squaresize, colIndex * squaresize, 0));
		}
	}
	for (int imgIndex = 0; imgIndex < imgNumber; imgIndex++)
	{
		obj.push_back(imgpoint);
	}
}

void outputCameraParam(void)
{
	
	/*保存数据*/
	/*输出数据*/
	FileStorage fs("..\\intrinsics.yml", FileStorage::WRITE);
	if (fs.isOpened())
	{
		fs << "cameraMatrixL" << cameraMatrixL << "cameraDistcoeffL" << distCoeffL << "cameraMatrixR" << cameraMatrixR << "cameraDistcoeffR" << distCoeffR;
		fs.release();
		//cout << "cameraMatrixL=:" << cameraMatrixL << endl << "cameraDistcoeffL=:" << distCoeffL << endl << "cameraMatrixR=:" << cameraMatrixR << endl << "cameraDistcoeffR=:" << distCoeffR << endl;
	}
	else
	{
		cout << "Error: can not save the intrinsics!!!!!" << endl;
	}

	fs.open("..\\extrinsics.yml", FileStorage::WRITE);
	if (fs.isOpened())
	{
		fs << "R" << R << "T" << T << "Rl" << Rl << "Rr" << Rr << "Pl" << Pl << "Pr" << Pr << "Q" << Q;
		//cout << "R=" << R << endl << "T=" << T << endl << "Rl=" << Rl << endl << "Rr=" << Rr << endl << "Pl=" << Pl << endl << "Pr=" << Pr << endl << "Q=" << Q << endl;
		fs.release();
	}
	else
		cout << "Error: can not save the extrinsic parameters\n";
	saved = true;
}


int main()
{
	cout << "进行双目标定，请先确认已进行单目标定\n请输入左摄像头和右摄像头的ID：";
	cin >> lid >> rid;
	VideoCapture lCapture(lid);
	VideoCapture rCapture(rid);

	while (!lCapture.isOpened() || !rCapture.isOpened()) {
		cout << "左摄像头打开：" << lCapture.isOpened() << " 右摄像头打开：" << rCapture.isOpened() << " 请重试：";
		cin >> lid >> rid;
		lCapture.open(lid);
		rCapture.open(rid);
	}

	if (!loadCameraParams()) {
		cout << "Load Params Fail, Please Check Files: Camera" << lid << ".yml And Camera" << rid << ".yml are exist!";
	}

	cout << "按下 c 来抓取一张图片\n按下 ESC 退出程序" << endl;

	int goodFrameCount = 0;
	while (goodFrameCount < frameNumber) {
		lCapture >> rgbImageL;
		rCapture >> rgbImageR;
	
		cvtColor(rgbImageL, grayImageL, CV_BGR2GRAY);
		cvtColor(rgbImageR, grayImageR, CV_BGR2GRAY);
		imshow("Camera L", rgbImageL);
		imshow("Camera R", rgbImageR);

		char c = waitKey(10);

		if (c == 27) //退出
			return 0;
		else if (c == 'c') {//拍照并检测

			bool isFindL, isFindR;

			isFindL = findChessboardCorners(rgbImageL, boardSize, cornerL);
			isFindR = findChessboardCorners(rgbImageR, boardSize, cornerR);
			if (isFindL && isFindR )	 //如果两幅图像都找到了所有的角点 则说明这两幅图像是可行的
			{
				/*
				Size(5,5) 搜索窗口的一半大小
				Size(-1,-1) 死区的一半尺寸
				TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 20, 0.1)迭代终止条件
				*/
				cornerSubPix(grayImageL, cornerL, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 20, 0.1));
				drawChessboardCorners(rgbImageL, boardSize, cornerL, isFindL);
				imshow("Chessboard L", rgbImageL);
				imagePointL.push_back(cornerL);


				cornerSubPix(grayImageR, cornerR, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 20, 0.1));
				drawChessboardCorners(rgbImageR, boardSize, cornerR, isFindR);
				imshow("Chessboard R", rgbImageR);
				imagePointR.push_back(cornerR);

				/*
				本来应该判断这两幅图像是不是好的，如果可以匹配的话才可以用来标定
				但是在这个例程当中，用的图像是系统自带的图像，都是可以匹配成功的。
				所以这里就没有判断
				*/
				goodFrameCount++;

				cout << "The " << goodFrameCount << "/" << frameNumber << " image is good" << endl;
			}
			else
			{
				cout << "The " << (isFindL?"R":"L") << " image is bad please try again" << endl;
			}

		}

	}

	/*
	计算实际的校正点的三维坐标
	根据实际标定格子的大小来设置
	*/
	calRealPoint(objRealPoint, boardWidth, boardHeight, frameNumber, squareSize);
	cout << "cal real successful" << endl;

	/*
	标定摄像头
	由于左右摄像机分别都经过了单目标定
	所以在此处选择flag = CALIB_USE_INTRINSIC_GUESS
	*/
	double rms = stereoCalibrate(objRealPoint, imagePointL, imagePointR,
		cameraMatrixL, distCoeffL,
		cameraMatrixR, distCoeffR,
		Size(imageWidth, imageHeight), R, T, E, F,
		CALIB_USE_INTRINSIC_GUESS,
		TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5));

	cout << "Stereo Calibration done with RMS error = " << rms << endl;

	/*
	立体校正的时候需要两幅图像共面并且行对准 以使得立体匹配更加的可靠
	使得两幅图像共面的方法就是把两个摄像头的图像投影到一个公共成像面上，这样每幅图像从本图像平面投影到公共图像平面都需要一个旋转矩阵R
	stereoRectify 这个函数计算的就是从图像平面投影都公共成像平面的旋转矩阵Rl,Rr。 Rl,Rr即为左右相机平面行对准的校正旋转矩阵。
	左相机经过Rl旋转，右相机经过Rr旋转之后，两幅图像就已经共面并且行对准了。
	其中Pl,Pr为两个相机的投影矩阵，其作用是将3D点的坐标转换到图像的2D点的坐标:P*[X Y Z 1]' =[x y w]
	Q矩阵为重投影矩阵，即矩阵Q可以把2维平面(图像平面)上的点投影到3维空间的点:Q*[x y d 1] = [X Y Z W]。其中d为左右两幅图像的时差
	*/
	stereoRectify(cameraMatrixL, distCoeffL, cameraMatrixR, distCoeffR, imageSize, R, T, Rl, Rr, Pl, Pr, Q,
		CALIB_ZERO_DISPARITY, -1, imageSize, &validROIL, &validROIR);
	/*
	根据stereoRectify 计算出来的R 和 P 来计算图像的映射表 mapx,mapy
	mapx,mapy这两个映射表接下来可以给remap()函数调用，来校正图像，使得两幅图像共面并且行对准
	ininUndistortRectifyMap()的参数newCameraMatrix就是校正后的摄像机矩阵。在openCV里面，校正后的计算机矩阵Mrect是跟投影矩阵P一起返回的。
	所以我们在这里传入投影矩阵P，此函数可以从投影矩阵P中读出校正后的摄像机矩阵
	*/
	initUndistortRectifyMap(cameraMatrixL, distCoeffL, Rl, Pl, imageSize, CV_32FC1, mapLx, mapLy);
	initUndistortRectifyMap(cameraMatrixR, distCoeffR, Rr, Pr, imageSize, CV_32FC1, mapRx, mapRy);

	/*保存并输出数据*/
	outputCameraParam();


	destroyWindow("Camera L");
	destroyWindow("Camera R");
	destroyWindow("Chessboard L");
	destroyWindow("Chessboard R");

	Mat rectifyImageL, rectifyImageR;

	while (true) {
		lCapture >> rectifyImageL;
		rCapture >> rectifyImageR;

		remap(rectifyImageL, rectifyImageL, mapLx, mapLy, INTER_LINEAR);
		remap(rectifyImageR, rectifyImageR, mapRx, mapRy, INTER_LINEAR);

		/*
		把校正结果显示出来
		把左右两幅图像显示到同一个画面上
		这里只显示了最后一副图像的校正结果。并没有把所有的图像都显示出来
		*/
		Mat canvas;
		double sf;
		int w, h;
		sf = 600. / MAX(imageSize.width, imageSize.height);
		w = cvRound(imageSize.width * sf);
		h = cvRound(imageSize.height * sf);
		canvas.create(h, w * 2, CV_8UC3);

		/*左图像画到画布上*/
		Mat canvasPart = canvas(Rect(w * 0, 0, w, h));								//得到画布的一部分
		resize(rectifyImageL, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);		//把图像缩放到跟canvasPart一样大小
		Rect vroiL(cvRound(validROIL.x*sf), cvRound(validROIL.y*sf),				//获得被截取的区域	
			cvRound(validROIL.width*sf), cvRound(validROIL.height*sf));
		rectangle(canvasPart, vroiL, Scalar(0, 0, 255), 3, 8);						//画上一个矩形

		//cout << "Painted ImageL" << endl;

																					/*右图像画到画布上*/
		canvasPart = canvas(Rect(w, 0, w, h));										//获得画布的另一部分
		resize(rectifyImageR, canvasPart, canvasPart.size(), 0, 0, INTER_LINEAR);
		Rect vroiR(cvRound(validROIR.x * sf), cvRound(validROIR.y*sf),
			cvRound(validROIR.width * sf), cvRound(validROIR.height * sf));
		rectangle(canvasPart, vroiR, Scalar(0, 255, 0), 3, 8);

		//cout << "Painted ImageR" << endl;

		/*画上对应的线条*/
		for (int i = 0; i < canvas.rows; i += 16)
			line(canvas, Point(0, i), Point(canvas.cols, i), Scalar(0, 255, 0), 1, 8);

		imshow("Rectified", canvas);
		char c = waitKey(1);
		if (c == 27)
			break;
	}

	//cvtColor(grayImageL, rectifyImageL, CV_GRAY2BGR);
	//cvtColor(grayImageR, rectifyImageR, CV_GRAY2BGR);

	//imshow("Before Rectify L", rectifyImageL);
	//imshow("Before Rectify R", rectifyImageR);
	/*
	经过remap之后，左右相机的图像已经共面并且行对准了
	*/


	//imshow("After Rectify L", rectifyImageL);
	//imshow("After Rectify R", rectifyImageR);



	return 0;
}
