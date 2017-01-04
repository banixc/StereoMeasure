// StereoCalibration.cpp : �������̨Ӧ�ó������ڵ㡣
//

#include "stdafx.h"
#include <opencv2/opencv.hpp>
#include <iostream>

//�ڽ���˫Ŀ����ͷ�ı궨֮ǰ��������ȷֱ����������ͷ���е�Ŀ�Ӿ��ı궨 
//�ֱ�ȷ����������ͷ���ڲξ���Ȼ���ٿ�ʼ����˫Ŀ����ͷ�ı궨
//�ڴ����������ȶ���������ͷ���е����궨(����һƪ��Ŀ�궨����)��Ȼ���ڽ�������궨


using namespace std;
using namespace cv;

const int imageWidth = 640;								//����ͷ�ķֱ���
const int imageHeight = 480;
const int boardWidth = 7;								//����Ľǵ���Ŀ
const int boardHeight = 7;								//����Ľǵ�����
const int boardCorner = boardWidth * boardHeight;		//�ܵĽǵ�����
const int frameNumber = 10;								//����궨ʱ��Ҫ���õ�ͼ��֡��
const int squareSize = 20;								//�궨��ڰ׸��ӵĴ�С ��λmm
const Size boardSize = Size(boardWidth, boardHeight);	//
Size imageSize = Size(imageWidth, imageHeight);

int lid, rid;

Mat R, T, E, F;											//R ��תʸ�� Tƽ��ʸ�� E�������� F��������
vector<Mat> rvecs;									    //��ת����
vector<Mat> tvecs;										//ƽ������
vector<vector<Point2f>> imagePointL;				    //��������������Ƭ�ǵ�����꼯��
vector<vector<Point2f>> imagePointR;					//�ұ������������Ƭ�ǵ�����꼯��
vector<vector<Point3f>> objRealPoint;					//����ͼ��Ľǵ��ʵ���������꼯��


vector<Point2f> cornerL;								//��������ĳһ��Ƭ�ǵ����꼯��
vector<Point2f> cornerR;								//�ұ������ĳһ��Ƭ�ǵ����꼯��

Mat rgbImageL, grayImageL;
Mat rgbImageR, grayImageR;

Mat Rl, Rr, Pl, Pr, Q;									//У����ת����R��ͶӰ����P ��ͶӰ����Q (�����о���ĺ�����ͣ�	
Mat mapLx, mapLy, mapRx, mapRy;							//ӳ���
Rect validROIL, validROIR;								//ͼ��У��֮�󣬻��ͼ����вü��������validROI����ָ�ü�֮�������
Mat cameraMatrixL, distCoeffL, cameraMatrixR, distCoeffR;


// ���ļ��л�ȡ����궨����
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
	���ȱ궨�õ�������ڲξ���
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

/*����궨����ģ���ʵ����������*/
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
	/*��������*/
	/*�������*/
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
}


int main()
{
	cout << "����˫Ŀ�궨������ȷ���ѽ��е�Ŀ�궨\n������������ͷ��������ͷ��ID��";
	cin >> lid >> rid;
	VideoCapture lCapture(lid);
	VideoCapture rCapture(rid);

	while (!lCapture.isOpened() || !rCapture.isOpened()) {
		cout << "������ͷ�򿪣�" << lCapture.isOpened() << " ������ͷ�򿪣�" << rCapture.isOpened() << " �����ԣ�";
		cin >> lid >> rid;
		lCapture.open(lid);
		rCapture.open(rid);
	}

	if (!loadCameraParams()) {
		cout << "Load Params Fail, Please Check Files: Camera" << lid << ".yml And Camera" << rid << ".yml are exist!";
	}

	cout << "���� c ��ץȡһ��ͼƬ\n���� ESC �˳�����" << endl;

	int goodFrameCount = 0;
	while (goodFrameCount < frameNumber) {
		lCapture >> rgbImageL;
		rCapture >> rgbImageR;
	
		cvtColor(rgbImageL, grayImageL, CV_BGR2GRAY);
		cvtColor(rgbImageR, grayImageR, CV_BGR2GRAY);
		imshow("Camera L", rgbImageL);
		imshow("Camera R", rgbImageR);

		char c = waitKey(10);

		if (c == 27) //�˳�
			return 0;
		else if (c == 'c') {//���ղ����

			bool isFindL, isFindR;

			isFindL = findChessboardCorners(rgbImageL, boardSize, cornerL);
			isFindR = findChessboardCorners(rgbImageR, boardSize, cornerR);
			if (isFindL && isFindR )	 //�������ͼ���ҵ������еĽǵ� ��˵��������ͼ���ǿ��е�
			{
				/*
				Size(5,5) �������ڵ�һ���С
				Size(-1,-1) ������һ��ߴ�
				TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 20, 0.1)������ֹ����
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
				����Ӧ���ж�������ͼ���ǲ��Ǻõģ��������ƥ��Ļ��ſ��������궨
				������������̵��У��õ�ͼ����ϵͳ�Դ���ͼ�񣬶��ǿ���ƥ��ɹ��ġ�
				���������û���ж�
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
	����ʵ�ʵ�У�������ά����
	����ʵ�ʱ궨���ӵĴ�С������
	*/
	calRealPoint(objRealPoint, boardWidth, boardHeight, frameNumber, squareSize);
	cout << "cal real successful" << endl;

	/*
	�궨����ͷ
	��������������ֱ𶼾����˵�Ŀ�궨
	�����ڴ˴�ѡ��flag = CALIB_USE_INTRINSIC_GUESS
	*/
	double rms = stereoCalibrate(objRealPoint, imagePointL, imagePointR,
		cameraMatrixL, distCoeffL,
		cameraMatrixR, distCoeffR,
		Size(imageWidth, imageHeight), R, T, E, F,
		CALIB_USE_INTRINSIC_GUESS,
		TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5));

	cout << "Stereo Calibration done with RMS error = " << rms << endl;

	/*
	����У����ʱ����Ҫ����ͼ���沢���ж�׼ ��ʹ������ƥ����ӵĿɿ�
	ʹ������ͼ����ķ������ǰ���������ͷ��ͼ��ͶӰ��һ�������������ϣ�����ÿ��ͼ��ӱ�ͼ��ƽ��ͶӰ������ͼ��ƽ�涼��Ҫһ����ת����R
	stereoRectify �����������ľ��Ǵ�ͼ��ƽ��ͶӰ����������ƽ�����ת����Rl,Rr�� Rl,Rr��Ϊ�������ƽ���ж�׼��У����ת����
	���������Rl��ת�����������Rr��ת֮������ͼ����Ѿ����沢���ж�׼�ˡ�
	����Pl,PrΪ���������ͶӰ�����������ǽ�3D�������ת����ͼ���2D�������:P*[X Y Z 1]' =[x y w]
	Q����Ϊ��ͶӰ���󣬼�����Q���԰�2άƽ��(ͼ��ƽ��)�ϵĵ�ͶӰ��3ά�ռ�ĵ�:Q*[x y d 1] = [X Y Z W]������dΪ��������ͼ���ʱ��
	*/
	stereoRectify(cameraMatrixL, distCoeffL, cameraMatrixR, distCoeffR, imageSize, R, T, Rl, Rr, Pl, Pr, Q,
		CALIB_ZERO_DISPARITY, -1, imageSize, &validROIL, &validROIR);
	/*
	����stereoRectify ���������R �� P ������ͼ���ӳ��� mapx,mapy
	mapx,mapy������ӳ������������Ը�remap()�������ã���У��ͼ��ʹ������ͼ���沢���ж�׼
	ininUndistortRectifyMap()�Ĳ���newCameraMatrix����У����������������openCV���棬У����ļ��������Mrect�Ǹ�ͶӰ����Pһ�𷵻صġ�
	�������������ﴫ��ͶӰ����P���˺������Դ�ͶӰ����P�ж���У��������������
	*/
	initUndistortRectifyMap(cameraMatrixL, distCoeffL, Rl, Pr, imageSize, CV_32FC1, mapLx, mapLy);
	initUndistortRectifyMap(cameraMatrixR, distCoeffR, Rr, Pr, imageSize, CV_32FC1, mapRx, mapRy);

	destroyWindow("Camera L");
	destroyWindow("Camera R");
	destroyWindow("Chessboard L");
	destroyWindow("Chessboard R");

	Mat rectifyImageL, rectifyImageR;
	cvtColor(grayImageL, rectifyImageL, CV_GRAY2BGR);
	cvtColor(grayImageR, rectifyImageR, CV_GRAY2BGR);

	imshow("Before Rectify L", rectifyImageL);
	imshow("Before Rectify R", rectifyImageR);
	/*
	����remap֮�����������ͼ���Ѿ����沢���ж�׼��
	*/
	remap(rectifyImageL, rectifyImageL, mapLx, mapLy, INTER_LINEAR);
	remap(rectifyImageR, rectifyImageR, mapRx, mapRy, INTER_LINEAR);

	//imshow("After Rectify L", rectifyImageL);
	//imshow("After Rectify R", rectifyImageR);

	/*���沢�������*/
	outputCameraParam();

	/*
	��У�������ʾ����
	����������ͼ����ʾ��ͬһ��������
	����ֻ��ʾ�����һ��ͼ���У���������û�а����е�ͼ����ʾ����
	*/
	Mat canvas;
	double sf;
	int w, h;
	sf = 600. / MAX(imageSize.width, imageSize.height);
	w = cvRound(imageSize.width * sf);
	h = cvRound(imageSize.height * sf);
	canvas.create(h, w * 2, CV_8UC3);

	/*��ͼ�񻭵�������*/
	Mat canvasPart = canvas(Rect(w * 0, 0, w, h));								//�õ�������һ����
	resize(rectifyImageL, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);		//��ͼ�����ŵ���canvasPartһ����С
	Rect vroiL(cvRound(validROIL.x*sf), cvRound(validROIL.y*sf),				//��ñ���ȡ������	
		cvRound(validROIL.width*sf), cvRound(validROIL.height*sf));
	rectangle(canvasPart, vroiL, Scalar(0, 0, 255), 3, 8);						//����һ������

	//cout << "Painted ImageL" << endl;

	/*��ͼ�񻭵�������*/
	canvasPart = canvas(Rect(w, 0, w, h));										//��û�������һ����
	resize(rectifyImageR, canvasPart, canvasPart.size(), 0, 0, INTER_LINEAR);
	Rect vroiR(cvRound(validROIR.x * sf), cvRound(validROIR.y*sf),
		cvRound(validROIR.width * sf), cvRound(validROIR.height * sf));
	rectangle(canvasPart, vroiR, Scalar(0, 255, 0), 3, 8);

	//cout << "Painted ImageR" << endl;

	/*���϶�Ӧ������*/
	for (int i = 0; i < canvas.rows; i += 16)
		line(canvas, Point(0, i), Point(canvas.cols, i), Scalar(0, 255, 0), 1, 8);

	imshow("Rectified", canvas);
	waitKey(0);
	return 0;
}