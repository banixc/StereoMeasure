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

int imageWidth;											//����ͷ�ķֱ���
int imageHeight;
const int boardWidth = 7;								//����Ľǵ���Ŀ
const int boardHeight = 6;								//����Ľǵ�����
const int boardCorner = boardWidth * boardHeight;		//�ܵĽǵ�����
const int frameNumber = 10;								//����궨ʱ��Ҫ���õ�ͼ��֡��
const int squareSize = 20;								//�궨��ڰ׸��ӵĴ�С ��λmm
const Size boardSize = Size(boardWidth, boardHeight);	//
Size imageSize;

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
Rect validROIL, validROIR, validROI;					//ͼ��У��֮�󣬻��ͼ����вü��������validROI����ָ�ü�֮�������
Mat cameraMatrixL, distCoeffL, cameraMatrixR, distCoeffR;
Mat Mask;
VideoCapture lCapture, rCapture;						//���������Դ

Ptr<StereoBM> sbm = StereoBM::create(16 * 5, 31);

//    sbm->setMinDisparity(0);
//    sbm->setNumDisparities(64);
//    sbm->setTextureThreshold(10);
//    sbm->setDisp12MaxDiff(-1);
//    sbm->setPreFilterCap(31);
//    sbm->setUniquenessRatio(25);
//    sbm->setSpeckleRange(32);
//    sbm->setSpeckleWindowSize(100);



// ���ļ��л�ȡ����궨����
bool loadCameraParams(void) {

	char filename[64];
	double fx, cx, fy, cy, k1, k2, p1, p2, p3;
	FileStorage fs;

	sprintf_s(filename, 64, "..\\camera%d.yml", lid);
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

/*���ļ��м���˫Ŀ�궨����*/
bool loadStereoCalibrationParams(void) {

	FileStorage fs;

	fs.open("..\\CameraID.yml", FileStorage::READ);

	if (!fs.isOpened())
		return false;
	fs["LID"] >> lid;
	fs["RID"] >> rid;
	fs["W"] >> imageWidth;
	fs["H"] >> imageHeight;
	imageSize = Size(imageWidth, imageHeight);

	fs.open("..\\intrinsics.yml", FileStorage::READ);

	if (!fs.isOpened())
		return false;

	fs["cameraMatrixL"] >> cameraMatrixL;
	fs["cameraDistcoeffL"] >> distCoeffL;
	fs["cameraMatrixR"] >> cameraMatrixR;
	fs["cameraDistcoeffR"] >> distCoeffR;
	fs.release();

	fs.open("..\\extrinsics.yml", FileStorage::READ);
	if (!fs.isOpened())
		return false;

	fs["R"] >> R;
	fs["T"] >> T;
	fs["Rl"] >> Rl;
	fs["Rr"] >> Rr;
	fs["Pl"] >> Pl;
	fs["Pr"] >> Pr;
	fs["Q"] >> Q;
	fs.release();


	fs.open("..\\remap.yml", FileStorage::READ);
	if (!fs.isOpened())
		return false;

	fs["mapLx"] >> mapLx;
	fs["mapLy"] >> mapLy;
	fs["mapRx"] >> mapRx;
	fs["mapRy"] >> mapRy;
	fs.release();

	return true;

}

/*����궨����ģ���ʵ����������*/
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

void outputCameraParam(void)
{
	
	/*��������*/
	FileStorage fs;
	
	fs.open("..\\CameraID.yml", FileStorage::WRITE);
	if (fs.isOpened()) {
		fs << "LID" << lid << "RID" << rid << "W" << imageWidth << "H" << imageHeight;
		fs.release();
	}

	fs.open("..\\intrinsics.yml", FileStorage::WRITE);
	if (fs.isOpened())
	{
		fs << "cameraMatrixL" << cameraMatrixL << "cameraDistcoeffL" << distCoeffL << "cameraMatrixR" << cameraMatrixR << "cameraDistcoeffR" << distCoeffR;
		fs.release();
	}
	else
	{
		cout << "Error: can not save the intrinsics!!!!!" << endl;
	}

	fs.open("..\\extrinsics.yml", FileStorage::WRITE);
	if (fs.isOpened())
	{
		fs << "R" << R << "T" << T << "Rl" << Rl << "Rr" << Rr << "Pl" << Pl << "Pr" << Pr << "Q" << Q;
		fs.release();
	}
	else
		cout << "Error: can not save the extrinsic parameters\n";

	fs.open("..\\remap.yml", FileStorage::WRITE);
	if (fs.isOpened())
	{
		fs << "mapLx" << mapLx << "mapLy" << mapLy << "mapRx" << mapRx << "mapRy" << mapRy;
		fs.release();
	}
	else
		cout << "Error: can not save the remap parameters\n";
}

void showBM(Mat& l, Mat& r) {


	Mat imgDisparity16S = Mat(l.rows, l.cols, CV_16S);
	Mat imgDisparity8U = Mat(l.rows, l.cols, CV_8UC1);

	sbm->compute(l, r, imgDisparity16S);

	imgDisparity16S.convertTo(imgDisparity8U, CV_8UC1, 255.0 / 1000.0);
	compare(imgDisparity16S, 0, Mask, CMP_GE);
	applyColorMap(imgDisparity8U, imgDisparity8U, COLORMAP_HSV);
	Mat disparityShow;
	imgDisparity8U.copyTo(disparityShow, Mask);
	imshow("BM", disparityShow);

}


void showRectifyImage(void) {

	destroyAllWindows();

	Mat rectifyImageL, rectifyImageR;

	while (true) {
		lCapture >> rectifyImageL;
		rCapture >> rectifyImageR;

		remap(rectifyImageL, rectifyImageL, mapLx, mapLy, INTER_LINEAR);
		remap(rectifyImageR, rectifyImageR, mapRx, mapRy, INTER_LINEAR);
		
		/*
		��У�������ʾ����
		����������ͼ����ʾ��ͬһ��������
		*/
		Mat canvas;
		double sf;
		int w, h;
		sf = 640. / MAX(imageSize.width, imageSize.height);
		w = cvRound(imageSize.width * sf);
		h = cvRound(imageSize.height * sf);
		canvas.create(h, w * 2, CV_8UC3);

		/*��ͼ�񻭵�������*/
		Mat canvasPartL = canvas(Rect(w * 0, 0, w, h));									//�õ�������һ����
		resize(rectifyImageL, canvasPartL, canvasPartL.size(), 0, 0, INTER_AREA);		//��ͼ�����ŵ���canvasPartһ����С
		Rect vroiL(cvRound(validROIL.x*sf), cvRound(validROIL.y*sf),					//��ñ���ȡ������	
			cvRound(validROIL.width*sf), cvRound(validROIL.height*sf));
		rectangle(canvasPartL, vroiL, Scalar(0, 0, 255), 3, 8);							//����һ������

		/*��ͼ�񻭵�������*/
		Mat canvasPartR = canvas(Rect(w, 0, w, h));										//��û�������һ����
		resize(rectifyImageR, canvasPartR, canvasPartR.size(), 0, 0, INTER_LINEAR);
		Rect vroiR(cvRound(validROIR.x * sf), cvRound(validROIR.y*sf),
			cvRound(validROIR.width * sf), cvRound(validROIR.height * sf));
		rectangle(canvasPartR, vroiR, Scalar(0, 255, 0), 3, 8);

		validROI = vroiL & vroiR;

		if (validROI.size().area() == 0) {
			cout << "ROI EMPTY" << vroiL.size().area() << " " << vroiR.size().area() << endl;
			system("pause");
			break;
		}

		Mat lImg = canvasPartL(validROI).clone();
		Mat rImg = canvasPartR(validROI).clone();

		/*���϶�Ӧ������*/
		for (int i = 0; i < canvas.rows; i += 16)
			line(canvas, Point(0, i), Point(canvas.cols, i), Scalar(0, 255, 0), 1, 8);

		imshow("RECTIFIED", canvas);

		cvtColor(lImg, lImg, CV_BGR2GRAY);
		cvtColor(rImg, rImg, CV_BGR2GRAY);

		showBM(lImg, rImg);

		char c = waitKey(1);
		if (c == 27)
			break;
	}
}



int main()
{

	cout << "����˫Ŀ�궨������ȷ���ѽ��е�Ŀ�궨\n������������ͷ��������ͷ��ID��";
	cin >> lid >> rid;
	lCapture.open(lid);
	rCapture.open(rid);

	while (!lCapture.isOpened() || !rCapture.isOpened()) {
		cout << "������ͷ�򿪣�" << lCapture.isOpened() << " ������ͷ�򿪣�" << rCapture.isOpened() << " �����ԣ�";
		cin >> lid >> rid;
		lCapture.open(lid);
		rCapture.open(rid);
	}

	imageWidth = lCapture.get(CAP_PROP_FRAME_WIDTH);
	imageHeight = lCapture.get(CAP_PROP_FRAME_HEIGHT);
	imageSize = Size(imageWidth, imageHeight);

	if (!loadCameraParams()) {
		cout << "�޷��������е����������";
		return -1;
	}

	cout << "���� C ��ץȡһ��ͼƬ\n���� ESC �˳�����" << endl;

	int goodFrameCount = 0;
	while (goodFrameCount < frameNumber) {
		lCapture >> rgbImageL;
		rCapture >> rgbImageR;
	
		cvtColor(rgbImageL, grayImageL, CV_BGR2GRAY);
		cvtColor(rgbImageR, grayImageR, CV_BGR2GRAY);
		imshow("Camera L", grayImageL);
		imshow("Camera R", grayImageR);

		char c = waitKey(10);

		if (c == 27) //�˳�
			return 0;
		else if (c == 'c') {//���ղ����
			bool isFindL = findChessboardCorners(grayImageL, boardSize, cornerL);
			bool isFindR = findChessboardCorners(grayImageR, boardSize, cornerR);
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

				goodFrameCount++;

				cout << "�� " << goodFrameCount << "/" << frameNumber << " ��ͼƬ�Ѳɼ�" << endl;
			}
			else
			{
				cout << (isFindL?"��":"��") << "ͼ�޷��ҵ����нǵ㣬�����ԣ�" << endl;
			}

		}

	}

	destroyAllWindows();

	/*
	����ʵ�ʵ�У�������ά����
	����ʵ�ʱ궨���ӵĴ�С������
	*/
	calRealPoint(objRealPoint, boardWidth, boardHeight, frameNumber, squareSize);

	/*
	�궨����ͷ
	��������������ֱ𶼾����˵�Ŀ�궨
	�����ڴ˴�ѡ��flag = CALIB_USE_INTRINSIC_GUESS
	*/
	double rms = stereoCalibrate(objRealPoint, imagePointL, imagePointR,
		cameraMatrixL, distCoeffL,
		cameraMatrixR, distCoeffR,
		imageSize, R, T, E, F,
		CALIB_USE_INTRINSIC_GUESS,
		TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5));

	cout << "RMS: " << rms << endl;

	/*
	����У����ʱ����Ҫ����ͼ���沢���ж�׼ ��ʹ������ƥ����ӵĿɿ�
	ʹ������ͼ����ķ������ǰ���������ͷ��ͼ��ͶӰ��һ�������������ϣ�����ÿ��ͼ��ӱ�ͼ��ƽ��ͶӰ������ͼ��ƽ�涼��Ҫһ����ת����R
	stereoRectify �����������ľ��Ǵ�ͼ��ƽ��ͶӰ����������ƽ�����ת����Rl,Rr�� Rl,Rr��Ϊ�������ƽ���ж�׼��У����ת����
	���������Rl��ת�����������Rr��ת֮������ͼ����Ѿ����沢���ж�׼�ˡ�
	����Pl,PrΪ���������ͶӰ�����������ǽ�3D�������ת����ͼ���2D�������:P*[X Y Z 1]' =[x y w]
	Q����Ϊ��ͶӰ���󣬼�����Q���԰�2άƽ��(ͼ��ƽ��)�ϵĵ�ͶӰ��3ά�ռ�ĵ�:Q*[x y d 1] = [X Y Z W]������dΪ��������ͼ���ʱ��
	*/
	stereoRectify(cameraMatrixL, distCoeffL, cameraMatrixR, distCoeffR, imageSize, R, T, Rl, Rr, Pl, Pr, Q,
		CALIB_ZERO_DISPARITY, 0, imageSize, &validROIL, &validROIR);

	mapLx = Mat(imageSize, CV_32FC1);
	mapLy = Mat(imageSize, CV_32FC1);
	mapRx = Mat(imageSize, CV_32FC1);
	mapRy = Mat(imageSize, CV_32FC1);
	/*
	����stereoRectify ���������R �� P ������ͼ���ӳ��� mapx,mapy
	mapx,mapy������ӳ�����������Ը�remap()�������ã���У��ͼ��ʹ������ͼ���沢���ж�׼
	ininUndistortRectifyMap()�Ĳ���newCameraMatrix����У����������������openCV���棬У����ļ��������Mrect�Ǹ�ͶӰ����Pһ�𷵻صġ�
	�������������ﴫ��ͶӰ����P���˺������Դ�ͶӰ����P�ж���У��������������
	*/
	initUndistortRectifyMap(cameraMatrixL, distCoeffL, Rl, Pl, imageSize, CV_32FC1, mapLx, mapLy);
	initUndistortRectifyMap(cameraMatrixR, distCoeffR, Rr, Pr, imageSize, CV_32FC1, mapRx, mapRy);

	/*���沢�������*/
	outputCameraParam();
	
	showRectifyImage();

	return 0;
}

