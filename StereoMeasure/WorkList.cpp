#include "stdafx.h"
#include "WorkList.h"

using namespace std;
using namespace cv;

#define L_CAMERA_VALUE 1
#define R_CAMERA_VALUE 2

WorkList::WorkList()
{
}


WorkList::~WorkList()
{
}

void WorkList::openCamera()
{
	lCameraCapture = VideoCapture(L_CAMERA_VALUE);
	rCameraCapture = VideoCapture(R_CAMERA_VALUE);

	if (!lCameraCapture.isOpened())
		return;
	else
		cout << "�Ѵ�������ͷ:" << L_CAMERA_VALUE << endl;
	if (!rCameraCapture.isOpened())
		return;
	else
		cout << "�Ѵ�������ͷ:" << R_CAMERA_VALUE << endl;

	while (true) {


		Mat frame;
		lCameraCapture >> frame;
		imshow(L_CAMERA_NAME, frame);
		rCameraCapture >> frame;
		imshow(R_CAMERA_NAME, frame);
		char c = waitKey(10);
		if (c == 27)
			break;
	}
}

void WorkList::calibrate()
{
	//StereoCalibrate();
}

void WorkList::start()
{
	openCamera();
}
