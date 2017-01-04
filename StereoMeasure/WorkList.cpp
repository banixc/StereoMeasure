#include "stdafx.h"
#include "WorkList.h"

using namespace std;
using namespace cv;

WorkList::WorkList()
{
}


WorkList::~WorkList()
{
}

void WorkList::openCamera()
{
	lCameraCapture = VideoCapture();
	rCameraCapture = VideoCapture();
	lCameraCapture.open(L_CAMERA_VALUE);
	rCameraCapture.open(R_CAMERA_VALUE);

	if (!lCameraCapture.isOpened())
		return;
	else
		cout << "�Ѵ�������ͷ:" << L_CAMERA_VALUE << endl;
	if (!rCameraCapture.isOpened())
		return;
	else
		cout << "�Ѵ�������ͷ:" << R_CAMERA_VALUE << endl;

	char c;
	while (true) {
		Mat frame;
		lCameraCapture >> frame;
		imshow(L_CAMERA_NAME, frame);
		rCameraCapture >> frame;
		imshow(R_CAMERA_NAME, frame);
		c = waitKey(10);
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
