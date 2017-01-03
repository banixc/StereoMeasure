#include "stdafx.h"
#include "WorkList.h"


WorkList::WorkList()
{
}


WorkList::~WorkList()
{
}

void WorkList::setCamera()
{
	//lCamera = Camera(L_CAMERA_NAME, L_CAMERA_VALUE);
	//rCamera = Camera(R_CAMERA_NAME, R_CAMERA_VALUE);
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

void WorkList::start()
{
	setCamera();
	openCamera();
}
