#pragma once
#include <opencv2\opencv.hpp>
#include "Camera.h"

#define L_CAMERA_NAME "������ͷ"
#define R_CAMERA_NAME "������ͷ"

#define L_CAMERA_VALUE "D:\\Desktop\\StereoMeasure\\Rescoures\\1.mp4"
#define R_CAMERA_VALUE "D:\\Desktop\\StereoMeasure\\Rescoures\\2.mp4"

#define CAMERA_WINDOW "���������"

using namespace std;
using namespace cv;

class WorkList
{

public:

	WorkList();
	~WorkList();
	void openCamera();
	void calibrate();
	void start();
private:
	//Camera lCamera, rCamera;
	VideoCapture lCameraCapture,rCameraCapture;
};

