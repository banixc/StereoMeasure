#pragma once
#include <opencv2\opencv.hpp>
#include "Camera.h"

#define L_CAMERA_NAME "左摄像头"
#define R_CAMERA_NAME "右摄像头"

#define L_CAMERA_VALUE "D:\\Desktop\\StereoMeasure\\Rescoures\\1.mp4"
#define R_CAMERA_VALUE "D:\\Desktop\\StereoMeasure\\Rescoures\\2.mp4"

#define CAMERA_WINDOW "摄像机界面"

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

