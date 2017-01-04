#pragma once
#include <opencv2\opencv.hpp>
#include "Camera.h"

#define L_CAMERA_NAME "������ͷ"
#define R_CAMERA_NAME "������ͷ"



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

