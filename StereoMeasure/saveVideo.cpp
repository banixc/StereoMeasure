#include <iostream> 
#include "stdafx.h"
#include "saveVideo.h"

#define L_CAMERA_VALUE "D:\\Desktop\\StereoMeasure\\Rescoures\\1.mp4"
#define R_CAMERA_VALUE "D:\\Desktop\\StereoMeasure\\Rescoures\\2.mp4"


#define L_ID 0
#define R_ID 1

double fps = 30;


void saveVideo()
{
	VideoCapture capturel(L_ID);
	VideoCapture capturer(R_ID);

	//FPS
	double rate = capturer.get(CV_CAP_PROP_FPS);

	VideoWriter writerl("../L.avi",-1, capturel.get(CV_CAP_PROP_FPS), Size(capturel.get(CV_CAP_PROP_FRAME_WIDTH), capturel.get(CV_CAP_PROP_FRAME_HEIGHT)));
	VideoWriter writerr("../R.avi", -1, capturer.get(CV_CAP_PROP_FPS), Size(capturer.get(CV_CAP_PROP_FRAME_WIDTH), capturer.get(CV_CAP_PROP_FRAME_HEIGHT)));

	/*
	writerl.open("../result.avi", 
		-1,
		//CV_FOURCC('F', 'L', 'V', '1'),
		fps,
		videoSize);
	*/

	Mat frame;
	while (true) {

		capturel >> frame;
		imshow("L", frame);
		writerl << frame;

		capturer >> frame;
		imshow("R", frame);
		writerr << frame;

		char c = waitKey(1000/fps);
		if (c == 27) {
			writerl.release();
			writerr.release();
			break;
		}
	}

}
