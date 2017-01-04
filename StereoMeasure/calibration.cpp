#include "stdafx.h"
#include "calibration.h"
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

#define PATH1 "D:\\Desktop\\StereoMeasure\\Rescoures\\1.jpg"
#define PATH2 "D:\\Desktop\\StereoMeasure\\Rescoures\\2.jpg"

#define ROWS 6
#define COLS 9
#define L 20.0

void calibration()
{
	Mat frame,out;
	frame = imread(PATH2);

	Size patternsize(ROWS, COLS); //interior number of corners
	//Mat gray; //source image
	//cvtColor(frame, gray, COLOR_RGB2GRAY);
	vector<Point2f> corners; //this will be filled by the detected corners

							 //CALIB_CB_FAST_CHECK saves a lot of time on images
							 //that do not contain any chessboard corners
	bool patternfound = findChessboardCorners(frame, patternsize, corners,
		CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
		+ CALIB_CB_FAST_CHECK);

	if (patternfound) {
/*		cornerSubPix(frame, corners, patternsize, Size(-1, -1),
			TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));*/

		drawChessboardCorners(frame, patternsize, Mat(corners), patternfound);
		imshow("T", frame);
		while (true) {
			char c = waitKey(0);
			break;
		}
	}

}
