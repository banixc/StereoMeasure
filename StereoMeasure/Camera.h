#pragma once
#include <iostream>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

class Camera
{
public:
	Camera();
	Camera(string,int);
	Camera(string, string);
	~Camera();
	string getName();
	void setName(string);
	void* getPath();
	void setPath(int);
	void setPath(string);
	int getNumber();
	string getUrl();
	void print();

	static void test();
	
private:
	bool isFile;
	string name;
	void* path;
};

