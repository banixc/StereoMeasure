#include "stdafx.h"
#include "Camera.h"
#include <string>

Camera::Camera()
{
	path = NULL;
}

Camera::Camera(string name, int number)
{
	Camera();
	setName(name);
	setPath(number);
}

Camera::Camera(string name, string url)
{
	Camera();
	setName(name);
	setPath(url);
}


Camera::~Camera()
{
	//if (path != NULL)
	delete path;
}

string Camera::getName()
{
	return name;
}

void Camera::setName(string name)
{
	this->name = name;
}

void * Camera::getPath()
{
	return path;
}

void Camera::setPath(int number)
{
	isFile = false;
	//if (path != NULL)
		//delete path;
	path = new int;
	*(int*)path = number;
}

void Camera::setPath(string url)
{
	isFile = true;
	//if (path != NULL)
		//delete path;
	path = new string(url);
}

int Camera::getNumber()
{
	if(isFile)
		return -1;
	return *(int*)path;
}

string Camera::getUrl()
{
	if (!isFile)
		return UD;
	return *(string*)path;
}

void Camera::print()
{
	cout << "NAME: " << name;
	if (isFile)
		cout << " FILE: " << *(string*)path;
	else
		cout << " DEVICE: " << *(int*)path;
	cout << endl;
}

void Camera::test()
{
	Camera l = Camera("左相机", "text");
	Camera r = Camera("右相机", 0);
	l.print();
	r.print();
	cout << "[TEST END]" << endl;
}

