#pragma once
#include "objecttracking.h"
#include "Serial.h"

using namespace std;

Entity::Entity(){}
Entity::~Entity(){}




Entity::Entity(string name) {
	setType(name);
	if (name== "playerfront") {//predefined HSV values for automatic detection
		setHSVmin(Scalar(0, 140, 240));
		setHSVmax(Scalar(11, 181, 256));
		setColor(Scalar(0, 0, 255));
	}
	if (name == "playerrear") {//predefined HSV values for automatic detection
		setHSVmin(Scalar(37, 72, 221));
		setHSVmax(Scalar(117, 117, 256));
		setColor(Scalar(255, 0, 0));
	}
	if (name == "ball") {//predefined HSV values for automatic detection
		setHSVmin(Scalar(11, 96, 217));
		setHSVmax(Scalar(26, 255, 255));
		setColor(Scalar(0, 255, 125));
	}
	if (name == "goal") {//predefined HSV values for automatic detection
		setHSVmin(Scalar(32, 72, 158));
		setHSVmax(Scalar(40, 163, 223));
		setColor(Scalar(0, 255, 0));
	}
}

int Entity::getXPos(){//get x and y coordinates of the car
	return Entity::xPos;
}
int Entity::getYPos(){
	return Entity::yPos;
}
void Entity::setXPos(int x) {//set x and y coordinates of the car
	Entity::xPos = x;
}
void Entity::setYPos(int y) {
	Entity::yPos = y;
}

Scalar Entity::getHSVmin() {//getters for HSV min and max values
	return Entity::HSVmin;
}

Scalar Entity::getHSVmax() {
	return Entity::HSVmax;
}

void Entity::setHSVmin(Scalar min) {//setters for HSV min and max values
	Entity::HSVmin = min;
}

void Entity::setHSVmax(Scalar max) {
	Entity::HSVmax = max;
}


string findDirection(Entity playerrear, Entity ball) {
	if (playerrear.getXPos() >= ball.getXPos() && playerrear.getYPos() >= ball.getYPos())//if ball is in the left top corner
		return "NORTHWEST";
	if (playerrear.getXPos() >= ball.getXPos() && playerrear.getYPos() <= ball.getYPos())//if the ball is in the left bottom corner
		return "SOUTHWEST";
	if (playerrear.getXPos() <= ball.getXPos() && playerrear.getYPos() <= ball.getYPos())//if the ball is in the bottom right corner
		return "SOUTHEAST";
	if (playerrear.getXPos() <= ball.getXPos() && playerrear.getYPos() >= ball.getYPos())//if the ball is in the top right corner
		return "NORTHEAST";
}
//
//void movePlayer(string balld, Entity playerfront, Entity playerrear, Entity ball,Serial &serial) {
//	int x, y;
//	//if direction is northwest x and y both are decreased until they match
//	if (balld == "NORTHWEST") 
//	{
//		if (!((playerrear.getXPos() - playerfront.getXPos() > 0) && (playerfront.getYPos() == playerrear.getYPos()))) 
//		{
//			myData("L",serial);
//		}
//		if (!(playerrear.getXPos() == ball.getXPos())) 
//		{
//			myData("F", serial);
//		}
//		if (!((playerrear.getYPos() - playerfront.getYPos() > 0) && (playerfront.getXPos() == playerrear.getXPos()))) 
//		{
//			myData("R", serial);
//		}
//		if (!(playerfront.getYPos() == ball.getYPos()-5)) 
//		{
//			myData("F", serial);
//		}
//	}
//	//if direction is southwest x is decreased and y is increased until they match
//	if (balld == "SOUTHWEST") {
//		//turn left until car rear x - car front x >= whatever distance is between them && car's front y == car's rear y
//		//go forward until ball's x coordinate == car's x coordinate
//		//turn left until car's front y - car rear y >= whatever distance between them && car's front x == car's rear x
//		//go forward until ball's y coordinate == car's y coordinate
//		//stop
//		if (!((playerrear.getXPos() - playerfront.getXPos() > 0) && (playerfront.getYPos() == playerrear.getYPos()))) {
//			myData("L", serial);
//		}
//		if (!(playerrear.getXPos() == ball.getXPos())) {
//			myData("F", serial);
//		}
//		if (!((playerfront.getYPos() - playerrear.getYPos() > 0) && (playerfront.getXPos() == playerrear.getXPos()))) {
//			myData("L", serial);
//		}
//		if (!(playerfront.getYPos() == ball.getYPos()-5)) {
//			myData("F", serial);
//		}
//	}
//	//if direction is southeast x and y both are increased until they match
//	if (balld == "SOUTHEAST") {
//		//turn right until car front x - car rear x >= whatever distance is between them && car's front y == car's rear y
//		//go forward until ball's x coordinate == car's x coordinate
//		//turn right until car's front y - car rear y >= whatever distance between them && car's front x == car's rear x
//		//go forward until ball's y coordinate == car's y coordinate
//		//stop
//		if (!((playerfront.getXPos() - playerrear.getXPos() > 0) && (playerfront.getYPos() == playerrear.getYPos()))) {
//			myData("R", serial);
//		}
//		if (!(playerrear.getXPos() == ball.getXPos())) {
//			myData("F", serial);
//		}
//		if (!((playerfront.getYPos() - playerrear.getYPos() > 0) && (playerfront.getXPos() == playerrear.getXPos()))) {
//			myData("R", serial);
//		}
//		if (!(playerfront.getYPos() == ball.getYPos()-5)) {
//			myData("F", serial);
//		}
//	}
//	//if direction is northeast x is increased and y is decreased until they match
//	if (balld == "NORTHEAST") {
//		//turn right until car front x - car rear x >= whatever distance is between them && car's front y == car's rear y
//		//go forward until ball's x coordinate == car's x coordinate
//		//turn left until car's rear y - car front y >= whatever distance between them && car's front x == car's rear x
//		//go forward until ball's y coordinate == car's y coordinate
//		//stop
//		if (!((playerfront.getXPos() - playerrear.getXPos() > 0) && (playerfront.getYPos() == playerrear.getYPos()))) {
//			myData("R", serial);
//		}
//		if (!(playerrear.getXPos() == ball.getXPos())) {
//			myData("F", serial);
//		}
//		if (!((playerrear.getYPos() - playerfront.getYPos() > 0) && (playerfront.getXPos() == playerrear.getXPos()))) {
//			myData("L", serial);
//		}
//		if (!(playerfront.getYPos() == ball.getYPos() - 5)) {
//			myData("F", serial);
//		}
//	}
//}
//
//
//
