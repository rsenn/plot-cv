#pragma once
#include <iostream>
#include <string>
#include <vector>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "Serial.h"
using namespace std;
using namespace cv;

class Entity {
public:
  Entity();
  ~Entity();

  Entity(string name);

  int getXPos();
  int getYPos();
  void setXPos(int x);
  void setYPos(int y);
  Scalar getHSVmin();
  Scalar getHSVmax();
  void setHSVmin(Scalar min);
  void setHSVmax(Scalar max);
  string
  getType() {
    return type;
  } // get type of the object
  void
  setType(string t) { // to set type of the object
    type = t;
  }
  Scalar
  getColor() { // to get color for the string to display type of object
    return Color;
  }

  void
  setColor(Scalar c) { // to set color for the string to display type of object
    Color = c;
  }

private:
  int xPos, yPos;        // x y coordinates of the object
  string type;           // type of object (player, ball, goal
  Scalar HSVmin, HSVmax; // to store colour of the object
  Scalar Color;          // colour for the text displayed
};

// finds direction to make the movement easier
string findDirection(Entity player, Entity ball);
// coordinates of the player are changed according to the direction of the ball