#include <algorithm>
#include <cassert>
#include <cstdio>
#include <iostream>
#include <iomanip>
#include <list>
#include <map>
#include <sstream>
#include <vector>

#include "Aria.h"

#include "global.hpp"
#include "RobotController.hpp"

using namespace std;


// GLOBAL DATA DEFINITIONS -----------------------------------------------------

const double ADVANCE_STOP_RANGE = 2000.0;
const double GAP_X_CLEARANCE = 500.0;
const double MOVE_VEL = 400.0;
const double STOP_DONE_VEL = 0.01;
const double STOP_DONE_ROT_VEL = 0.1;

const double HEADING_PLUS_X = 0.0;
const double HEADING_PLUS_Y = 90.0;
const double HEADING_MINUS_X = 180.0;
const double HEADING_MINUS_Y = -90.0;


int main(int argc, char** argv)
{

	RobotController *roboController = new RobotController(argc, argv);

	roboController->startAutomation();

}
