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

ArRobot robot;
ArSonarDevice sonar;



// FUNCTION DEFINITIONS --------------------------------------------------------

int main(int argc, char** argv)
{
	Aria::init();

	robot.addRangeDevice(&sonar);

	ArArgumentParser parser(&argc, argv);
	ArSimpleConnector connector(&parser);

	if (!connector.parseArgs()) {
		cout << "Parse error" << endl;
		Aria::exit(0);
		exit(1);
	}

	if (!connector.connectRobot(&robot)) {
		cout << "Couldn't connect to robot" << endl;
		Aria::exit(0);
		exit(1);
	}

	robot.enableMotors();
	robot.runAsync(true);

	robot.setMoveDoneDist(1.0);
	robot.setHeadingDoneDiff(0.5);

	// main loop
	ArPose gapLocation;
	while (true) {
		doCenter();
		doAdvance();
		if (!doScan(gapLocation))
			break;
		doMoveThroughGap(gapLocation);
	}
}

/*
	Stops the robot. Invokes ArRobot::stop() on the robot, and then blocks until
	its translational and rotational velocities are sufficiently close to zero.
*/
void stopRobot()
{
	cout << "  robot stopping" << endl;
	robot.lock();
	robot.stop();
	robot.unlock();
	while (abs(robot.getVel()) > STOP_DONE_VEL ||
			abs(robot.getRotVel()) > STOP_DONE_ROT_VEL)
		ArUtil::sleep(100);
}

/*
	Moves the robot forward by the specified distance and blocks until the move
    is done.
*/
void moveRobot(double distance)
{
	cout << "  robot moving " << distance << " mm" << endl;

	// NOTE: This approach was buggy (the robot didn't always stop quickly), so
	// I replaced it. Maybe we can troubleshoot it later? -Ryan
//	robot.lock();
//	robot.move(distance);
//	robot.unlock();
//	while (!robot.isMoveDone())
//		ArUtil::sleep(100);
//	stopRobot();
	
	static const double SLEEP_COEF = 1000.0 / MOVE_VEL;

	robot.lock();
	robot.setVel((distance < 0) ? -MOVE_VEL : MOVE_VEL);
	robot.unlock();
	ArUtil::sleep(abs(distance * SLEEP_COEF));
	stopRobot();
}

/*
	Turns the robot to the specified heading and blocks until the turn is done.
*/
void setRobotTh(double th)
{
	cout << "  robot turning to " << th << " deg" << endl;
	
	robot.lock();
	robot.setHeading(th);
	robot.unlock();
	while (!robot.isHeadingDone())
		ArUtil::sleep(100);
	stopRobot();

	// This is an attempt to make the turn faster. It works, but isn't much
	// better. Needs work. -Ryan
//	th = ArMath::fixAngle(th);
//	while (true) {
//		double relTh = ArMath::subAngle(th, robot.getTh());
//		double absRelTh = abs(relTh);
//		if (absRelTh < 1.0)
//			break;
//		double rotVel = clamp(2.0, 50.0, absRelTh);
//		robot.lock();
//		robot.setRotVel((relTh < 0) ? -rotVel : rotVel);
//		robot.unlock();
//		ArUtil::sleep(1000.0 * absRelTh / rotVel);
//	}
//	stopRobot();
}

/*
	Positions robot in the center of the hallway.
*/
void doCenter()
{
	cout << "CENTERING" << endl;
	setRobotTh(HEADING_PLUS_X);
	
	int left = robot.getSonarRange(SONAR_FL90);
	int right = robot.getSonarRange(SONAR_FR90);
	int deltaY = left - (left + right) / 2;
	cout << "  center is " << deltaY << " mm away" << endl;

	setRobotTh(HEADING_PLUS_Y);
	moveRobot(deltaY);

	//TODO: Possibly add a check here to ensure we are actually in the middle
	//after returning from doMove because of possible obstacle avoidance.
}

/*
	Moves the robot forward until the front two sensors are encounter an object below the threshold range
*/
void doAdvance()
{
	cout << "ADVANCING" << endl;

	setRobotTh(HEADING_PLUS_X);

	// move forward
	cout << "  setting forward velocity to " << MOVE_VEL << " mm/s" << endl;
	robot.lock();
	robot.setVel(MOVE_VEL);
	robot.unlock();

	// don't stop until one of the front two sensors sees an obstacle
	while (robot.getSonarRange(SONAR_FL10) > ADVANCE_STOP_RANGE &&
			robot.getSonarRange(SONAR_FR10) > ADVANCE_STOP_RANGE)
		ArUtil::sleep(100);
	cout << "  obstacle detected ahead at " << ADVANCE_STOP_RANGE << " mm" << endl;

	//TODO: the while loop above currently waits for a sonar reading whose range
	//      is within ADVANCE_STOP_RANGE; as we've defined that parameter, it
	//      should technically wait for a reading whose *x-coordinate* is within
	//      ADVANCE_STOP_RANGE.
	
	stopRobot();
}

/*
	Moves the robot through the gap. First, the robot moves along the y-axis
	so it is across from the gap's center; then, the robot moves through the
	gap along the x-axis. The robot goes GAP_X_CLEARANCE mm past the gap's
	center.
*/
void doMoveThroughGap(ArPose gapLocation)
{
	cout << "MOVING THROUGH GAP" << endl;
	
	setRobotTh(HEADING_PLUS_Y);
	moveRobot(gapLocation.getY());

	setRobotTh(HEADING_PLUS_X);
	moveRobot(gapLocation.getX() + GAP_X_CLEARANCE);
}