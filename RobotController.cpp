#include <iostream>

#include "RobotController.hpp"
#include "global.hpp"
#include <conio.h>

using namespace std;

/*
  This is a class that has some callback methods. Functors which refer to these
  callbacks will be passed to the DriverClass.  
*/
class CallbackContainer
{
public:

  void callback1();
};

void CallbackContainer::callback1()
{
  printf("CallbackContainer::callback1 called.\n");
}


RobotController :: RobotController(int &argc, char **argv) {
	
	// ************ Robot Init Block ********************
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

	//Escape Key
	CallbackContainer cb;
  

	ArFunctorC<CallbackContainer> functor1(cb, &CallbackContainer::callback1);

	ArKeyHandler keyHandler;
	keyHandler.addKeyHandler(ArKeyHandler::SPACE, &functor1);

	Aria::setKeyHandler(&keyHandler);
	robot.attachKeyHandler(&keyHandler,true, true);

	
	robot.runAsync(true);

	// Sets the difference required for being done with a move
	robot.setMoveDoneDist(1.0);
	// Sets the difference required for being done with a heading change (e.g. used in isHeadingDone())
	robot.setHeadingDoneDiff(0.5);
	// ***************************************************	
	
}



/*
	The entry point into the RobotController class. This starts the 
	automation sequence for the robot.
*/
void RobotController :: startAutomation() {
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
void RobotController :: stopRobot()
{
	cout << "  robot stopping" << endl;
	robot.lock();
	robot.stop();
	robot.unlock();
	while (abs(robot.getVel()) > STOP_DONE_VEL ||
			abs(robot.getRotVel()) > STOP_DONE_ROT_VEL)
		safeSleep(100);
}


/*
	Moves the robot forward by the specified distance and blocks until the move
    is done.
*/
void RobotController :: moveRobot(double distance)
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
	safeSleep(abs(distance * SLEEP_COEF));
	stopRobot();
}


/*
	Turns the robot to the specified heading and blocks until the turn is done.
*/
void RobotController :: setRobotTh(double th)
{
	cout << "  robot turning to " << th << " deg" << endl;
	
	robot.lock();
	robot.setHeading(th);
	robot.unlock();
	while (!robot.isHeadingDone())
		safeSleep(100);
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
void RobotController :: doCenter()
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
void RobotController :: doAdvance()
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
		safeSleep(100);
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
void RobotController :: doMoveThroughGap(ArPose gapLocation)
{
	cout << "MOVING THROUGH GAP" << endl;
	
	setRobotTh(HEADING_PLUS_Y);
	moveRobot(gapLocation.getY());

	setRobotTh(HEADING_PLUS_X);
	moveRobot(gapLocation.getX() + GAP_X_CLEARANCE);
}



/*
	Rotate robot and find a gap. Returns the estimated center of the gap.
	@Param gapLocation - By Ref of center of gap
	@Return bool - If found opening
*/
bool RobotController :: doScan(ArPose& gapLocation)
{
	cout << "SCANNING" << endl;
	PoseVec readings;

	setRobotTh(-10.0);

	cout << "  setting the robot's rotational velocity to 5 deg/s" << endl;
	// start the robot rotating counter-clockwise
	robot.lock();
	robot.setRotVel(5.0);
	robot.unlock();

	// collect four sets of sonar readings
	for (int i = 0; i < 4; ++i) {

		// wait for the robot to rotate by 5 degrees for each iteration
		while (robot.getTh() < (-10.0 + i * 5.0))
			safeSleep(100);

		// collect readings from the six front sonars spanning the -50 deg to
		// +50 deg foreward arc
		robot.lock();
		ArPose robotPose = robot.getPose();
		cout << "  collecting readings at heading " << robot.getTh() << " deg" << endl;
		for (size_t i = SONAR_FL50; i <= SONAR_FR50; ++i) {
			ArSensorReading* r = robot.getSonarReading(i);

			// the reading's coordinates relative to the robot's position
			ArPose p = r->getPose() - robotPose;

			// the absolute heading at which the reading was taken
			p.setTh(ArMath::addAngle(r->getSensorTh(), r->getThTaken()));

			// record the reading
			readings.push_back(p);
		}
		robot.unlock();
	}

	stopRobot();

	// sort the readings by their th values
	sort(readings.begin(), readings.end(), PoseComparator());

	// dump the collected readings to stdout
//	cout << fixed << setprecision(2);
//	for (PoseVecIter i = readings.begin(); i != readings.end(); ++i) {
//		cout << setw(9) << i->getTh() << ": "
//		     << "(" << setw(9) << i->getX()
//		     << "," << setw(9) << i->getY() << ")" << endl;
//	}

	// Look for a gap:
	// Find the longest sequence of readings (sorted by the angle at which they
	// were taken) whose x-coordinate (which is relative to the robot's
	// position) is greater than ADVANCE_STOP_RANGE plus a 200mm margin of
	// error. To simplify boundary conditions, the first and last readings are
	// ignored.
	int gapLength;
	PoseVecIter gapStart = findLongestSubsequence(
			readings.begin() + 1, readings.end() - 1,
			ScanPredicate(ADVANCE_STOP_RANGE + 200.0), gapLength);

	// if no gap was found, return false
	if (gapLength == 0) {
		cout << "  no gap found" << endl;
		return false;
	}

	// find the edges of the gap
	ArPose gapEdge1 = *(gapStart - 1);
	ArPose gapEdge2 = *(gapStart + gapLength);

	// average the edge positions to find the center of the gap
	gapLocation.setPose((gapEdge1.getX() + gapEdge2.getX()) / 2,
	                    (gapEdge1.getY() + gapEdge2.getY()) / 2);
	cout << "  gap found at " << gapLocation << endl;

	return true;
}

	// Finds the longest subsequence of elements in [first, last) for which the
	// predicate evaluates to true. Returns an iterator to the first element in the
	// subsequence, and sets length to the subsequence's length. If the predicate
	// is false for all elements in [first, last), length == 0 and last is returned.
	// NOTE: this function is used by doScan to identify the gap.
	template <typename ForewardIterator, typename UnaryPredicate>
	ForewardIterator RobotController :: findLongestSubsequence(
			ForewardIterator first, ForewardIterator last,
			UnaryPredicate pred, int& length)
	{
		ForewardIterator currentStart, maxStart = last;
		int currentLength = 0, maxLength = 0;

		for (ForewardIterator i = first; i != last; ++i) {
			if (pred(*i)) {
				if (currentLength == 0)
					currentStart = i;
				++currentLength;
			} else {
				if (currentLength > maxLength) {
					maxStart = currentStart;
					maxLength = currentLength;
				}
				currentLength = 0;
			}
		}

		length = maxLength;
		return maxStart;
	};

	/*
		Same as the ArUtil::sleep, but divided into 10 intervals
		wherein it checks to make sure there hasn't been a keystroke.
		If there has, attempts to stop the robot and kills the process.
	*/
	void RobotController :: safeSleep(int duration)
	{
		for (int i=0; i<10; i++)
		{
			ArUtil::sleep(duration/10);
			if (kbhit())
			{
				cout << "  KEYSTROKE -- halting robot and killing process" << endl;
				try {
					robot.lock();
					robot.stop();
					robot.unlock();
				}
				catch (char * e) { cout << "Error stopping robot: " << e << endl; }

				exit(1);
			}
		}
	}
