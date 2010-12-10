#ifndef _ROBOTCONTROLLER_H
#define _ROBOTCONTROLLER_H

#include "Aria.h"
#include <vector>
#include "global.hpp"
#include <algorithm>
#include <iomanip>

using namespace std;

class RobotController {
public:
	//Constructors
	RobotController(int &argc, char **argv);

	//Functions
	void startAutomation();

private:
	ArRobot robot;
	ArSonarDevice sonar;

	void stopRobot();
	void moveRobot(double distance);
	void setRobotTh(double th);
	void doCenter();
	void doAdvance();
	void doMoveThroughGap(ArPose gapLocation);
	bool doScan(ArPose& gapLocation);
	void safeSleep(int duration);

	typedef vector<ArPose> PoseVec;
	typedef PoseVec::iterator PoseVecIter;

	template <typename ForewardIterator, typename UnaryPredicate>
	ForewardIterator findLongestSubsequence(
			ForewardIterator first, ForewardIterator last,
			UnaryPredicate pred, int& length);

	// Returns true if the reading's x-coordinate is greater than a threshold value.
	struct ScanPredicate {
		int myThreshold;
		ScanPredicate(int threshold) : myThreshold(threshold) {}
		bool operator()(const ArPose& pose) {
			return pose.getX() > myThreshold;
		}
	};

	// Orders poses by their th values.
	struct PoseComparator {
		bool operator()(const ArPose& p1, const ArPose& p2) {
			return p1.getTh() < p2.getTh();
		}
	};


};

#endif