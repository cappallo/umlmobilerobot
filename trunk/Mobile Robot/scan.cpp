#include <algorithm>
#include <iostream>
#include <iomanip>
#include <vector>

#include "Aria.h"

#include "global.hpp"

using namespace std;

// Finds the longest subsequence of elements in [first, last) for which the
// predicate evaluates to true. Returns an iterator to the first element in the
// subsequence, and sets length to the subsequence's length. If the predicate
// is false for all elements in [first, last), length == 0 and last is returned.
// NOTE: this function is used by doScan to identify the gap.
template <typename ForewardIterator, typename UnaryPredicate>
ForewardIterator findLongestSubsequence(
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
}

typedef vector<ArPose> PoseVec;
typedef PoseVec::iterator PoseVecIter;

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

/*
	Rotate robot and find a gap. Returns the estimated center of the gap.
	@Param gapLocation - By Ref of center of gap
	@Return bool - If found opening
*/
bool doScan(ArPose& gapLocation)
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
			ArUtil::sleep(100);

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

