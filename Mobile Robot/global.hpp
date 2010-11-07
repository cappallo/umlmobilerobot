#ifndef GLOBAL_HPP
#define	GLOBAL_HPP

#include <iostream>
#include <iomanip>
#include <sstream>

#include "Aria.h"

// GLOBAL DATA -----------------------------------------------------------------

extern const double ADVANCE_STOP_RANGE;
extern const double GAP_X_CLEARANCE;
extern const double MOVE_VEL;

extern const double STOP_DONE_VEL;
extern const double STOP_DONE_ROT_VEL;

// indexes of the 16 sonars
enum SonarIndex {
	SONAR_FL90, SONAR_FL50, SONAR_FL30, SONAR_FL10,  // front left quadrant
	SONAR_FR10, SONAR_FR30, SONAR_FR50, SONAR_FR90,  // front right quadrant
	SONAR_BR90, SONAR_BR50, SONAR_BR30, SONAR_BR10,  // back right quadrant
	SONAR_BL10, SONAR_BL30, SONAR_BL50, SONAR_BL90,  // back left quadrant
	NUM_SONARS
};

extern const double HEADING_PLUS_X, HEADING_PLUS_Y,
                    HEADING_MINUS_X, HEADING_MINUS_Y;

extern ArRobot robot;
extern ArSonarDevice sonar;

// FUNCTION DECLARATIONS -------------------------------------------------------

void stopRobot();
void moveRobot(double distance);
void setRobotTh(double th);

void doCenter();
void doAdvance();
bool doScan(ArPose& gapLocation);
void doMoveThroughGap(ArPose gapLocation);

// CONVENIENCE FUNCTIONS -------------------------------------------------------

/* Returns the absolute value of x. */
template <typename T>
inline T abs(const T& x)
{
	return (x < 0) ? -x : x;
}

/* Clamps x to the range [a, b] and returns the result*/
template <typename T>
inline T clamp(const T& a, const T& b, const T& x)
{
	return (x < a) ? a : ((b < x) ? b : x);
}

/* Writes an ArPose's (x, y) coordinates to a stream */
inline std::ostream& operator<<(std::ostream& os, const ArPose& p)
{
	std::ostringstream oss;
	oss << std::fixed << std::setprecision(2)
	    << "(" << std::setw(9) << p.getX()
	    << "," << std::setw(9) << p.getY() << ")";
	return os << oss.str();
}


#endif

