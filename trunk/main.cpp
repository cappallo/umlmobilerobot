#include "Aria.h"

/** @example simpleConnect.cpp example showing how to connect to the robot with ArRobotConnector
 *
 * One of the simplest ARIA programs possible:
 * Connects with ArRobotConnector, waits 3 seconds doing
 * nothing, then exits.
 *
 * This program will work either with the MobileSim simulator or on a real
 * robot's onboard computer.  (Or use -remoteHost to connect to a wireless
 * ethernet-serial bridge.)
 */

int main(int argc, char **argv)
{
  Aria::init();
  ArRobot robot;
  ArArgumentParser parser(&argc, argv);
  parser.loadDefaultArguments();

  // Connect to the robot, get some initial data from it such as type and name,
  // and then load parameter files for this robot.
  ArRobotConnector robotConnector(&parser, &robot);
  if(!robotConnector.connectRobot())
  {
    ArLog::log(ArLog::Terse, "simpleConnect: Could not connect to the robot.");
    if(parser.checkHelpAndWarnUnparsed())
    {
        // -help not given
        Aria::logOptions();
        Aria::exit(1);
    }
  }
  if (!Aria::parseArgs())
  {
    Aria::logOptions();
    Aria::shutdown();
    return 1;
  }
  
  ArLog::log(ArLog::Normal, "simpleConnect: Connected.");

  // Start the robot processing cycle running in the background.
  // True parameter means that if the connection is lost, then the 
  // run loop ends.
  robot.runAsync(true);

  // Print out some data from the SIP.  We must "lock" the ArRobot object
  // before calling its methods, and "unlock" when done, to prevent conflicts
  // with the background thread started by the call to robot.runAsync() above.
  // See the section on threading in the manual for more about this.
  robot.lock();
  ArLog::log(ArLog::Normal, "simpleConnect: Pose=(%.2f,%.2f,%.2f), Trans. Vel=%.2f, Battery=%.2fV",
    robot.getX(), robot.getY(), robot.getTh(), robot.getVel(), robot.getBatteryVoltage());
  robot.unlock();

  // Sleep for 3 seconds.
  ArLog::log(ArLog::Normal, "simpleConnect: Sleeping for 3 seconds...");
  ArUtil::sleep(3000);

  
  ArLog::log(ArLog::Normal, "simpleConnect: Ending robot thread...");
  robot.stopRunning();

  // wait for the thread to stop
  robot.waitForRunExit();

  // exit
  ArLog::log(ArLog::Normal, "simpleConnect: Exiting.");
  return 0;
}
