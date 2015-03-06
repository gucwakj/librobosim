#ifndef LINKBOT_H_
#define LINKBOT_H_

#include <rsSim/Sim>
#include <rsSim/Linkbot>

#include "robosim.h"

class CLinkbot : public rsSim::Linkbot {
	public:
		CLinkbot(void);
		virtual ~CLinkbot(void);

		int accelJointAngleNB(rsLinkbot::Joint, double, double);
		int accelJointCycloidalNB(rsLinkbot::Joint, double, double);
		int accelJointHarmonicNB(rsLinkbot::Joint, double, double);
		int accelJointSmoothNB(rsLinkbot::Joint, double, double, double, double);
		int accelJointTimeNB(rsLinkbot::Joint, double, double);
		int accelJointToMaxSpeedNB(rsLinkbot::Joint, double);
		int accelJointToVelocityNB(rsLinkbot::Joint, double, double);
		int connect(char* = NULL, int = 3);
		int driveAccelCycloidalNB(double, double, double);
		int driveAccelDistanceNB(double, double, double);
		int driveAccelHarmonicNB(double, double, double);
		int driveAccelSmoothNB(double, double, double, double, double);
		int driveAccelTimeNB(double, double, double);
		int driveAccelToMaxSpeedNB(double, double);
		int driveAccelToVelocityNB(double, double, double);
		int driveForeverNB(void);
		int driveForwardNB(double);
		int drivexyToSmooth(double, double, double, double, double, double, double, double);
		int getJointAngles(double&, double&, double&, int = 10);
		int getJointAnglesInstant(double&, double&, double&);
		int getJointSpeeds(double&, double&, double&);
		int getJointSpeedRatios(double&, double&, double&);
		int turnLeft(double, double, double);
		int turnRight(double, double, double);
};

class CLinkbotI : public CLinkbot {
	public:
		CLinkbotI(void) : rsRobots::Robot(rs::LINKBOTI), rsSim::Robot(rsLinkbot::JOINT1, rsLinkbot::JOINT3) {};
};

class CLinkbotL : public CLinkbot {
	public:
		CLinkbotL(void) : rsRobots::Robot(rs::LINKBOTL), rsSim::Robot(rsLinkbot::JOINT1, rsLinkbot::JOINT2) {};
};

class CLinkbotT : public CLinkbot {
	public:
		CLinkbotT(void) : rsRobots::Robot(rs::LINKBOTT), rsSim::Robot(rsLinkbot::JOINT1, rsLinkbot::JOINT3) {};
};

// simulation
extern RoboSim *g_sim;

#endif // LINKBOT_H_

