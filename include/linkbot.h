#ifndef LINKBOT_H_
#define LINKBOT_H_

#include <rsSim/Sim>
#include <rsSim/Linkbot>

#include "robosim.h"

class CLinkbot : public rsSim::Linkbot {
	public:
		CLinkbot(void);
		virtual ~CLinkbot(void);

		int connect(char* = NULL, int = 3);
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

