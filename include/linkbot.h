#ifndef LINKBOT_H_
#define LINKBOT_H_

#include <rsSim/Sim>
#include <rsSim/Linkbot>

#include "robosim.h"

class CLinkbot : public rsSim::Linkbot {
	public:
		CLinkbot(void);
		virtual ~CLinkbot(void);
		int turnLeft(double, double, double);
		int turnRight(double, double, double);
};

class CLinkbotI : public CLinkbot {
	public:
		CLinkbotI(void);
		virtual ~CLinkbotI(void) {};

		int connect(char* = NULL, int = 3);
};

class CLinkbotL : public CLinkbot {
	public:
		CLinkbotL(void);
		virtual ~CLinkbotL(void) {};

		int connect(char* = NULL, int = 3);
};

class CLinkbotT : public CLinkbot {
	public:
		CLinkbotT(void);
		virtual ~CLinkbotT(void) {};

		int connect(char* = NULL, int = 3);
};

// simulation
extern RoboSim *g_sim;

#endif // LINKBOT_H_

