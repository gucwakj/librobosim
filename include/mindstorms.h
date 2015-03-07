#ifndef MINDSTORMS_H_
#define MINDSTORMS_H_

#include <rsSim/Sim>
#include <rsSim/Mindstorms>

#include "robosim.h"
#include "robot.h"

class CMindstorms : public rsSim::Mindstorms, public Robot {
	public:
		CMindstorms(char* = NULL, bool = true);
		virtual ~CMindstorms(void);
};

// simulation
extern RoboSim *g_sim;

#endif // MINDSTORMS_H_

