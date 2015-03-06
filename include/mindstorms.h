#ifndef MINDSTORMS_H_
#define MINDSTORMS_H_

#include <rsSim/Sim>
#include <rsSim/Mindstorms>

#include "robosim.h"

class CMindstorms : public rsSim::Mindstorms {
	public:
		CMindstorms(char* = NULL, bool = true);
		virtual ~CMindstorms(void);
};

// simulation
extern RoboSim *g_sim;

#endif // MINDSTORMS_H_

