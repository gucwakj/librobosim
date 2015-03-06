#ifndef MINDSTORMS_H_
#define MINDSTORMS_H_

#include <rsSim/Sim>
#include <rsSim/Mindstorms>

#include "robosim.h"

class CMindstorms : public rsSim::Mindstorms {
	public:
		CMindstorms(void);
		virtual ~CMindstorms(void);

		int connect(char* = NULL, int = 3);
};

// simulation
extern RoboSim *g_sim;

#endif // MINDSTORMS_H_

