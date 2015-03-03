#ifndef CMINDSTORMS_HPP_
#define CMINDSTORMS_HPP_

#include <rsSim/Sim>
#include <rsSim/Mindstorms>

#include "robosim.hpp"

class CMindstorms : public rsSim::Mindstorms {
	public:
		CMindstorms(void);
		virtual ~CMindstorms(void);

		int connect(char* = NULL, int = 3);
};

// simulation
extern RoboSim *g_sim;

#endif // CMINDSTORMS_HPP_
