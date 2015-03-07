#include "mindstorms.h"

CMindstorms::CMindstorms(char *name, bool pause) : rsRobots::Robot(rs::MINDSTORMS), rsSim::Mindstorms(), Robot(rsMindstorms::JOINT1, rsMindstorms::JOINT2) {
	// create simulation object if necessary
	if (!g_sim)
		g_sim = new RoboSim(name, pause);

	// add to simulation
	g_sim->addRobot(this);
}

CMindstorms::~CMindstorms(void) {
	if (!_sim->deleteRobot(_pos)) { delete _sim; }
}

