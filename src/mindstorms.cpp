#include "mindstorms.h"

CMindstorms::CMindstorms(void) : rsRobots::Robot(rs::MINDSTORMS), rsSim::Mindstorms(), rsSim::Robot(rsMindstorms::JOINT1, rsMindstorms::JOINT2) {
}

CMindstorms::~CMindstorms(void) {
	if (!_sim->deleteRobot(_pos)) { delete _sim; }
}

int CMindstorms::connect(char *name, int pause) {
	// create simulation object if necessary
	if (!g_sim)
		g_sim = new RoboSim(name, pause);

	// add to simulation
	g_sim->addRobot(this);

	// call base class connect
	rsSim::Mindstorms::connect();

	// success
	return 0;
}

