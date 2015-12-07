#include "dof.h"

using namespace rsDof;

CDof::CDof(short joint, float scale, const char *name, bool pause) :
		rsRobots::Robot(rs::Dof),
		rsRobots::Dof(joint, scale),
		rsScene::Dof(joint, scale),
		rsSim::Dof(joint, scale),
		rsCallback::Dof(),
		roboSim::Robot(Bodies::Joint, Bodies::Joint) {
	// create simulation object if necessary
	if (g_sim == NULL) g_sim = new RoboSim(name, pause);

	// add to simulation
	g_sim->addRobot(this, this, this);
}

CDof::~CDof(void) {
	if (!g_sim->deleteRobot(this->getID())) { delete g_sim; _sim = NULL; }
}

