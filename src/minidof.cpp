#include "minidof.h"

using namespace rsMiniDof;

CMiniDof::CMiniDof(int joint, char *name, bool pause) :	rsRobots::Robot(rs::MiniDof),
												rsRobots::MiniDof(joint),
												rsScene::MiniDof(joint),
												rsSim::MiniDof(joint),
												rsCallback::MiniDof(),
												roboSim::Robot(Bodies::Joint, Bodies::Joint) {
	// create simulation object if necessary
	if (g_sim == NULL) g_sim = new RoboSim(name, pause);

	// add to simulation
	g_sim->addRobot(this, this, this);
}

CMiniDof::~CMiniDof(void) {
	if (!g_sim->deleteRobot(this->getID())) { delete g_sim; _sim = NULL; }
}

