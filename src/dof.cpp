#include "dof.h"

using namespace rsDof;

CDof::CDof(int joint, char *name, bool pause) :	rsRobots::Robot(rs::DOF),
												rsRobots::Dof(joint),
												rsScene::Dof(joint),
												rsSim::Dof(joint),
												rsCallback::Dof(),
												roboSim::Robot(Bodies::Joint, Bodies::Joint) {
	// create simulation object if necessary
	if (g_sim == NULL) g_sim = new RoboSim(name, pause);

	// add to simulation
	g_sim->addRobot(this, this, this);
}

CDof::~CDof(void) {
	if (!g_sim->deleteRobot(_id)) { delete g_sim; _sim = NULL; }
}

