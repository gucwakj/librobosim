#include "linkbot.h"

CLinkbotI::CLinkbotI(void) : rsRobots::Robot(rs::LINKBOTI), rsSim::LinkbotI(), rsSim::Robot(rsLinkbot::JOINT1, rsLinkbot::JOINT3) {
}

CLinkbotI::~CLinkbotI(void) {
	if (!_sim->deleteRobot(_pos)) { delete _sim; }
}

int CLinkbotI::connect(char *name, int pause) {
	// create simulation object if necessary
	if (!g_sim)
		g_sim = new RoboSim(name, pause);

	// add to simulation
	g_sim->addRobot(this);

	// call base class connect
	rsSim::LinkbotI::connect();

	// success
	return 0;
}

CLinkbotL::CLinkbotL(void) : rsRobots::Robot(rs::LINKBOTL), rsSim::LinkbotL(), rsSim::Robot(rsLinkbot::JOINT1, rsLinkbot::JOINT2) {
}

CLinkbotL::~CLinkbotL(void) {
	if (!_sim->deleteRobot(_pos)) { delete _sim; }
}

int CLinkbotL::connect(char *name, int pause) {
	// create simulation object if necessary
	if (!g_sim)
		g_sim = new RoboSim(name, pause);

	// add to simulation
	g_sim->addRobot(this);

	// call base class connect
	rsSim::LinkbotL::connect();

	// success
	return 0;
}

CLinkbotT::CLinkbotT(void) : rsRobots::Robot(rs::LINKBOTT), rsSim::LinkbotT(), rsSim::Robot(rsLinkbot::JOINT1, rsLinkbot::JOINT3) {
}

CLinkbotT::~CLinkbotT(void) {
	if (!_sim->deleteRobot(_pos)) { delete _sim; }
}

int CLinkbotT::connect(char *name, int pause) {
	// create simulation object if necessary
	if (!g_sim)
		g_sim = new RoboSim(name, pause);

	// add to simulation
	g_sim->addRobot(this);

	// call base class connect
	rsSim::LinkbotT::connect();

	// success
	return 0;
}

