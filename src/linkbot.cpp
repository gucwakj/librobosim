#include "clinkbot.hpp"

CLinkbotI::CLinkbotI(void) : rsRobots::Robot(rs::LINKBOTI), rsSim::CLinkbotI(), rsSim::Robot(rs::JOINT1, rs::JOINT3) {
}

CLinkbotI::~CLinkbotI(void) {
}

int CLinkbotI::connect(char *name, int pause) {
	// create simulation object if necessary
	if (!g_sim)
		g_sim = new RoboSim(name, pause);

	// add to simulation
	g_sim->addRobot(this);

	// call base class connect
	rsSim::CLinkbotI::connect();

	// success
	return 0;
}

CLinkbotL::CLinkbotL(void) : rsRobots::Robot(rs::LINKBOTI), rsSim::CLinkbotL(), rsSim::Robot(rs::JOINT1, rs::JOINT2) {
}

CLinkbotL::~CLinkbotL(void) {
}

int CLinkbotL::connect(char *name, int pause) {
	// create simulation object if necessary
	if (!g_sim)
		g_sim = new RoboSim(name, pause);

	// add to simulation
	g_sim->addRobot(this);

	// call base class connect
	rsSim::CLinkbotL::connect();

	// success
	return 0;
}

CLinkbotT::CLinkbotT(void) : rsRobots::Robot(rs::LINKBOTI), rsSim::CLinkbotT(), rsSim::Robot(rs::JOINT1, rs::JOINT3) {
}

CLinkbotT::~CLinkbotT(void) {
}

int CLinkbotT::connect(char *name, int pause) {
	// create simulation object if necessary
	if (!g_sim)
		g_sim = new RoboSim(name, pause);

	// add to simulation
	g_sim->addRobot(this);

	// call base class connect
	rsSim::CLinkbotT::connect();

	// success
	return 0;
}

