#include "clinkbot.hpp"

CLinkbotI::CLinkbotI(void) : rsRobots::Robot(rs::LINKBOTI), rsSim::LinkbotI(), rsSim::Robot(rs::JOINT1, rs::JOINT3) {
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
	rsSim::LinkbotI::connect();

	// success
	return 0;
}

CLinkbotL::CLinkbotL(void) : rsRobots::Robot(rs::LINKBOTI), rsSim::LinkbotL(), rsSim::Robot(rs::JOINT1, rs::JOINT2) {
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
	rsSim::LinkbotL::connect();

	// success
	return 0;
}

CLinkbotT::CLinkbotT(void) : rsRobots::Robot(rs::LINKBOTI), rsSim::LinkbotT(), rsSim::Robot(rs::JOINT1, rs::JOINT3) {
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
	rsSim::LinkbotT::connect();

	// success
	return 0;
}

