#include "linkbot.h"

CLinkbot::CLinkbot(void) : rsRobots::Robot(rs::LINKBOTT), rsSim::Linkbot(), rsSim::Robot(rsLinkbot::JOINT1, rsLinkbot::JOINT3) {
}

CLinkbot::~CLinkbot(void) {
	if (!_sim->deleteRobot(_pos)) { delete _sim; }
}

/**********************************************************
	public functions
 **********************************************************/
int CLinkbot::connect(char *name, int pause) {
	// create simulation object if necessary
	if (!g_sim)
		g_sim = new RoboSim(name, pause);

	// add to simulation
	g_sim->addRobot(this);

	// call base class connect
	rsSim::Linkbot::connect();

	// success
	return 0;
}

int CLinkbot::turnLeft(double angle, double radius, double trackwidth) {
	// calculate angle to turn
	double r0 = this->getRotation(rsLinkbot::BODY, 2);
	angle = DEG2RAD(angle);
	double rf = r0 + angle;

	// get speed of robot
	double *speed = new double[_dof]();
	this->getJointSpeeds(speed[0], speed[1], speed[2]);

	// turn toward new postition until pointing correctly
	while (fabs(angle) > 0.005) {
		// turn in shortest path
		double theta = (angle*trackwidth)/(2*radius);

		// turn
		if (angle > 0.005)
			this->move(-RAD2DEG(theta), 0, -RAD2DEG(theta));
		else if (RAD2DEG(angle) < -0.005)
			this->move(RAD2DEG(theta), 0, RAD2DEG(theta));

		// calculate new rotation from error
		angle = rf - this->getRotation(rsLinkbot::BODY, 2);

		// move slowly
		this->setJointSpeeds(45, 45, 45);
	}

	// reset to original speed after turning
	this->setJointSpeeds(speed[0], speed[1], speed[2]);

	// success
	return 1;
}

int CLinkbot::turnRight(double angle, double radius, double trackwidth) {
	// calculate angle to turn
	double r0 = this->getRotation(rsLinkbot::BODY, 2);
	angle = DEG2RAD(angle);
	double rf = r0 - angle;

	// get speed of robot
	double *speed = new double[_dof]();
	this->getJointSpeeds(speed[0], speed[1], speed[2]);

	// turn toward new postition until pointing correctly
	while (fabs(angle) > 0.005) {
		// turn in shortest path
		double theta = (angle*trackwidth)/(2*radius);

		// turn
		if (angle > 0.005)
			this->move(RAD2DEG(theta), 0, RAD2DEG(theta));
		else if (RAD2DEG(angle) < -0.005)
			this->move(-RAD2DEG(theta), 0, -RAD2DEG(theta));

		// calculate new rotation from error
		angle = rf - this->getRotation(rsLinkbot::BODY, 2);

		// move slowly
		this->setJointSpeeds(45, 45, 45);
	}

	// reset to original speed after turning
	this->setJointSpeeds(speed[0], speed[1], speed[2]);

	// success
	return 1;
}

