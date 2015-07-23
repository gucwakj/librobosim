#include "mindstorms.h"

using namespace rsMindstorms;

CMindstorms::CMindstorms(char *name, bool pause) : rsRobots::Robot(rs::EV3), rsRobots::Mindstorms(rs::EV3), rsSim::Mindstorms(), Robot(JOINT1, JOINT2) {
	// create simulation object if necessary
	if (!g_sim)
		g_sim = new RoboSim(name, pause);

	// add to simulation
	g_sim->addRobot(this);
}

CMindstorms::~CMindstorms(void) {
	if (!g_sim->deleteRobot(_id)) { delete g_sim; this->_sim = NULL; }
}

int CMindstorms::getJointAngles(double &angle1, double &angle2, double &angle3, double &angle4, int numReadings) {
	angle1 = 0;
	this->getJointAngle(JOINT1, angle2, numReadings);
	this->getJointAngle(JOINT2, angle3, numReadings);
	angle4 = 0;

	// success
	return 0;
}

int CMindstorms::getJointAnglesInstant(double &angle1, double &angle2, double &angle3, double &angle4) {
	angle1 = 0;
	this->getJointAngleInstant(JOINT1, angle2);
	this->getJointAngleInstant(JOINT2, angle3);
	angle4 = 0;

	// success
	return 0;
}

int CMindstorms::getJointSpeeds(double &speed1, double &speed2, double &speed3, double &speed4) {
	speed1 = 0;
	this->getJointSpeed(JOINT1, speed2);
	this->getJointSpeed(JOINT2, speed3);
	speed4 = 0;

	// success
	return 0;
}

int CMindstorms::getJointSpeedRatios(double &ratio1, double &ratio2, double &ratio3, double &ratio4) {
	ratio1 = 0;
	this->getJointSpeedRatio(JOINT1, ratio2);
	this->getJointSpeedRatio(JOINT2, ratio3);
	ratio4 = 0;

	// success
	return 0;
}

int CMindstorms::move(double angle1, double angle2, double angle3, double angle4) {
	this->moveNB(angle1, angle2, angle3, angle4);
	this->moveWait();

	// success
	return 0;
}

int CMindstorms::moveNB(double angle1, double angle2, double angle3, double angle4) {
	this->moveJointNB(JOINT1, angle2);
	this->moveJointNB(JOINT2, angle3);

	// success
	return 0;
}

int CMindstorms::moveTo(double angle1, double angle2, double angle3, double angle4) {
	this->moveToNB(angle1, angle2, angle3, angle4);
	this->moveWait();

	// success
	return 0;
}

int CMindstorms::moveToNB(double angle1, double angle2, double angle3, double angle4) {
	this->moveJointToNB(JOINT1, angle2);
	this->moveJointToNB(JOINT2, angle3);

	// success
	return 0;
}

int CMindstorms::moveToByTrackPos(double angle1, double angle2, double angle3, double angle4) {
	this->moveToByTrackPosNB(angle1, angle2, angle3, angle4);
	this->moveWait();

	// success
	return 0;
}

int CMindstorms::moveToByTrackPosNB(double angle1, double angle2, double angle3, double angle4) {
	this->moveToNB(angle1, angle2, angle3, angle4);

	// success
	return 0;
}

int CMindstorms::setJointSpeeds(double speed1, double speed2, double speed3, double speed4) {
	this->setJointSpeed(JOINT1, speed2);
	this->setJointSpeed(JOINT2, speed3);

	// success
	return 0;
}

int CMindstorms::setJointSpeedRatios(double ratio1, double ratio2, double ratio3, double ratio4) {
	this->setJointSpeedRatio(JOINT1, ratio2);
	this->setJointSpeedRatio(JOINT2, ratio3);

	// success
	return 0;
}

