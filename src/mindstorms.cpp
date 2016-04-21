#include "mindstorms.h"

using namespace rsMindstorms;

CMindstorms::CMindstorms(const char *name, bool pause) :	rsRobots::Robot(rs::EV3),
															rsRobots::Mindstorms(rs::EV3),
															rsScene::Mindstorms(rs::EV3),
															rsSim::Mindstorms(),
															rsCallback::Mindstorms(),
															roboSim::Robot(Bodies::Joint2, Bodies::Joint3) {
	// create simulation object if necessary
	if (g_sim == NULL) g_sim = new RoboSim(name, pause);

	// add to simulation
	g_sim->addRobot(this, this, this);

	// dummy recording variable
	_rec_time = new double *[_dof];
	for (int i = 0; i < _dof; i++) {
		_rec_time[i] = NULL;
	}
}

CMindstorms::~CMindstorms(void) {
	// remove from simulation
	if (!g_sim->deleteRobot(this->getID())) { delete g_sim; _sim = NULL; }

	// delete recording time vars
	for (int i = 0; i < _dof; i++) {
		if (_rec_time[i] != NULL) delete _rec_time[i];
	}
	delete[] _rec_time;
}

int CMindstorms::getJointAngles(double &angle1, double &angle2, double &angle3, double &angle4, int numReadings) {
	angle1 = 0;
	this->getJointAngle(Bodies::Joint2, angle2, numReadings);
	this->getJointAngle(Bodies::Joint3, angle3, numReadings);
	angle4 = 0;

	// success
	return 0;
}

int CMindstorms::getJointAnglesInstant(double &angle1, double &angle2, double &angle3, double &angle4) {
	angle1 = 0;
	this->getJointAngleInstant(Bodies::Joint2, angle2);
	this->getJointAngleInstant(Bodies::Joint3, angle3);
	angle4 = 0;

	// success
	return 0;
}

int CMindstorms::getJointSpeeds(double &speed1, double &speed2, double &speed3, double &speed4) {
	speed1 = 0;
	this->getJointSpeed(Bodies::Joint2, speed2);
	this->getJointSpeed(Bodies::Joint3, speed3);
	speed4 = 0;

	// success
	return 0;
}

int CMindstorms::getJointSpeedRatios(double &ratio1, double &ratio2, double &ratio3, double &ratio4) {
	ratio1 = 0;
	this->getJointSpeedRatio(Bodies::Joint2, ratio2);
	this->getJointSpeedRatio(Bodies::Joint3, ratio3);
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
	this->moveJointNB(Bodies::Joint2, angle2);
	this->moveJointNB(Bodies::Joint3, angle3);

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
	this->moveJointToNB(Bodies::Joint2, angle2);
	this->moveJointToNB(Bodies::Joint3, angle3);

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

int CMindstorms::recordAnglesBegin(robotRecordData_t &time, robotRecordData_t &angle1, robotRecordData_t &angle2, double seconds, int shiftData) {
	// wait until the program starts
	this->pauseWait();

	// check if recording already
	for (int i = 0; i < _dof; i++) {
		if (_motor[i].record) { return -1; }
	}

	// record two joints
	this->recordAngleBegin(Bodies::Joint1, time, angle1, seconds, shiftData);
	this->recordAngleBegin(Bodies::Joint2, _rec_time[1], angle2, seconds, shiftData);

	// success
	return 0;
}

int CMindstorms::setJointSpeeds(double speed1, double speed2, double speed3, double speed4) {
	this->setJointSpeed(Bodies::Joint2, speed2);
	this->setJointSpeed(Bodies::Joint3, speed3);

	// success
	return 0;
}

int CMindstorms::setJointSpeedRatios(double ratio1, double ratio2, double ratio3, double ratio4) {
	this->setJointSpeedRatio(Bodies::Joint2, ratio2);
	this->setJointSpeedRatio(Bodies::Joint3, ratio3);

	// success
	return 0;
}

