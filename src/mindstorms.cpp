#include "mindstorms.h"

using namespace rsMindstorms;

CMindstorms::CMindstorms(char *name, bool pause) : rsRobots::Robot(rs::EV3), rsSim::Mindstorms(), Robot(JOINT1, JOINT2) {
	// create simulation object if necessary
	if (!g_sim)
		g_sim = new RoboSim(name, pause);

	// add to simulation
	g_sim->addRobot(this);
}

CMindstorms::~CMindstorms(void) {
	if (!_sim->deleteRobot(_pos)) { delete _sim; }
}

int CMindstorms::getJointAngles(double &angle1, double &angle2, int numReadings) {
	this->getJointAngle(JOINT1, angle1, numReadings);
	this->getJointAngle(JOINT2, angle2, numReadings);

	// success
	return 0;
}

int CMindstorms::getJointAnglesInstant(double &angle1, double &angle2) {
	this->getJointAngleInstant(JOINT1, angle1);
	this->getJointAngleInstant(JOINT2, angle2);

	// success
	return 0;
}

int CMindstorms::getJointSpeeds(double &speed1, double &speed2) {
	speed1 = RAD2DEG(_motor[JOINT1].omega);
	speed2 = RAD2DEG(_motor[JOINT2].omega);

	// success
	return 0;
}

int CMindstorms::getJointSpeedRatios(double &ratio1, double &ratio2) {
	ratio1 = _motor[JOINT1].omega/_motor[JOINT1].omega_max;
	ratio2 = _motor[JOINT2].omega/_motor[JOINT2].omega_max;

	// success
	return 0;
}

int CMindstorms::move(double angle1, double angle2) {
	this->moveNB(angle1, angle2);
	this->moveWait();

	// success
	return 0;
}

int CMindstorms::moveNB(double angle1, double angle2) {
	// store angles
	double *angles = new double[_dof];
	angles[JOINT1] = angle1;
	angles[JOINT2] = angle2;

	// call base class recording function
	int retval = Robot::moveNB(angles);

	// clean up
	delete angles;

	// success
	return retval;
}

int CMindstorms::moveTo(double angle1, double angle2) {
	this->moveToNB(angle1, angle2);
	this->moveWait();

	// success
	return 0;
}

int CMindstorms::moveToNB(double angle1, double angle2) {
	// store angles
	double *angles = new double[_dof];
	angles[JOINT1] = angle1;
	angles[JOINT2] = angle2;

	// call base class recording function
	int retval = Robot::moveToNB(angles);

	// clean up
	delete angles;

	// success
	return retval;
}

int CMindstorms::moveToByTrackPos(double angle1, double angle2) {
	this->moveToByTrackPosNB(angle1, angle2);
	this->moveWait();

	// success
	return 0;
}

int CMindstorms::moveToByTrackPosNB(double angle1, double angle2) {
	this->moveToNB(angle1, angle2);

	// success
	return 0;
}

int CMindstorms::recordAngles(double *time, double *angle1, double *angle2, int num, double seconds, int shiftData) {
	// check if recording already
	for (int i = 0; i < _dof; i++) {
		if (_motor[i].record) { return -1; }
	}

	// store angles
	double **angles = new double * [_dof];
	angles[JOINT1] = angle1;
	angles[JOINT2] = angle2;

	// call base class recording function
	return Robot::recordAngles(time, angles, num, seconds, shiftData);
}

int CMindstorms::recordAnglesBegin(robotRecordData_t &time, robotRecordData_t &angle1, robotRecordData_t &angle2, double seconds, int shiftData) {
	// check if recording already
	for (int i = 0; i < _dof; i++) {
		if (_motor[i].record) { return -1; }
	}

	// store angles
	double **angles = new double * [_dof];
	angles[JOINT1] = angle1;
	angles[JOINT2] = angle2;

	// call base class recording function
	return Robot::recordAnglesBegin(time, angles, seconds, shiftData);
}

int CMindstorms::recordDistancesBegin(robotRecordData_t &time, robotRecordData_t &distance1, robotRecordData_t &distance2, double radius, double seconds, int shiftData) {
	// check if recording already
	for (int i = 0; i < _dof; i++) {
		if (_motor[i].record) { return -1; }
	}

	// store angles
	double **angles = new double * [_dof];
	angles[JOINT1] = distance1;
	angles[JOINT2] = distance2;

	// call base class recording function
	return Robot::recordAnglesBegin(time, angles, seconds, shiftData);
}

int CMindstorms::setJointSpeeds(double speed1, double speed2) {
	this->setJointSpeed(JOINT1, speed1);
	this->setJointSpeed(JOINT2, speed2);

	// success
	return 0;
}

int CMindstorms::setJointSpeedRatios(double ratio1, double ratio2) {
	this->setJointSpeedRatio(JOINT1, ratio1);
	this->setJointSpeedRatio(JOINT2, ratio2);

	// success
	return 0;
}

