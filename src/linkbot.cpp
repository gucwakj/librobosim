#include <rs/Macros>

#include "linkbot.h"

using namespace rsLinkbot;

CLinkbot::CLinkbot(char *name, bool pause) : rsRobots::Robot(rs::LINKBOTT), rsRobots::Linkbot(rs::LINKBOTT), rsSim::Linkbot(rs::LINKBOTT), Robot(rsLinkbot::JOINT1, rsLinkbot::JOINT3) {
	// create simulation object if necessary
	if (!g_sim)
		g_sim = new RoboSim(name, pause);

	// add to simulation
	g_sim->addRobot(this);
}

CLinkbot::~CLinkbot(void) {
	if (!g_sim->deleteRobot(_id)) { delete g_sim; this->_sim = NULL; }
}

/**********************************************************
	public functions
 **********************************************************/
int CLinkbot::accelJointAngleNB(Joint id, double a, double angle) {
	this->accelJointTimeNB(id, a, sqrt(2*angle/a));

	// success
	return 0;
}

int CLinkbot::accelJointCycloidalNB(Joint id, double angle, double t) {
	// lock goal
	MUTEX_LOCK(&_goal_mutex);

	// set initial omega
	if (_motor[id].state != POSITIVE || _motor[id].state != NEGATIVE) {
		if (angle > rs::Epsilon)
			_motor[id].omega = 0.01;
		else
			_motor[id].omega = -0.01;
	}

	// set timeout
	_motor[id].timeout = t/g_sim->getStep();

	// set acceleration parameters
	_motor[id].mode = ACCEL_CYCLOIDAL;
	_motor[id].goal = rs::D2R(angle);
	_motor[id].accel.init = _motor[id].theta;
	_motor[id].accel.period = t;
	_motor[id].accel.run = 0;
	_motor[id].accel.start = 0;

	// enable motor
	MUTEX_LOCK(&_theta_mutex);
	dJointEnable(_motor[id].id);
	dJointSetAMotorAngle(_motor[id].id, 0, _motor[id].theta);
	dBodyEnable(_body[BODY]);
	MUTEX_UNLOCK(&_theta_mutex);

	// unsuccessful
	MUTEX_LOCK(&_motor[id].success_mutex);
	_motor[id].success = false;
	MUTEX_UNLOCK(&_motor[id].success_mutex);

	// unlock goal
	MUTEX_UNLOCK(&_goal_mutex);

	// success
	return 0;
}

int CLinkbot::accelJointHarmonicNB(Joint id, double angle, double t) {
	// lock goal
	MUTEX_LOCK(&_goal_mutex);

	// set initial omega
	if (_motor[id].state != POSITIVE || _motor[id].state != NEGATIVE) {
		if (angle > rs::Epsilon)
			_motor[id].omega = 0.01;
		else
			_motor[id].omega = -0.01;
	}

	// set timeout
	_motor[id].timeout = t/g_sim->getStep();

	// set acceleration parameters
	_motor[id].mode = ACCEL_HARMONIC;
	_motor[id].goal = rs::D2R(angle) - rs::D2R(2);
	_motor[id].accel.init = _motor[id].theta;
	_motor[id].accel.period = t;
	_motor[id].accel.run = 0;
	_motor[id].accel.start = 0;

	// enable motor
	MUTEX_LOCK(&_theta_mutex);
	dJointEnable(_motor[id].id);
	dJointSetAMotorAngle(_motor[id].id, 0, _motor[id].theta);
	dBodyEnable(_body[BODY]);
	MUTEX_UNLOCK(&_theta_mutex);

	// unsuccessful
	MUTEX_LOCK(&_motor[id].success_mutex);
	_motor[id].success = false;
	MUTEX_UNLOCK(&_motor[id].success_mutex);

	// unlock goal
	MUTEX_UNLOCK(&_goal_mutex);

	// success
	return 0;
}

int CLinkbot::accelJointSmoothNB(Joint id, double a0, double af, double vmax, double angle) {
	_motor[id].omega = rs::D2R(vmax);
	this->moveJoint(id, angle);

	// success
	return 0;
}

int CLinkbot::accelJointTimeNB(Joint id, double a, double t) {
	// lock goal
	MUTEX_LOCK(&_goal_mutex);

	// set initial omega
	if (_motor[id].state != POSITIVE || _motor[id].state != NEGATIVE) {
		if (a > rs::Epsilon)
			_motor[id].omega = 0.01;
		else
			_motor[id].omega = -0.01;
	}

	// set timeout
	double step = g_sim->getStep();
	if (t == 0)
		_motor[id].timeout = fabs((_motor[id].omega_max-fabs(_motor[id].omega))/rs::D2R(a)/step);
	else
		_motor[id].timeout = fabs(t/step);

	// set acceleration parameters
	_motor[id].alpha = rs::D2R(a);
	_motor[id].mode = ACCEL_CONSTANT;

	// enable motor
	MUTEX_LOCK(&_theta_mutex);
	dJointEnable(_motor[id].id);
	dJointSetAMotorAngle(_motor[id].id, 0, _motor[id].theta);
	dBodyEnable(_body[BODY]);
	MUTEX_UNLOCK(&_theta_mutex);

	// unsuccessful
	MUTEX_LOCK(&_motor[id].success_mutex);
	_motor[id].success = false;
	MUTEX_UNLOCK(&_motor[id].success_mutex);

	// unlock goal
	MUTEX_UNLOCK(&_goal_mutex);

	// success
	return 0;
}

int CLinkbot::accelJointToMaxSpeedNB(Joint id, double a) {
	this->accelJointTimeNB(id, a, 0);

	// success
	return 0;
}

int CLinkbot::accelJointToVelocityNB(Joint id, double a, double v) {
	this->accelJointTimeNB(id, a, v/a);

	// success
	return 0;
}

int CLinkbot::closeGripper(void) {
	double gripperAngleOld = 0;
	double gripperAngleNew = 0;
	int retval = getJointAngleInstant(JOINT1, gripperAngleNew);
	while ( fabs(gripperAngleNew - gripperAngleOld) > 0.1 ) {
		gripperAngleOld = gripperAngleNew;
		retval = retval || getJointAngleInstant(JOINT1, gripperAngleNew);
		retval = retval || moveNB(8, 0, 8);
		delaySeconds(1);
		retval = retval || getJointAngleInstant(JOINT1, gripperAngleNew);
	}
	retval = retval || moveNB(8, 0, 8);
	delaySeconds(1);
	retval = retval || holdJoints();
	return retval;
}

int CLinkbot::closeGripperNB(void) {
	// create thread
	THREAD_T moving;

	// store args
	LinkbotMove *move = new LinkbotMove;
	move->robot = this;

	// motion in progress
	_motion = true;

	// start thread
	THREAD_CREATE(&moving, closeGripperNBThread, (void *)move);

	// success
	return 0;
}

int CLinkbot::driveAccelCycloidalNB(double radius, double d, double t) {
	this->accelJointCycloidalNB(JOINT1,  rs::R2D(d/radius), t);
	this->accelJointCycloidalNB(JOINT3, -rs::R2D(d/radius), t);

	// success
	return 0;
}

int CLinkbot::driveAccelDistanceNB(double radius, double a, double d) {
	a = rs::D2R(a);
	this->accelJointTimeNB(JOINT1,  rs::R2D(a/radius), sqrt(2*d/a));
	this->accelJointTimeNB(JOINT3, -rs::R2D(a/radius), sqrt(2*d/a));

	// success
	return 0;
}

int CLinkbot::driveAccelHarmonicNB(double radius, double d, double t) {
	this->accelJointHarmonicNB(JOINT1,  rs::R2D(d/radius), t);
	this->accelJointHarmonicNB(JOINT3, -rs::R2D(d/radius), t);

	// success
	return 0;
}

int CLinkbot::driveAccelSmoothNB(double radius, double a0, double af, double vmax, double d) {
	this->accelJointSmoothNB(JOINT1, a0, af, vmax, d/radius);
	this->accelJointSmoothNB(JOINT3, a0, af, vmax, d/radius);

	// success
	return 0;
}

int CLinkbot::driveAccelTimeNB(double radius, double a, double t) {
	a = rs::D2R(a);
	this->accelJointTimeNB(JOINT1,  rs::R2D(a/radius), t);
	this->accelJointTimeNB(JOINT3, -rs::R2D(a/radius), t);

	// success
	return 0;
}

int CLinkbot::driveAccelToMaxSpeedNB(double radius, double a) {
	a = rs::D2R(a);
	this->accelJointTimeNB(JOINT1,  rs::R2D(a/radius), 0);
	this->accelJointTimeNB(JOINT3, -rs::R2D(a/radius), 0);

	// success
	return 0;
}

int CLinkbot::driveAccelToVelocityNB(double radius, double a, double v) {
	a = rs::D2R(a);
	this->accelJointTimeNB(JOINT1,  rs::R2D(a/radius), v/a);
	this->accelJointTimeNB(JOINT3, -rs::R2D(a/radius), v/a);

	// success
	return 0;
}

int CLinkbot::driveAngleNB(double angle) {
	this->moveJointNB(_leftWheel, angle);
	this->moveJointNB(_rightWheel, -angle);

	// success
	return 0;
}

int CLinkbot::driveForeverNB(void) {
	// negate speed to act as a car
	_motor[JOINT3].omega = -_motor[JOINT3].omega;

	// set joint movements
	this->moveJointForeverNB(JOINT1);
	this->moveJointForeverNB(JOINT3);

	// success
	return 0;
}

int CLinkbot::drivexyToSmooth(double x1, double y1, double x2, double y2, double x3, double y3, double radius, double trackwidth) {
	// get midpoints
	double p[2] = {(x1 + x2)/2, (y1 + y2)/2};
	double q[2] = {(x2 + x3)/2, (y2 + y3)/2};

	// calculate equations of bisecting lines
	double m1 = -1/((y1 - y2)/(x1 - x2));
	double m2 = -1/((y2 - y3)/(x2 - x3));
	double b1 = -m1*p[0] + p[1];
	double b2 = -m2*q[0] + q[1];

	// circle parameters that passes through these points
	double c[2] = {(b2 - b1)/(m1 - m2), m1*(b2 - b1)/(m1 - m2) + b1};
	double rho = sqrt((c[0] - x2)*(c[0] - x2) + (c[1] - y2)*(c[1] - y2));
	double theta = 2*fabs(atan((m1 - m2)/(1 + m1*m2)));

	// distance to travel for each wheel
	trackwidth = this->convert(_trackwidth, 0);
	double s1 = theta*(rho + trackwidth/2);
	double s2 = theta*(rho - trackwidth/2);

	// move joints the proper amount
	this->setJointSpeed(JOINT1, rs::R2D(s1/theta/rho/radius*_speed));
	this->setJointSpeed(JOINT3, rs::R2D(s2/theta/rho/radius*_speed));
	this->moveJointNB(JOINT1, rs::R2D(s1/radius));
	this->moveJointNB(JOINT3, -rs::R2D(s2/radius));
	this->delay(theta*rho/_speed*1000);

	// success
	return 0;
}

int CLinkbot::getJointAngles(double &angle1, double &angle2, double &angle3, int numReadings) {
	this->getJointAngle(JOINT1, angle1, numReadings);
	this->getJointAngle(JOINT2, angle2, numReadings);
	this->getJointAngle(JOINT3, angle3, numReadings);

	// success
	return 0;
}

int CLinkbot::getJointAnglesInstant(double &angle1, double &angle2, double &angle3) {
	this->getJointAngleInstant(JOINT1, angle1);
	this->getJointAngleInstant(JOINT2, angle2);
	this->getJointAngleInstant(JOINT3, angle3);

	// success
	return 0;
}

int CLinkbot::getJointSpeeds(double &speed1, double &speed2, double &speed3) {
	this->getJointSpeed(JOINT1, speed1);
	this->getJointSpeed(JOINT2, speed2);
	this->getJointSpeed(JOINT3, speed3);

	// success
	return 0;
}

int CLinkbot::getJointSpeedRatios(double &ratio1, double &ratio2, double &ratio3) {
	this->getJointSpeedRatio(JOINT1, ratio1);
	this->getJointSpeedRatio(JOINT2, ratio2);
	this->getJointSpeedRatio(JOINT3, ratio3);

	// success
	return 0;
}

int CLinkbot::move(double angle1, double angle2, double angle3) {
	this->moveNB(angle1, angle2, angle3);
	this->moveWait();

	// success
	return 0;
}

int CLinkbot::moveNB(double angle1, double angle2, double angle3) {
	this->moveJointNB(JOINT1, angle1);
	this->moveJointNB(JOINT2, angle2);
	this->moveJointNB(JOINT3, angle3);

	// success
	return 0;
}

int CLinkbot::moveTo(double angle1, double angle2, double angle3) {
	this->moveToNB(angle1, angle2, angle3);
	this->moveWait();

	// success
	return 0;
}

int CLinkbot::moveToNB(double angle1, double angle2, double angle3) {
	this->moveJointToNB(JOINT1, angle1);
	this->moveJointToNB(JOINT2, angle2);
	this->moveJointToNB(JOINT3, angle3);

	// success
	return 0;
}

int CLinkbot::moveToByTrackPos(double angle1, double angle2, double angle3) {
	this->moveToByTrackPosNB(angle1, angle2, angle3);
	this->moveWait();

	// success
	return 0;
}

int CLinkbot::moveToByTrackPosNB(double angle1, double angle2, double angle3) {
	this->moveToNB(angle1, angle2, angle3);

	// success
	return 0;
}

int CLinkbot::openGripper(double angle) {
	this->openGripperNB(angle);
	this->moveWait();

	// success
	return 0;
}

int CLinkbot::openGripperNB(double angle) {
	if (_form == rs::LINKBOTL)
		this->moveJointToNB(JOINT1, -angle);
	else
		this->moveToNB(-angle/2, 0, -angle/2);

	// success
	return 0;
}

int CLinkbot::recordAngles(double *time, double *angle1, double *angle2, double *angle3, int num, double seconds, int shiftData) {
	// check if recording already
	for (int i = 0; i < _dof; i++) {
		if (_motor[i].record) { return -1; }
	}

	// store angles
	double **angles = new double * [_dof];
	angles[JOINT1] = angle1;
	angles[JOINT2] = angle2;
	angles[JOINT3] = angle3;

	// call base class recording function
	return Robot::recordAngles(time, angles, num, seconds, shiftData);
}

int CLinkbot::recordAnglesBegin(robotRecordData_t &time, robotRecordData_t &angle1, robotRecordData_t &angle2, robotRecordData_t &angle3, double seconds, int shiftData) {
	// check if recording already
	for (int i = 0; i < _dof; i++) {
		if (_motor[i].record) { return -1; }
	}

	// store angles
	double **angles = new double * [_dof];
	angles[JOINT1] = angle1;
	angles[JOINT2] = angle2;
	angles[JOINT3] = angle3;

	// call base class recording function
	return Robot::recordAnglesBegin(time, angles, seconds, shiftData);
}

int CLinkbot::recordDistancesBegin(robotRecordData_t &time, robotRecordData_t &distance1, robotRecordData_t &distance2, robotRecordData_t &distance3, double radius, double seconds, int shiftData) {
	// check if recording already
	for (int i = 0; i < _dof; i++) {
		if (_motor[i].record) { return -1; }
	}

	// store angles
	double **angles = new double * [_dof];
	angles[JOINT1] = distance1;
	angles[JOINT2] = distance2;
	angles[JOINT3] = distance3;

	// call base class recording function
	return Robot::recordAnglesBegin(time, angles, seconds, shiftData);
}

int CLinkbot::setJointSpeeds(double speed1, double speed2, double speed3) {
	this->setJointSpeed(JOINT1, speed1);
	this->setJointSpeed(JOINT2, speed2);
	this->setJointSpeed(JOINT3, speed3);

	// success
	return 0;
}

int CLinkbot::setJointSpeedRatios(double ratio1, double ratio2, double ratio3) {
	this->setJointSpeedRatio(JOINT1, ratio1);
	this->setJointSpeedRatio(JOINT2, ratio2);
	this->setJointSpeedRatio(JOINT3, ratio3);

	// success
	return 0;
}

int CLinkbot::turnLeft(double angle, double radius, double trackwidth) {
	// calculate final rotation after turn
	angle = rs::D2R(angle);
	double rf = this->getRotation(0, 2) + angle;

	// get speed of robot
	double *speed = new double[_dof]();
	this->getJointSpeeds(speed[0], speed[1], speed[2]);

	// turn toward new postition until pointing correctly
	while (fabs(angle) > 0.005) {
		// calculate wheel theta
		double theta = fabs((angle*this->convert(_trackwidth, 0)) / (2 * radius));

		// turn left
		if (rs::R2D(angle) > 0.005) {
			this->moveJointNB(rsLinkbot::JOINT1, -rs::R2D(theta));
			this->moveJoint(rsLinkbot::JOINT3, -rs::R2D(theta));
		}
		// turn right
		else if (rs::R2D(angle) < -0.005) {
			this->moveJointNB(rsLinkbot::JOINT1, rs::R2D(theta));
			this->moveJoint(rsLinkbot::JOINT3, rs::R2D(theta));
		}

		// calculate new rotation from error
		double turnone = rf - this->getRotation(0, 2);
		double turntwo = rf - (2 * rs::Pi + this->getRotation(0, 2));
		if (fabs(turnone) < fabs(turntwo))	angle = turnone;
		else								angle = turntwo;

		// move slowly
		this->setJointSpeeds(45, 45, 45);
	}

	// reset to original speed after turning
	this->setJointSpeeds(speed[0], speed[1], speed[2]);

	// success
	return 0;
}

int CLinkbot::turnRight(double angle, double radius, double trackwidth) {
	// calculate final rotation after turn
	angle = rs::D2R(angle);
	double rf = this->getRotation(0, 2) + 2 * rs::Pi - angle;
	int num = rf / (2 * rs::Pi);
	rf -= num * 2 * rs::Pi;
	double left = rf - this->getRotation(0, 2);
	double right = rf - (2 * rs::Pi + this->getRotation(0, 2));
	if (fabs(left) < fabs(right))	angle = left;
	else							angle = right;

	// get speed of robot
	double *speed = new double[_dof]();
	this->getJointSpeeds(speed[0], speed[1], speed[2]);

	// turn toward new postition until pointing correctly
	while (fabs(angle) > 0.005) {
		// calculate wheel theta
		double theta = fabs((angle*this->convert(_trackwidth, 0)) / (2 * radius));

		// turn left
		if (rs::R2D(angle) > 0.005) {
			this->moveJointNB(rsLinkbot::JOINT1, -rs::R2D(theta));
			this->moveJoint(rsLinkbot::JOINT3, -rs::R2D(theta));
		}
		// turn right
		else if (rs::R2D(angle) < -0.005) {
			this->moveJointNB(rsLinkbot::JOINT1, rs::R2D(theta));
			this->moveJoint(rsLinkbot::JOINT3, rs::R2D(theta));
		}

		// calculate new rotation from error
		double turnone = rf - this->getRotation(0, 2);
		double turntwo = rf - (2 * rs::Pi + this->getRotation(0, 2));
		if (fabs(turnone) < fabs(turntwo))	angle = turnone;
		else								angle = turntwo;

		// move slowly
		this->setJointSpeeds(45, 45, 45);
	}

	// reset to original speed after turning
	this->setJointSpeeds(speed[0], speed[1], speed[2]);

	// success
	return 0;
}

/**********************************************************
	private functions
 **********************************************************/
void* CLinkbot::closeGripperNBThread(void *arg) {
	// cast arg
	LinkbotMove *move = (LinkbotMove *)arg;

	// perform motion
	move->robot->closeGripper();

	// signal successful completion
	COND_ACTION(&move->robot->_motion_cond, &move->robot->_motion_mutex, move->robot->_motion = false);

	// cleanup
	delete move;

	// success
	return NULL;
}

