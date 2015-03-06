#include "linkbot.h"

using namespace rsLinkbot;

CLinkbot::CLinkbot(void) : rsRobots::Robot(rs::LINKBOTT), rsSim::Linkbot(), rsSim::Robot(rsLinkbot::JOINT1, rsLinkbot::JOINT3) {
}

CLinkbot::~CLinkbot(void) {
	if (!_sim->deleteRobot(_pos)) { delete _sim; }
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
		if ( angle > EPSILON )
			_motor[id].omega = 0.01;
		else
			_motor[id].omega = -0.01;
	}

	// set timeout
	_motor[id].timeout = t/g_sim->getStep();

	// set acceleration parameters
	_motor[id].mode = ACCEL_CYCLOIDAL;
	_motor[id].goal = DEG2RAD(angle);
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
		if ( angle > EPSILON )
			_motor[id].omega = 0.01;
		else
			_motor[id].omega = -0.01;
	}

	// set timeout
	_motor[id].timeout = t/g_sim->getStep();

	// set acceleration parameters
	_motor[id].mode = ACCEL_HARMONIC;
	_motor[id].goal = DEG2RAD(angle) - DEG2RAD(2);
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
	_motor[id].omega = DEG2RAD(vmax);
	this->moveJoint(id, angle);

	// success
	return 0;
}

int CLinkbot::accelJointTimeNB(Joint id, double a, double t) {
	// lock goal
	MUTEX_LOCK(&_goal_mutex);

	// set initial omega
	if (_motor[id].state != POSITIVE || _motor[id].state != NEGATIVE) {
		if ( a > EPSILON )
			_motor[id].omega = 0.01;
		else
			_motor[id].omega = -0.01;
	}

	// set timeout
	double step = g_sim->getStep();
	if (t == 0)
		_motor[id].timeout = fabs((_motor[id].omega_max-fabs(_motor[id].omega))/DEG2RAD(a)/step);
	else
		_motor[id].timeout = fabs(t/step);

	// set acceleration parameters
	_motor[id].alpha = DEG2RAD(a);
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

int CLinkbot::driveAccelCycloidalNB(double radius, double d, double t) {
	this->accelJointCycloidalNB(JOINT1,  RAD2DEG(d/radius), t);
	this->accelJointCycloidalNB(JOINT3, -RAD2DEG(d/radius), t);

	// success
	return 0;
}

int CLinkbot::driveAccelDistanceNB(double radius, double a, double d) {
	a = DEG2RAD(a);
	this->accelJointTimeNB(JOINT1,  RAD2DEG(a/radius), sqrt(2*d/a));
	this->accelJointTimeNB(JOINT3, -RAD2DEG(a/radius), sqrt(2*d/a));

	// success
	return 0;
}

int CLinkbot::driveAccelHarmonicNB(double radius, double d, double t) {
	this->accelJointHarmonicNB(JOINT1,  RAD2DEG(d/radius), t);
	this->accelJointHarmonicNB(JOINT3, -RAD2DEG(d/radius), t);

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
	a = DEG2RAD(a);
	this->accelJointTimeNB(JOINT1,  RAD2DEG(a/radius), t);
	this->accelJointTimeNB(JOINT3, -RAD2DEG(a/radius), t);

	// success
	return 0;
}

int CLinkbot::driveAccelToMaxSpeedNB(double radius, double a) {
	a = DEG2RAD(a);
	this->accelJointTimeNB(JOINT1,  RAD2DEG(a/radius), 0);
	this->accelJointTimeNB(JOINT3, -RAD2DEG(a/radius), 0);

	// success
	return 0;
}

int CLinkbot::driveAccelToVelocityNB(double radius, double a, double v) {
	a = DEG2RAD(a);
	this->accelJointTimeNB(JOINT1,  RAD2DEG(a/radius), v/a);
	this->accelJointTimeNB(JOINT3, -RAD2DEG(a/radius), v/a);

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

