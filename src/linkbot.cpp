#include "linkbot.h"

using namespace rsLinkbot;

CLinkbot::CLinkbot(const char *name, bool pause) :	rsRobots::Robot(rs::LinkbotT),
													rsRobots::Linkbot(rs::LinkbotT),
													rsScene::Linkbot(rs::LinkbotT),
													rsSim::Linkbot(rs::LinkbotT),
													rsCallback::Linkbot(),
													roboSim::Robot(Bodies::Joint1, Bodies::Joint3) {
	// create simulation object if necessary
	if (g_sim == NULL) g_sim = new RoboSim(name, pause);

	// add to simulation
	g_sim->addRobot(this, this, this);
}

CLinkbot::~CLinkbot(void) {
	if (!g_sim->deleteRobot(this->getID())) { delete g_sim; _sim = NULL; }
}

/**********************************************************
	public functions
 **********************************************************/
int CLinkbot::accelJointAngleNB(Bodies::Joint id, double a, double angle) {
	this->accelJointTimeNB(id, a, sqrt(2*angle/a));

	// success
	return 0;
}

int CLinkbot::accelJointCycloidalNB(Bodies::Joint id, double angle, double t) {
	// lock goal
	RS_MUTEX_LOCK(&_goal_mutex);

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
	RS_MUTEX_LOCK(&_theta_mutex);
	dJointEnable(_motor[id].id);
	dJointSetAMotorAngle(_motor[id].id, 0, _motor[id].theta);
	dBodyEnable(_body[Bodies::Body]);
	RS_MUTEX_UNLOCK(&_theta_mutex);

	// unsuccessful
	RS_MUTEX_LOCK(&_motor[id].success_mutex);
	_motor[id].success = false;
	RS_MUTEX_UNLOCK(&_motor[id].success_mutex);

	// unlock goal
	RS_MUTEX_UNLOCK(&_goal_mutex);

	// success
	return 0;
}

int CLinkbot::accelJointHarmonicNB(Bodies::Joint id, double angle, double t) {
	// lock goal
	RS_MUTEX_LOCK(&_goal_mutex);

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
	RS_MUTEX_LOCK(&_theta_mutex);
	dJointEnable(_motor[id].id);
	dJointSetAMotorAngle(_motor[id].id, 0, _motor[id].theta);
	dBodyEnable(_body[Bodies::Body]);
	RS_MUTEX_UNLOCK(&_theta_mutex);

	// unsuccessful
	RS_MUTEX_LOCK(&_motor[id].success_mutex);
	_motor[id].success = false;
	RS_MUTEX_UNLOCK(&_motor[id].success_mutex);

	// unlock goal
	RS_MUTEX_UNLOCK(&_goal_mutex);

	// success
	return 0;
}

int CLinkbot::accelJointSmoothNB(Bodies::Joint id, double a0, double af, double vmax, double angle) {
	_motor[id].omega = rs::D2R(vmax);
	this->moveJoint(id, angle);

	// success
	return 0;
}

int CLinkbot::accelJointTimeNB(Bodies::Joint id, double a, double t) {
	// wait until the program starts
	this->pauseWait();

	// lock goal
	RS_MUTEX_LOCK(&_goal_mutex);

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
	RS_MUTEX_LOCK(&_theta_mutex);
	dJointEnable(_motor[id].id);
	dJointSetAMotorAngle(_motor[id].id, 0, _motor[id].theta);
	dBodyEnable(_body[Bodies::Body]);
	RS_MUTEX_UNLOCK(&_theta_mutex);

	// unsuccessful
	RS_MUTEX_LOCK(&_motor[id].success_mutex);
	_motor[id].success = false;
	RS_MUTEX_UNLOCK(&_motor[id].success_mutex);

	// unlock goal
	RS_MUTEX_UNLOCK(&_goal_mutex);

	// success
	return 0;
}

int CLinkbot::accelJointToMaxSpeedNB(Bodies::Joint id, double a) {
	this->accelJointTimeNB(id, a, 0);

	// success
	return 0;
}

int CLinkbot::accelJointToVelocityNB(Bodies::Joint id, double a, double v) {
	this->accelJointTimeNB(id, a, v/a);

	// success
	return 0;
}

int CLinkbot::closeGripper(void) {
	double gripperAngleOld = 0;
	double gripperAngleNew = 0;
	int retval = getJointAngleInstant(Bodies::Joint1, gripperAngleNew);
	while ( fabs(gripperAngleNew - gripperAngleOld) > 0.1 ) {
		gripperAngleOld = gripperAngleNew;
		retval = retval || getJointAngleInstant(Bodies::Joint1, gripperAngleNew);
		retval = retval || moveNB(8, 0, 8);
		delaySeconds(1);
		retval = retval || getJointAngleInstant(Bodies::Joint1, gripperAngleNew);
	}
	retval = retval || moveNB(8, 0, 8);
	delaySeconds(1);
	retval = retval || holdJoints();
	return retval;
}

int CLinkbot::closeGripperNB(void) {
	// create thread
	RS_THREAD_T moving;

	// store args
	LinkbotMove *move = new LinkbotMove;
	move->robot = this;

	// motion in progress
	_motion = true;

	// start thread
	RS_THREAD_CREATE(&moving, closeGripperNBThread, (void *)move);

	// success
	return 0;
}

int CLinkbot::driveAccelCycloidalNB(double radius, double d, double t) {
	this->accelJointCycloidalNB(Bodies::Joint1,  rs::R2D(d/radius), t);
	this->accelJointCycloidalNB(Bodies::Joint3, -rs::R2D(d/radius), t);

	// success
	return 0;
}

int CLinkbot::driveAccelDistanceNB(double radius, double a, double d) {
	a = rs::D2R(a);
	this->accelJointTimeNB(Bodies::Joint1,  rs::R2D(a/radius), sqrt(2*d/a));
	this->accelJointTimeNB(Bodies::Joint3, -rs::R2D(a/radius), sqrt(2*d/a));

	// success
	return 0;
}

int CLinkbot::driveAccelHarmonicNB(double radius, double d, double t) {
	this->accelJointHarmonicNB(Bodies::Joint1,  rs::R2D(d/radius), t);
	this->accelJointHarmonicNB(Bodies::Joint3, -rs::R2D(d/radius), t);

	// success
	return 0;
}

int CLinkbot::driveAccelSmoothNB(double radius, double a0, double af, double vmax, double d) {
	this->accelJointSmoothNB(Bodies::Joint1, a0, af, vmax, d/radius);
	this->accelJointSmoothNB(Bodies::Joint3, a0, af, vmax, d/radius);

	// success
	return 0;
}

int CLinkbot::driveAccelTimeNB(double radius, double a, double t) {
	a = rs::D2R(a);
	this->accelJointTimeNB(Bodies::Joint1,  rs::R2D(a/radius), t);
	this->accelJointTimeNB(Bodies::Joint3, -rs::R2D(a/radius), t);

	// success
	return 0;
}

int CLinkbot::driveAccelToMaxSpeedNB(double radius, double a) {
	a = rs::D2R(a);
	this->accelJointTimeNB(Bodies::Joint1,  rs::R2D(a/radius), 0);
	this->accelJointTimeNB(Bodies::Joint3, -rs::R2D(a/radius), 0);

	// success
	return 0;
}

int CLinkbot::driveAccelToVelocityNB(double radius, double a, double v) {
	a = rs::D2R(a);
	this->accelJointTimeNB(Bodies::Joint1,  rs::R2D(a/radius), v/a);
	this->accelJointTimeNB(Bodies::Joint3, -rs::R2D(a/radius), v/a);

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
	_motor[Bodies::Joint3].omega = -_motor[Bodies::Joint3].omega;

	// set joint movements
	this->moveJointForeverNB(Bodies::Joint1);
	this->moveJointForeverNB(Bodies::Joint3);

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
	this->setJointSpeed(Bodies::Joint1, rs::R2D(s1/theta/rho/radius*_speed));
	this->setJointSpeed(Bodies::Joint3, rs::R2D(s2/theta/rho/radius*_speed));
	this->moveJointNB(Bodies::Joint1, rs::R2D(s1/radius));
	this->moveJointNB(Bodies::Joint3, -rs::R2D(s2/radius));
	this->delay(theta*rho/_speed*1000);

	// success
	return 0;
}

int CLinkbot::getJointAngles(double &angle1, double &angle2, double &angle3, int numReadings) {
	this->getJointAngle(Bodies::Joint1, angle1, numReadings);
	this->getJointAngle(Bodies::Joint2, angle2, numReadings);
	this->getJointAngle(Bodies::Joint3, angle3, numReadings);

	// success
	return 0;
}

int CLinkbot::getJointAnglesInstant(double &angle1, double &angle2, double &angle3) {
	this->getJointAngleInstant(Bodies::Joint1, angle1);
	this->getJointAngleInstant(Bodies::Joint2, angle2);
	this->getJointAngleInstant(Bodies::Joint3, angle3);

	// success
	return 0;
}

int CLinkbot::getJointSpeeds(double &speed1, double &speed2, double &speed3) {
	this->getJointSpeed(Bodies::Joint1, speed1);
	this->getJointSpeed(Bodies::Joint2, speed2);
	this->getJointSpeed(Bodies::Joint3, speed3);

	// success
	return 0;
}

int CLinkbot::getJointSpeedRatios(double &ratio1, double &ratio2, double &ratio3) {
	this->getJointSpeedRatio(Bodies::Joint1, ratio1);
	this->getJointSpeedRatio(Bodies::Joint2, ratio2);
	this->getJointSpeedRatio(Bodies::Joint3, ratio3);

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
	this->moveJointNB(Bodies::Joint1, angle1);
	this->moveJointNB(Bodies::Joint2, angle2);
	this->moveJointNB(Bodies::Joint3, angle3);

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
	this->moveJointToNB(Bodies::Joint1, angle1);
	this->moveJointToNB(Bodies::Joint2, angle2);
	this->moveJointToNB(Bodies::Joint3, angle3);

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
	if (this->getForm() == rs::LinkbotL)
		this->moveJointToNB(Bodies::Joint1, -angle);
	else
		this->moveToNB(-angle/2, 0, -angle/2);

	// success
	return 0;
}

int CLinkbot::recordAnglesBegin(robotRecordData_t &time, robotRecordData_t &angle1, robotRecordData_t &angle2, robotRecordData_t &angle3, double seconds, int shiftData) {
	// wait until the program starts
	this->pauseWait();

	// check if recording already
	for (int i = 0; i < _dof; i++) {
		if (_motor[i].record) { return -1; }
	}

	// store angles
	double **angles = new double * [_dof];
	angle1 = new double[RECORD_ANGLE_ALLOC_SIZE];
	angle2 = new double[RECORD_ANGLE_ALLOC_SIZE];
	angle3 = new double[RECORD_ANGLE_ALLOC_SIZE];
	angles[Bodies::Joint1] = angle1;
	angles[Bodies::Joint2] = angle2;
	angles[Bodies::Joint3] = angle3;

	// call base class recording function
	return roboSim::Robot::recordAnglesBegin(time, angles, seconds, shiftData);
}

int CLinkbot::setJointSpeeds(double speed1, double speed2, double speed3) {
	this->setJointSpeed(Bodies::Joint1, speed1);
	this->setJointSpeed(Bodies::Joint2, speed2);
	this->setJointSpeed(Bodies::Joint3, speed3);

	// success
	return 0;
}

int CLinkbot::setJointSpeedRatios(double ratio1, double ratio2, double ratio3) {
	this->setJointSpeedRatio(Bodies::Joint1, ratio1);
	this->setJointSpeedRatio(Bodies::Joint2, ratio2);
	this->setJointSpeedRatio(Bodies::Joint3, ratio3);

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
			this->moveJointNB(rsLinkbot::Bodies::Joint1, -rs::R2D(theta));
			this->moveJoint(rsLinkbot::Bodies::Joint3, -rs::R2D(theta));
		}
		// turn right
		else if (rs::R2D(angle) < -0.005) {
			this->moveJointNB(rsLinkbot::Bodies::Joint1, rs::R2D(theta));
			this->moveJoint(rsLinkbot::Bodies::Joint3, rs::R2D(theta));
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
	angle = rs::D2R(angle);												// convert to radians
	int rotations = (angle+0.1) / 2 / rs::Pi;							// number of rotations
	double rf = this->getRotation(0, 2) + 2*rs::Pi - angle + 2*rotations*rs::Pi;	// final angle of rotation
	double left = rf - this->getRotation(0, 2);							// radians if turning left
	double right = rf - (2 * rs::Pi + this->getRotation(0, 2));			// radians if turning right
	if (fabs(left) < rs::Epsilon)			angle = -right;				// if left==0, use right angle
	else if (fabs(right) < rs::Epsilon)		angle = -left;				// if right==0, use left angle
	else if (fabs(left) < fabs(right))		angle = left;				// turn smaller of left or right
	else									angle = right;				// other option

	// get speed of robot
	double *speed = new double[_dof]();
	this->getJointSpeeds(speed[0], speed[1], speed[2]);

	// turn toward new postition until pointing correctly
	while (fabs(angle) > 0.005) {
		// calculate wheel theta
		double theta = fabs((angle*this->convert(_trackwidth, 0)) / (2 * radius));

		// turn left
		if (rs::R2D(angle) > 0.005) {
			this->moveJointNB(rsLinkbot::Bodies::Joint1, -rs::R2D(theta));
			this->moveJoint(rsLinkbot::Bodies::Joint3, -rs::R2D(theta));
		}
		// turn right
		else if (rs::R2D(angle) < -0.005) {
			this->moveJointNB(rsLinkbot::Bodies::Joint1, rs::R2D(theta));
			this->moveJoint(rsLinkbot::Bodies::Joint3, rs::R2D(theta));
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
	RS_COND_ACTION(&move->robot->_motion_cond, &move->robot->_motion_mutex, move->robot->_motion = false);

	// cleanup
	delete move;

	// success
	return NULL;
}

