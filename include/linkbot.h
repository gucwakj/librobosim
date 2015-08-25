#ifndef LINKBOT_H_
#define LINKBOT_H_

#include <rs/Macros>
#include <rsSim/Sim>
#include <rsSim/Linkbot>

#include "robosim.h"
#include "robot.h"

// individual
class LIBRSEXPORT CLinkbot : virtual public rsSim::Linkbot, virtual public Robot {
	public:
		CLinkbot(const char* = NULL, bool = true);
		virtual ~CLinkbot(void);

		int accelJointAngleNB(rsLinkbot::Bodies::Joint, double, double);
		int accelJointCycloidalNB(rsLinkbot::Bodies::Joint, double, double);
		int accelJointHarmonicNB(rsLinkbot::Bodies::Joint, double, double);
		int accelJointSmoothNB(rsLinkbot::Bodies::Joint, double, double, double, double);
		int accelJointTimeNB(rsLinkbot::Bodies::Joint, double, double);
		int accelJointToMaxSpeedNB(rsLinkbot::Bodies::Joint, double);
		int accelJointToVelocityNB(rsLinkbot::Bodies::Joint, double, double);
		int closeGripper(void);
		int closeGripperNB(void);
		int driveAccelCycloidalNB(double, double, double);
		int driveAccelDistanceNB(double, double, double);
		int driveAccelHarmonicNB(double, double, double);
		int driveAccelSmoothNB(double, double, double, double, double);
		int driveAccelTimeNB(double, double, double);
		int driveAccelToMaxSpeedNB(double, double);
		int driveAccelToVelocityNB(double, double, double);
		int driveAngleNB(double);
		int driveForeverNB(void);
		int drivexyToSmooth(double, double, double, double, double, double, double, double);
		int getJointAngles(double&, double&, double&, int = 10);
		int getJointAnglesInstant(double&, double&, double&);
		int getJointSpeeds(double&, double&, double&);
		int getJointSpeedRatios(double&, double&, double&);
		int move(double, double, double);
		int moveNB(double, double, double);
		int moveTo(double, double, double);
		int moveToNB(double, double, double);
		int moveToByTrackPos(double, double, double);
		int moveToByTrackPosNB(double, double, double);
		int openGripper(double);
		int openGripperNB(double);
		int recordAnglesBegin(robotRecordData_t&, robotRecordData_t&, robotRecordData_t&, robotRecordData_t&, double, int = 1);
		int setJointSpeeds(double, double, double);
		int setJointSpeedRatios(double, double, double);
		int turnLeft(double, double, double);
		int turnRight(double, double, double);

	private:
		static void* closeGripperNBThread(void*);
};

class LIBRSEXPORT CLinkbotI : public CLinkbot {
	public:
		CLinkbotI(const char *name = NULL, bool pause = true) : rsRobots::Robot(rs::LINKBOTI), rsRobots::Linkbot(rs::LINKBOTI), rsSim::Linkbot(rs::LINKBOTI), Robot(rsLinkbot::Bodies::Joint1, rsLinkbot::Bodies::Joint3), CLinkbot(name, pause) { };
};

class LIBRSEXPORT CLinkbotL : public CLinkbot {
	public:
		CLinkbotL(const char *name = NULL, bool pause = true) : rsRobots::Robot(rs::LINKBOTL), rsRobots::Linkbot(rs::LINKBOTL), rsSim::Linkbot(rs::LINKBOTL), Robot(rsLinkbot::Bodies::Joint1, rsLinkbot::Bodies::Joint2), CLinkbot(name, pause) { };
};

// group
class LIBRSEXPORT CLinkbotGroup : public RobotGroup<CLinkbot> {
	public:
		CLinkbotGroup(void) : RobotGroup<CLinkbot>() { };
		virtual ~CLinkbotGroup(void) { };

		inline int accelJointAngleNB(rsLinkbot::Bodies::Joint id, double a, double angle) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->accelJointAngleNB(id, a, angle);
			}
			return 0;
		}

		inline int accelJointCycloidalNB(rsLinkbot::Bodies::Joint id, double angle, double t) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->accelJointCycloidalNB(id, angle, t);
			}
			return 0;
		}

		inline int accelJointHarmonicNB(rsLinkbot::Bodies::Joint id, double angle, double t) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->accelJointHarmonicNB(id, angle, t);
			}
			return 0;
		}

		inline int accelJointSmoothNB(rsLinkbot::Bodies::Joint id, double a0, double af, double vmax, double angle) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->accelJointSmoothNB(id, a0, af, vmax, angle);
			}
			return 0;
		}

		inline int accelJointTimeNB(rsLinkbot::Bodies::Joint id, double a, double t) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->accelJointTimeNB(id, a, t);
			}
			return 0;
		}

		inline int accelJointToMaxSpeedNB(rsLinkbot::Bodies::Joint id, double a) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->accelJointToMaxSpeedNB(id, a);
			}
			return 0;
		}

		inline int accelJointToVelocityNB(rsLinkbot::Bodies::Joint id, double a, double v) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->accelJointToVelocityNB(id, a, v);
			}
			return 0;
		}

		inline int closeGripper(void) {
			for (unsigned int i = 0; i < _robots.size()-1; i++) {
				_robots[i]->closeGripperNB();
			}
			_robots.back()->closeGripper();
			return 0;
		}

		inline int closeGripperNB(void) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->closeGripperNB();
			}
			return 0;
		}

		inline int driveAccelCycloidalNB(double radius, double d, double t) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->driveAccelCycloidalNB(radius, d, t);
			}
			return 0;
		}

		inline int driveAccelDistanceNB(double radius, double a, double d) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->driveAccelDistanceNB(radius, a, d);
			}
			return 0;
		}

		inline int driveAccelHarmonicNB(double radius, double d, double t) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->driveAccelHarmonicNB(radius, d, t);
			}
			return 0;
		}

		inline int driveAccelSmoothNB(double radius, double a0, double af, double vmax, double d) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->driveAccelSmoothNB(radius, a0, af, vmax, d);
			}
			return 0;
		}

		inline int driveAccelTimeNB(double radius, double a, double t) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->driveAccelTimeNB(radius, a, t);
			}
			return 0;
		}

		inline int driveAccelToMaxSpeedNB(double radius, double a) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->driveAccelToMaxSpeedNB(radius, a);
			}
			return 0;
		}

		inline int driveAccelToVelocityNB(double radius, double a, double v) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->driveAccelToVelocityNB(radius, a, v);
			}
			return 0;
		}

		inline int move(double angle1, double angle2, double angle3) {
			moveNB(angle1, angle2, angle3);
			return moveWait();
		}

		inline int moveNB(double angle1, double angle2, double angle3) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->moveNB(angle1, angle2, angle3);
			}
			return 0;
		}

		inline int moveTo(double angle1, double angle2, double angle3) {
			moveToNB(angle1, angle2, angle3);
			return moveWait();
		}

		inline int moveToNB(double angle1, double angle2, double angle3) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->moveToNB(angle1, angle2, angle3);
			}
			return 0;
		}

		inline int moveToByTrackPos(double angle1, double angle2, double angle3) {
			moveToByTrackPosNB(angle1, angle2, angle3);
			return moveWait();
		}

		inline int moveToByTrackPosNB(double angle1, double angle2, double angle3) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->moveToByTrackPosNB(angle1, angle2, angle3);
			}
			return 0;
		}

		inline int openGripper(double angle) {
			openGripperNB(angle);
			return moveWait();
		}

		inline int openGripperNB(double angle) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->openGripperNB(angle);
			}
			return 0;
		}

		inline int setJointSpeeds(double speed1, double speed2, double speed3) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->setJointSpeeds(speed1, speed2, speed3);
			}
			return 0;
		}

		inline int setJointSpeedRatios(double ratio1, double ratio2, double ratio3) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->setJointSpeedRatios(ratio1, ratio2, ratio3);
			}
			return 0;
		}
};

class LIBRSEXPORT CLinkbotIGroup : public CLinkbotGroup { };
class LIBRSEXPORT CLinkbotLGroup : public CLinkbotGroup { };

// motion threading
struct LinkbotMove {
	CLinkbot *robot;
	char *expr;
	double x, y, radius, trackwidth;
	double (*func)(double x);
	int i;
};

// robosim
extern RoboSim *g_sim;

#endif // LINKBOT_H_

