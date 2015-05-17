#ifndef MINDSTORMS_H_
#define MINDSTORMS_H_

#include <rs/Macros>
#include <rsSim/Sim>
#include <rsSim/Mindstorms>

#include "robosim.h"
#include "robot.h"

// individual
class LIBRSEXPORT CMindstorms : public rsSim::Mindstorms, public Robot {
	public:
		CMindstorms(char* = NULL, bool = true);
		virtual ~CMindstorms(void);

		int getJointAngles(double&, double&, int = 10);
		int getJointAnglesInstant(double&, double&);
		int getJointSpeeds(double&, double&);
		int getJointSpeedRatios(double&, double&);
		int move(double, double);
		int moveNB(double, double);
		int moveTo(double, double);
		int moveToNB(double, double);
		int moveToByTrackPos(double, double);
		int moveToByTrackPosNB(double, double);
		int recordAngles(double[], double[], double[], int, double, int = 1);
		int recordAnglesBegin(robotRecordData_t&, robotRecordData_t&, robotRecordData_t&, double, int = 1);
		int recordDistancesBegin(robotRecordData_t&, robotRecordData_t&, robotRecordData_t&, double, double, int = 1);
		int setJointSpeeds(double, double);
		int setJointSpeedRatios(double, double);
};

// group
class LIBRSEXPORT CMindstormsGroup : public RobotGroup<CMindstorms> {
	public:
		CMindstormsGroup(void) : RobotGroup<CMindstorms>() { };
		virtual ~CMindstormsGroup(void) { };

		inline int move(double angle1, double angle2) {
			this->moveNB(angle1, angle2);
			return this->moveWait();
		}
		inline int moveNB(double angle1, double angle2) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->moveNB(angle1, angle2);
			}
			return 0;
		}
		inline int moveTo(double angle1, double angle2) {
			moveToNB(angle1, angle2);
			return moveWait();
		}
		inline int moveToNB(double angle1, double angle2) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->moveToNB(angle1, angle2);
			}
			return 0;
		}
		inline int moveToByTrackPos(double angle1, double angle2) {
			moveToNB(angle1, angle2);
			return moveWait();
		}
		inline int moveToByTrackPosNB(double angle1, double angle2) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->moveToByTrackPosNB(angle1, angle2);
			}
			return 0;
		}
		inline int setJointSpeeds(double speed1, double speed2) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->setJointSpeeds(speed1, speed2);
			}
			return 0;
		}
		inline int setJointSpeedRatios(double ratio1, double ratio2) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->setJointSpeedRatios(ratio1, ratio2);
			}
			return 0;
		}
};

// simulation
extern RoboSim *g_sim;

#endif // MINDSTORMS_H_

