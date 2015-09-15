#ifndef MINDSTORMS_H_
#define MINDSTORMS_H_

#include <rs/Macros>
#include <rsCallback/Mindstorms>
#include <rsScene/Mindstorms>
#include <rsSim/Sim>
#include <rsSim/Mindstorms>

#include "robosim.h"
#include "robot.h"

// individual
class LIBRSEXPORT CMindstorms : virtual public rsScene::Mindstorms, virtual public rsSim::Mindstorms, virtual public rsCallback::Mindstorms, virtual public roboSim::Robot {
	public:
		CMindstorms(const char* = NULL, bool = true);
		virtual ~CMindstorms(void);

		int getJointAngles(double&, double&, double&, double&, int = 10);
		int getJointAnglesInstant(double&, double&, double&, double&);
		int getJointSpeeds(double&, double&, double&, double&);
		int getJointSpeedRatios(double&, double&, double&, double&);
		int move(double, double, double, double = 0);
		int moveNB(double, double, double, double = 0);
		int moveTo(double, double, double, double = 0);
		int moveToNB(double, double, double, double = 0);
		int moveToByTrackPos(double, double, double, double = 0);
		int moveToByTrackPosNB(double, double, double, double = 0);
		int recordAnglesBegin(robotRecordData_t&, robotRecordData_t&, robotRecordData_t&, double, int = 1);
		int setJointSpeeds(double, double, double, double = 0);
		int setJointSpeedRatios(double, double, double, double = 0);
};

// group
class LIBRSEXPORT CMindstormsGroup : public roboSim::RobotGroup<CMindstorms> {
	public:
		CMindstormsGroup(void) : RobotGroup<CMindstorms>() { };
		virtual ~CMindstormsGroup(void) { };

		inline int move(double angle1, double angle2, double angle3, double angle4 = 0) {
			this->moveNB(angle1, angle2, angle3, angle4);
			return this->moveWait();
		}
		inline int moveNB(double angle1, double angle2, double angle3, double angle4 = 0) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->moveNB(angle1, angle2, angle3, angle4);
			}
			return 0;
		}
		inline int moveTo(double angle1, double angle2, double angle3, double angle4 = 0) {
			this->moveToNB(angle1, angle2, angle3, angle4);
			return moveWait();
		}
		inline int moveToNB(double angle1, double angle2, double angle3, double angle4 = 0) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->moveToNB(angle1, angle2, angle3, angle4);
			}
			return 0;
		}
		inline int moveToByTrackPos(double angle1, double angle2, double angle3, double angle4 = 0) {
			this->moveToNB(angle1, angle2, angle3, angle4);
			return moveWait();
		}
		inline int moveToByTrackPosNB(double angle1, double angle2, double angle3, double angle4 = 0) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->moveToByTrackPosNB(angle1, angle2, angle3, angle4);
			}
			return 0;
		}
		inline int setJointSpeeds(double speed1, double speed2, double speed3, double speed4 = 0) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->setJointSpeeds(speed1, speed2, speed3, speed4);
			}
			return 0;
		}
		inline int setJointSpeedRatios(double ratio1, double ratio2, double ratio3, double ratio4) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->setJointSpeedRatios(ratio1, ratio2, ratio3, ratio4);
			}
			return 0;
		}
};

// robosim
extern RoboSim *g_sim;

#endif // MINDSTORMS_H_

