#ifndef ROBOT_H_
#define ROBOT_H_

#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include <rs/Macros>
#include <rs/Threads>
#include <rsSim/Robot>

#include "robosim.h"
#include "rgbhashtable.h"

#define RECORD_ANGLE_ALLOC_SIZE 16

// individual
class LIBRSEXPORT Robot : virtual public rsSim::Robot {
	// public functions
	public:
		Robot(int, int);
		virtual ~Robot(void);

		int blinkLED(double, int);
		int delay(double);
		int delaySeconds(double);
		int disableRecordDataShift(void);
		int disconnect(void);
		int driveAngle(double);
		virtual int driveAngleNB(double);
		int driveDistance(double, double);
		int driveDistanceNB(double, double);
		int driveForever(void);
		virtual int driveForeverNB(void);
		int driveTime(double);
		int driveTimeNB(double);
		int drivexy(double, double, double, double);
		int drivexyNB(double, double, double, double);
		virtual int drivexyTo(double, double, double, double);
		int drivexyToNB(double, double, double, double);
		int drivexyToArrayNB(double*, double*, int, double, double);
		int drivexyToFunc(double, double, int, double (*func)(double), double, double);
		int drivexyToFuncNB(double, double, int, double (*func)(double), double, double);
		int drivexyToFuncSmooth(double, double, int, double (*func)(double), double, double);
		virtual int drivexyToSmooth(double, double, double, double, double, double, double, double);
		int drivexyWait(void);
		int enableRecordDataShift(void);
		int getAccelerometerData(double&, double&, double&);
		int getBatteryVoltage(double&);
		int getDistance(double&, double);
		int getFormFactor(int&);
		int getID(void);
		int getJointAngle(int, double&, int = 10);
		int getJointAngleInstant(int, double&);
		int getJointMaxSpeed(int, double&);
		int getJointSafetyAngle(double&);
		int getJointSafetyAngleTimeout(double&);
		int getJointSpeed(int, double&);
		int getJointSpeedRatio(int, double&);
		int getLEDColorName(char[]);
		int getLEDColorRGB(int&, int&, int&);
		int getPosition(double&, double&, double&);
		int getxy(double&, double&);
		int holdJointsAtExit(void);
		int isConnected(void);
		int isMoving(void);
		int isNotMoving(void);
		int moveForeverNB(void);
		int moveJointByPowerNB(int, int);
		int moveJointForeverNB(int);
		int moveJointTime(int, double);
		int moveJointTimeNB(int, double);
		int moveJointToByTrackPos(int, double);
		int moveJointToByTrackPosNB(int, double);
		int moveTime(double);
		int moveTimeNB(double);
		int moveToZero(void);
		int moveToZeroNB(void);
		int moveWait(void);
		int recordAngleBegin(int, robotRecordData_t&, robotRecordData_t&, double, int = 1);
		int recordAngleEnd(int, int&);
		int recordAnglesEnd(int&);
		int recordDistanceBegin(int, robotRecordData_t&, robotRecordData_t&, double, double, int = 1);
		int recordDistanceEnd(int, int&);
		int recordDistanceOffset(double);
		int recordWait(void);
		int recordxyBegin(robotRecordData_t&, robotRecordData_t&, double, int = 1);
		int recordxyEnd(int&);
		int relaxJoint(int);
		int relaxJoints(void);
		int resetToZero(void);
		int resetToZeroNB(void);
		int setBuzzerFrequency(int, double);
		int setBuzzerFrequencyOff(void);
		int setBuzzerFrequencyOn(int);
		int setLEDColor(char*);
		int setLEDColorRGB(int, int, int);
		int setJointSafetyAngle(double);
		int setJointSafetyAngleTimeout(double);
		int setJointSpeed(int, double);
		int setJointSpeedRatio(int, double);
		int setSpeed(double, double);
		int systemTime(double&);
		int traceOff(void);
		int traceOn(void);
		virtual int turnLeft(double, double, double);
		int turnLeftNB(double, double, double);
		virtual int turnRight(double, double, double);
		int turnRightNB(double, double, double);

	protected:
		double convert(double, int);
		int recordAnglesBegin(robotRecordData_t&, robotRecordData_t*&, double, int);

	protected:
		struct Recording {
			Robot *robot;		// robot
			int id;				// joint to record
			int num;			// number of points
			int msecs;			// ms between data points
			double *time;		// array for time
			double **ptime;		// pointer to time array
			double **angle;		// array of angles
			double ***pangle;	// point to array of angles
		};
		int _shift_data;			// shift recorded data or not
		int _g_shift_data;			// globally shift data for robot
		int _g_shift_data_en;		// globally shift data for robot enable/disable flag
		MUTEX_T _active_mutex;		// active recording
		COND_T _active_cond;		// active recording
		MUTEX_T _motion_mutex;		// motion in progress
		COND_T _motion_cond;		// motion in progress
		MUTEX_T _recording_mutex;	// recording data point
		COND_T _recording_cond;		// recording data  point
		int _leftWheel;
		int _rightWheel;

	private:
		bool is_shift_enabled(void);
		static void* driveDistanceThread(void*);
		static void* driveTimeNBThread(void*);
		static void* drivexyToThread(void*);
		static void* drivexyToArrayThread(void*);
		static void* drivexyToFuncThread(void*);
		static void* moveJointTimeNBThread(void*);
		static void* moveTimeNBThread(void*);
		static void* recordAngleThread(void*);
		static void* recordAngleBeginThread(void*);
		static void* recordAnglesBeginThread(void*);
		static void* recordxyBeginThread(void*);
		static void* turnThread(void*);
};

// group
template<class T> class LIBRSEXPORT RobotGroup {
	// public api
	public:
		RobotGroup(void) { };
		virtual ~RobotGroup(void) { };

		int addRobot(T &robot) {
			_robots.push_back(&robot);
			return 0;
		}

		int addRobots(T robots[], int num) {
			for (int i = 0; i < num; i++) {
				_robots.push_back(&robots[i]);
			}
			return 0;
		}

		int blinkLED(double delay, int num) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->blinkLED(delay, num);
			}
			return 0;
		}

		int connect(void) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->connect();
			}
			return 0;
		}

		int disconnect(void) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->disconnect();
			}
			return 0;
		}

		int driveAngle(double angle) {
			driveAngleNB(angle);
			return moveWait();
		}

		int driveAngleNB(double angle) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->driveAngleNB(angle);
			}
			return 0;
		}

		int driveDistance(double distance, double radius) {
			driveDistanceNB(distance, radius);
			return moveWait();
		}

		int driveDistanceNB(double distance, double radius) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->driveDistanceNB(distance, radius);
			}
			return 0;
		}

		int driveForever(void) {
			driveForeverNB();
			return moveWait();
		}

		int driveForeverNB(void) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->driveForeverNB();
			}
			return 0;
		}

		int driveTime(double seconds) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->driveTimeNB(seconds);
			}
			#ifdef _WIN32
			Sleep(seconds * 1000);
			#else
			usleep(seconds * 1000000);
			#endif
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->holdJoints();
			}
			return 0;
		}

		int driveTimeNB(double seconds) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->driveTimeNB(seconds);
			}
			return 0;
		}

		int holdJoint(int id) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->holdJoint(id);
			}
			return 0;
		}

		int holdJoints(void) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->holdJoints();
			}
			return 0;
		}

		int holdJointsAtExit(void) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->holdJointsAtExit();
			}
			return 0;
		}

		int isMoving(void) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				if(_robots[i]->isMoving()) return 1;
			}
			return 0;
		}

		int isNotMoving(void) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				if(_robots[i]->isNotMoving()) return 1;
			}
			return 0;
		}

		int moveForeverNB(void) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->moveForeverNB();
			}
			return 0;
		}

		int moveJoint(int id, double angle) {
			moveJointNB(id, angle);
			return moveWait();
		}

		int moveJointNB(int id, double angle) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->moveJointNB(id, angle);
			}
			return 0;
		}

		int moveJointByPowerNB(int id, int power) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->moveJointByPowerNB(id, power);
			}
			return 0;
		}

		int moveJointForeverNB(int id) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->moveJointForeverNB(id);
			}
			return 0;
		}

		int moveJointTime(int id, double seconds) {
			this->moveJointTimeNB(id, seconds);
			#ifdef _WIN32
			Sleep(seconds * 1000);
			#else
			usleep(seconds * 1000000);
			#endif
			return 0;
		}

		int moveJointTimeNB(int id, double seconds) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->moveJointTimeNB(id, seconds);
			}
			return 0;
		}

		int moveJointTo(int id, double angle) {
			moveJointToNB(id, angle);
			return moveWait();
		}

		int moveJointToNB(int id, double angle) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->moveJointToNB(id, angle);
			}
			return 0;
		}

		int moveJointToByTrackPos(int id, double angle) {
			moveJointToByTrackPosNB(id, angle);
			return moveJointWait(id);
		}

		int moveJointToByTrackPosNB(int id, double angle) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->moveJointToByTrackPosNB(id, angle);
			}
			return 0;
		}

		int moveJointWait(int id) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->moveJointWait(id);
			}
			return 0;
		}

		int moveTime(double seconds) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->moveForeverNB();
			}
			#ifdef _WIN32
			Sleep(seconds * 1000);
			#else
			usleep(seconds * 1000000);
			#endif
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->holdJoints();
			}
			return 0;
		}

		int moveTimeNB(double seconds) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->moveTimeNB(seconds);
			}
			return 0;
		}

		int moveToZero(void) {
			moveToZeroNB();
			return moveWait();
		}

		int moveToZeroNB(void) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->moveToZeroNB();
			}
			return 0;
		}

		int moveWait(void) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->moveWait();
			}
			return 0;
		}

		int relaxJoint(int id) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->relaxJoint(id);
			}
			return 0;
		}

		int relaxJoints(void) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->relaxJoints();
			}
			return 0;
		}

		int resetToZero(void) {
			resetToZeroNB();
			return moveWait();
		}

		int resetToZeroNB(void) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->resetToZeroNB();
			}
			return 0;
		}

		int setBuzzerFrequency(int frequency, double time) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->setBuzzerFrequency(frequency, time);
			}
			return 0;
		}

		int setBuzzerFrequencyOff(void) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->setBuzzerFrequencyOff();
			}
			return 0;
		}

		int setBuzzerFrequencyOn(int frequency) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->setBuzzerFrequencyOn(frequency);
			}
			return 0;
		}

		int setLEDColor(char *color) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->setLEDColor(color);
			}
			return 0;
		}

		int setLEDColorRGB(int r, int g, int b) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->setLEDColorRGB(r, g, b);
			}
			return 0;
		}

		int setJointSafetyAngle(double angle) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->setJointSafetyAngle(angle);
			}
			return 0;
		}

		int setJointSafetyAngleTimeout(double seconds) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->setJointSafetyAngleTimeout(seconds);
			}
			return 0;
		}

		int setJointSpeed(int id, double speed) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->setJointSpeed(id, speed);
			}
			return 0;
		}

		int setJointSpeedRatio(int id, double ratio) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->setJointSpeedRatio(id, ratio);
			}
			return 0;
		}

		int setSpeed(double speed, double radius) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->setSpeed(speed, radius);
			}
			return 0;
		}

		int traceOff(void) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->traceOff();
			}
			return 0;
		}

		int traceOn(void) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->traceOn();
			}
			return 0;
		}

		int turnLeft(double angle, double radius, double trackwidth) {
			this->turnLeftNB(angle, radius, trackwidth);
			return moveWait();
		}

		int turnLeftNB(double angle, double radius, double trackwidth) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->turnLeftNB(angle, radius, trackwidth);
			}
			return 0;
		}

		int turnRight(double angle, double radius, double trackwidth) {
			this->turnRightNB(angle, radius, trackwidth);
			return moveWait();
		}

		int turnRightNB(double angle, double radius, double trackwidth) {
			for (unsigned int i = 0; i < _robots.size(); i++) {
				_robots[i]->turnRightNB(angle, radius, trackwidth);
			}
			return 0;
		}

	// data members
	protected:
		std::vector<T*> _robots;
		double _d;
		int _i;
};

// motion threading
struct RobotMove {
	Robot *robot;
	char *expr;
	double x, y, *px, *py, radius, trackwidth;
	double (*func)(double x);
	int i;
};

// robosim
extern RoboSim *g_sim;

#endif // ROBOT_H_

