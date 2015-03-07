#ifndef ROBOT_H_
#define ROBOT_H_

#include <rs/Macros>
#include <rsSim/Robot>

#include "robosim.h"

class Robot : virtual public rsSim::Robot {
	// public functions
	public:
		Robot(int, int);
		virtual ~Robot(void);

		int blinkLED(double, int);
		int delay(double);
		int delaySeconds(double);
		int disableRecordDataShift(void);
		int disconnect(void);
		int driveBackward(double);
		int driveBackwardNB(double);
		int driveDistance(double, double);
		int driveDistanceNB(double, double);
		int driveForever(void);
		virtual int driveForeverNB(void);
		int driveForward(double);
		virtual int driveForwardNB(double);
		int driveTime(double);
		int driveTimeNB(double);
		int drivexy(double, double, double, double);
		int drivexyNB(double, double, double, double);
		virtual int drivexyTo(double, double, double, double);
		int drivexyToNB(double, double, double, double);
		int drivexyToFunc(double, double, int, double (*func)(double), double, double);
		int drivexyToFuncNB(double, double, int, double (*func)(double), double, double);
		int drivexyToFuncSmooth(double, double, int, double (*func)(double), double, double);
		int drivexyToPoly(double, double, int, char*, double, double);
		int drivexyToPolyNB(double, double, int, char*, double, double);
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
		int getxy(double&, double&);
		int holdJoint(int);
		int holdJoints(void);
		int holdJointsAtExit(void);
		int isConnected(void);
		int isMoving(void);
		int isNotMoving(void);
		int moveForeverNB(void);
		int moveJoint(int, double);
		int moveJointNB(int, double);
		int moveJointByPowerNB(int, int);
		int moveJointForeverNB(int);
		int moveJointTime(int, double);
		int moveJointTimeNB(int, double);
		int moveJointTo(int, double);
		int moveJointToNB(int, double);
		int moveJointToByTrackPos(int, double);
		int moveJointToByTrackPosNB(int, double);
		int moveJointWait(int);
		int moveTime(double);
		int moveTimeNB(double);
		int moveToZero(void);
		int moveToZeroNB(void);
		int moveWait(void);
		int recordAngle(int, double[], double[], int, double, int = 1);
		int recordAngleBegin(int, robotRecordData_t&, robotRecordData_t&, double, int = 1);
		int recordAngleEnd(int, int&);
		int recordAnglesEnd(int&);
		int recordDistanceBegin(int, robotRecordData_t&, robotRecordData_t&, double, double, int = 1);
		int recordDistanceEnd(int, int&);
		int recordDistanceOffset(double);
		int recordDistancesEnd(int&);
		int recordWait(void);
		int recordxyBegin(robotRecordData_t&, robotRecordData_t&, double, int = 1);
		int recordxyEnd(int&);
		int relaxJoint(int id);
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
		int recordAngles(double*, double**, int, double, int);
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
		static void* drivexyToFuncThread(void*);
		static void* drivexyToPolyThread(void*);
		static void* moveJointTimeNBThread(void*);
		static void* moveTimeNBThread(void*);
		static void* recordAngleThread(void*);
		static void* recordAngleBeginThread(void*);
		static void* recordAnglesThread(void*);
		static void* recordAnglesBeginThread(void*);
		static void* recordxyBeginThread(void*);
		static void* turnThread(void*);
};

// motion threading
struct RobotMove {
	Robot *robot;
	char *expr;
	double x, y, radius, trackwidth;
	double (*func)(double x);
	int i;
};

// simulation
extern RoboSim *g_sim;

#endif // ROBOT_H_

