#ifndef ROBOSIM_H_
#define ROBOSIM_H_

#include <iostream>

#include <rs/Enum>
#include <rs/Macros>
#include <rsCallback/Callback>
#include <rsCallback/ModularRobot>
#include <rsScene/ModularRobot>
#include <rsScene/Scene>
#include <rsSim/Sim>
#include <rsXML/Reader>
#include <rsXML/Robot>
#include <rsXML/Writer>

// macros
#define JOINT1 0
#define JOINT2 1
#define JOINT3 2
#define JOINT4 3
#define angle2distance(radius, angle) ((radius) * (angle * 0.01745329251994329547))
#define distance2angle(radius, distance) (((distance)/(radius))*57.29577951308232286465)
#define DEPRECATED(from, to) fprintf(_stderr, "Warning: The function \"%s()\" is deprecated. Please use \"%s()\"\n" , from, to)

// recorded data
typedef double* robotRecordData_t;

class LIBRSEXPORT RoboSim : virtual public rsScene::Scene, virtual public rsSim::Sim, virtual public rsXML::Reader, virtual public rsCallback::Callback {
	public:
		RoboSim(const char*, bool);
		virtual ~RoboSim(void) { };

		int addRobot(rsSim::Robot*, rsScene::Robot*, rsCallback::Robot*);
		int addRobot(rsSim::ModularRobot*, rsScene::ModularRobot*, rsCallback::ModularRobot*);
		int deleteRobot(int);
		bool getUnits(void);
		void keyPressed(int);
		void saveState(char*);
		void updateClock(void);

	private:
		bool _units;		// SI (true) or customary (false)
		std::vector< std::tuple<rsXML::Conn*, int> > _delay;
};

#endif	// ROBOSIM_H_

