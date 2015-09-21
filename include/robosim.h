#ifndef ROBOSIM_H_
#define ROBOSIM_H_

#include <iostream>

#include <rs/Enum>
#include <rs/Macros>
#include <rsScene/Scene>
#include <rsScene/ModularRobot>
#include <rsSim/Sim>
#include <rsXML/Robot>
#include <rsXML/Reader>
#include <rsXML/Writer>
#include <rsCallback/Callback>
#include <rsCallback/ModularRobot>

// macros
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

	private:
		bool _units;		// SI (true) or customary (false)
};

#endif	// ROBOSIM_H_

