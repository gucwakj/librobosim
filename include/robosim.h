#ifndef ROBOSIM_H_
#define ROBOSIM_H_

#include <iostream>

#include <rs/Enum>
#include <rsScene/Scene>
#include <rsSim/Sim>
#include <rsXML/Robot>
#include <rsXML/Reader>
#include <rsXML/Writer>
#include <rsCallback/Callback>

// macros
#define angle2distance(radius, angle) ((radius) * (angle * 0.01745329251994329547))
#define distance2angle(radius, distance) (((distance)/(radius))*57.29577951308232286465)
#define DEPRECATED(from, to) fprintf(_stderr, "Warning: The function \"%s()\" is deprecated. Please use \"%s()\"\n" , from, to)

// recorded data
typedef double* robotRecordData_t;

class RoboSim : public rsScene::Scene, public rsSim::Sim, public rsXML::Reader, public rsCallback::Callback {
	public:
		RoboSim(char*, int);
		virtual ~RoboSim(void) { };

		int addRobot(rsSim::Robot*);
		int addRobot(rsSim::ModularRobot*);
		int deleteRobot(int);
		void keyPressed(int);
		void saveState(char*);
};

#endif	// ROBOSIM_H_

