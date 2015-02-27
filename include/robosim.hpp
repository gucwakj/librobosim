#ifndef ROBOSIM_HPP_
#define ROBOSIM_HPP_

#include <iostream>

#include <rs/Enum>
#include <rsScene/Scene>
#include <rsSim/Sim>
#include <rsXML/Robot>
#include <rsXML/Reader>
#include <rsXML/Writer>
#include <rsCallback/Callback>

class RoboSim : public rsSim::Sim, public rsXML::Reader, public rsScene::Scene , public rsCallback::Callback {
	public:
		RoboSim(char*, int);
		virtual ~RoboSim(void);

		int addRobot(rsSim::Robot*);
		int addRobot(rsSim::ModularRobot*);
		int deleteRobot(int);
		void keyPressed(int);
		void saveState(char*);
};

#endif	// ROBOSIM_HPP_

