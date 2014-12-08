#ifndef ROBOSIM_HPP_
#define ROBOSIM_HPP_

#include <iostream>

#include <rsScene/scene.hpp>
#include <rsSim/sim.hpp>
#include <rsXML/store.hpp>
#include <rsXML/robot.hpp>
#include <rsCallback/callback.hpp>

class RoboSim : public rsSim::Sim, public rsXML::Store, public rsScene::Scene , public rsCallback::Callback {
	public:
		RoboSim(char*, int);
		virtual ~RoboSim(void);

		int addRobot(rsSim::ModularRobot*);
		int deleteRobot(int);
		void keyPressed(int);
};

#endif	// ROBOSIM_HPP_

