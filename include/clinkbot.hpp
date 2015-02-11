#ifndef CLINKBOT_HPP_
#define CLINKBOT_HPP_

#include <rsSim/sim.hpp>
#include <rsSim/linkbot.hpp>
//#include <rsSim/linkbotgroup.hpp>

#include "robosim.hpp"

class CLinkbotI : public rsSim::LinkbotI {
	public:
		CLinkbotI(void);
		virtual ~CLinkbotI(void);

		int connect(char* = NULL, int = 3);
};

class CLinkbotL : public rsSim::LinkbotL {
	public:
		CLinkbotL(void);
		virtual ~CLinkbotL(void);

		int connect(char* = NULL, int = 3);
};

class CLinkbotT : public rsSim::LinkbotT {
	public:
		CLinkbotT(void);
		virtual ~CLinkbotT(void);

		int connect(char* = NULL, int = 3);
};

/*class DLLIMPORT CLinkbotTGroup : public rsSim::CLinkbotTGroup {};
class DLLIMPORT CLinkbotIGroup : public rsSim::CLinkbotIGroup {};
class DLLIMPORT CLinkbotLGroup : public rsSim::CLinkbotLGroup {};*/

// simulation
extern RoboSim *g_sim;

#endif // CLINKBOT_HPP_

