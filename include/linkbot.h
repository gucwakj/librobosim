#ifndef LINKBOT_H_
#define LINKBOT_H_

#include <rsSim/Sim>
#include <rsSim/Linkbot>

#include "robosim.h"

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

#endif // LINKBOT_H_

