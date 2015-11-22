#ifndef MINIDOF_H_
#define MINIDOF_H_

#include <rs/Macros>
#include <rsCallback/MiniDof>
#include <rsScene/MiniDof>
#include <rsSim/Sim>
#include <rsSim/MiniDof>

#include "robosim.h"
#include "robot.h"

// individual
class LIBRSEXPORT CMiniDof : virtual public rsScene::MiniDof, virtual public rsSim::MiniDof, virtual public rsCallback::MiniDof, virtual public roboSim::Robot {
	public:
		CMiniDof(int, char* = "", bool = true);
		virtual ~CMiniDof(void);
};

// group
class LIBRSEXPORT CMiniDofGroup : public roboSim::RobotGroup<CMiniDof> {
	public:
		CMiniDofGroup(void) : RobotGroup<CMiniDof>() { };
		virtual ~CMiniDofGroup(void) { };
};

// robosim
extern RoboSim *g_sim;

#endif // MINIDOF_H_

