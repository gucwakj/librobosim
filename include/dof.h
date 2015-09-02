#ifndef DOF_H_
#define DOF_H_

#include <rs/Macros>
#include <rsScene/Dof>
#include <rsSim/Sim>
#include <rsSim/Dof>

#include "robosim.h"
#include "robot.h"

// individual
class LIBRSEXPORT CDof : virtual public rsScene::Dof, virtual public rsSim::Dof, virtual public roboSim::Robot {
	public:
		CDof(int, char* = "", bool = true);
		virtual ~CDof(void);
};

// group
class LIBRSEXPORT CDofGroup : public roboSim::RobotGroup<CDof> {
	public:
		CDofGroup(void) : RobotGroup<CDof>() { };
		virtual ~CDofGroup(void) { };
};

// robosim
extern RoboSim *g_sim;

#endif // DOF_H_

