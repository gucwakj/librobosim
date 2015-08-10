#ifndef DOF_H_
#define DOF_H_

#include <rs/Macros>
#include <rsSim/Sim>
#include <rsSim/Dof>

#include "robosim.h"
#include "robot.h"

// individual
class LIBRSEXPORT CDof : virtual public rsSim::Dof, virtual public Robot {
	public:
		CDof(int, char* = "", bool = true);
		virtual ~CDof(void);
};

// group
class LIBRSEXPORT CDofGroup : public RobotGroup<CDof> {
	public:
		CDofGroup(void) : RobotGroup<CDof>() { };
		virtual ~CDofGroup(void) { };
};

// simulation
extern RoboSim *g_sim;

#endif // DOF_H_

