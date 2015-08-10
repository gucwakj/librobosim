#include <iostream>
#include "dof.h"

using namespace rsDof;

int main(int argc, char *argv[]) {
	CDof robot1(Bodies::Face1);
	CDof robot2(Bodies::Face3);
	CDofGroup group;

	group.addRobot(robot1);
	group.addRobot(robot2);

	group.moveJointTo(Bodies::Joint, 360);
	group.moveJointTo(Bodies::Joint, -360);

	return 0;
}
