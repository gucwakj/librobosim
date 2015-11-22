#include <iostream>
#include "minidof.h"

using namespace rsMiniDof;

int main(int argc, char *argv[]) {
	CMiniDof robot1(Bodies::Face1);
	CMiniDof robot2(Bodies::Face3);
	CMiniDofGroup group;

	group.addRobot(robot1);
	group.addRobot(robot2);

	group.moveJointTo(Bodies::Joint, 360);
	group.moveJointTo(Bodies::Joint, -360);

	return 0;
}
