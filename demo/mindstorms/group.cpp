#include <iostream>
#include "mindstorms.h"

int main(int argc, char *argv[]) {
	CMindstorms robot1, robot2;
	CMindstormsGroup group;

	group.addRobot(robot1);
	group.addRobot(robot2);

	group.driveAngle(360);
	group.driveAngle(-360);

	return 0;
}
