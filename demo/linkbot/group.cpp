#include <iostream>
#include "linkbot.h"

int main(int argc, char *argv[]) {
	CLinkbotI robot1, robot2;
	CLinkbotIGroup group;

	group.addRobot(robot1);
	group.addRobot(robot2);

	group.driveForward(360);
	group.driveBackward(360);

	return 0;
}
