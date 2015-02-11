#include <iostream>
#include "clinkbot.hpp"

int main(int argc, char *argv[]) {
	CLinkbotI robot;
	robot.connect();

	//robot.driveDistance(5, 1.75);
	//robot.moveJointTo(rs::JOINT1, 360);
	robot.moveJointTo(rs::JOINT3, 360);
	return 0;
}
