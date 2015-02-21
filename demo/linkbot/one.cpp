#include <iostream>
#include "clinkbot.hpp"

int main(int argc, char *argv[]) {
	CLinkbotI robot;
	robot.connect();

	robot.addForce(rsLinkbot::BODY, 10, 0, 0);
	//robot.moveJointTo(rsLinkbot::JOINT3, 360);
	robot.delaySeconds(1);
	return 0;
}
