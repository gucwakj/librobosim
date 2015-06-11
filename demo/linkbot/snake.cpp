#include <iostream>
#include "linkbot.h"

#define NUM 4

int main(int argc, char *argv[]) {
	// make robots
	CLinkbotI *robot[NUM];
	for (int i = 0; i < NUM; i++) {
		robot[i] = new CLinkbotI();
	}

	// output positions
	for (int i = 0; i < NUM; i++) {
		std::cerr << robot[i]->getID() << "  pos: " << robot[i]->getCenter(0) << " " << robot[i]->getCenter(1) << " " << robot[i]->getCenter(2) << std::endl;
	}

	robot[0]->moveJointTo(rsLinkbot::JOINT1, 45);
	robot[0]->delaySeconds(1);

	// delte robots
	for (int i = 0; i < NUM; i++) {
		delete robot[i];
	}

	return 0;
}
