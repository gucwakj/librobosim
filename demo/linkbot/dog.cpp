#include <iostream>
#include "linkbot.h"

#define NUM 20

int main(int argc, char *argv[]) {
	CLinkbotI *robot[NUM];
	for (int i = 0; i < NUM; i++) {
		robot[i] = new CLinkbotI();
	}

	// output positions
	for (int i = 0; i < NUM; i++) {
		std::cerr << robot[i]->getID() << "  pos: " << robot[i]->getCenter(0) << " " << robot[i]->getCenter(1) << " " << robot[i]->getCenter(2) << std::endl;
	}

	// output feet locations
	rs::Pos p1 = robot[7]->getRobotFacePosition(rsLinkbot::FACE2, robot[7]->getPosition(), robot[7]->getQuaternion());
	std::cerr << "foot: " << p1[0] << " " << p1[1] << " " << p1[2] << std::endl;
	p1 = robot[11]->getRobotFacePosition(rsLinkbot::FACE2, robot[11]->getPosition(), robot[11]->getQuaternion());
	std::cerr << "foot: " << p1[0] << " " << p1[1] << " " << p1[2] << std::endl;
	p1 = robot[15]->getRobotFacePosition(rsLinkbot::FACE2, robot[15]->getPosition(), robot[15]->getQuaternion());
	std::cerr << "foot: " << p1[0] << " " << p1[1] << " " << p1[2] << std::endl;
	p1 = robot[19]->getRobotFacePosition(rsLinkbot::FACE2, robot[19]->getPosition(), robot[19]->getQuaternion());
	std::cerr << "foot: " << p1[0] << " " << p1[1] << " " << p1[2] << std::endl;

	robot[1]->moveTo(0, 0, 45);

	for (int i = 0; i < NUM; i++) {
		delete robot[i];
	}

	return 0;
}
