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
	double p1[3], q1[4];
	robot[7]->getRobotFaceOffset(rsLinkbot::FACE2, 0, robot[7]->getPosition(), robot[7]->getQuaternion(), p1, q1);
	std::cerr << "foot: " << p1[0] << " " << p1[1] << " " << p1[2] << std::endl;
	robot[11]->getRobotFaceOffset(rsLinkbot::FACE2, 0, robot[11]->getPosition(), robot[11]->getQuaternion(), p1, q1);
	std::cerr << "foot: " << p1[0] << " " << p1[1] << " " << p1[2] << std::endl;
	robot[15]->getRobotFaceOffset(rsLinkbot::FACE2, 0, robot[15]->getPosition(), robot[15]->getQuaternion(), p1, q1);
	std::cerr << "foot: " << p1[0] << " " << p1[1] << " " << p1[2] << std::endl;
	robot[19]->getRobotFaceOffset(rsLinkbot::FACE2, 0, robot[19]->getPosition(), robot[19]->getQuaternion(), p1, q1);
	std::cerr << "foot: " << p1[0] << " " << p1[1] << " " << p1[2] << std::endl;

	robot[0]->moveTo(45, 0, 45);

	return 0;
}
