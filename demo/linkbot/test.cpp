#include <iostream>
#include "clinkbot.hpp"

int main(int argc, char *argv[]) {
	CLinkbotI robot;
	robot.connect();

	robot.driveDistance(5, 1.75);
	return 0;
}
