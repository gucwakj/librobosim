#include <iostream>
#include "clinkbot.hpp"

int main(int argc, char *argv[]) {
	CLinkbotI robot;
	robot.connect();

	robot.move(90, 0, 90);
	return 0;
}
