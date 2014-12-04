#include <iostream>
#include "clinkbot.hpp"

int main(int argc, char *argv[]) {
	CLinkbotI robot;
	robot.connect();

	usleep(1000000);
	return 0;
}
