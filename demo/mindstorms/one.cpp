#include <iostream>
#include "mindstorms.h"

int main(int argc, char *argv[]) {
	CMindstorms robot;
	robot.connect();

	robot.driveDistance(4, 1.75);
	return 0;
}
