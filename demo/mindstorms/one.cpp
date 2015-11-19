#include <iostream>
#include "mindstorms.h"

int main(int argc, char *argv[]) {
	CMindstorms robot("mindstorms_one.xml");
	robot.delaySeconds(1);

	return 0;
}
