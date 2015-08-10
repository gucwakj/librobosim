#include <iostream>
#include "dof.h"

using namespace rsDof;

int main(int argc, char *argv[]) {
	CDof robot(Bodies::Face1);
	robot.delaySeconds(1);

	return 0;
}
