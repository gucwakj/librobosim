#include <iostream>
#include "minidof.h"

using namespace rsMiniDof;

int main(int argc, char *argv[]) {
	CMiniDof robot(Bodies::Face1, "minidof_one.xml");
	robot.delaySeconds(1);

	return 0;
}
