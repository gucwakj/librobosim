#include <iostream>
#include "minidof.h"

using namespace rsMiniDof;

int main(int argc, char *argv[]) {
	CMiniDof robot1(Bodies::Face1);
	CMiniDof robot2(Bodies::Face1);
	robot1.delaySeconds(1);

	return 0;
}
