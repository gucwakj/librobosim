#include <iostream>
#include "dof.h"

using namespace rsDof;

int main(int argc, char *argv[]) {
	CDof robot1(Bodies::Face1);
	CDof robot2(Bodies::Face1);
	robot1.delaySeconds(1);

	return 0;
}
