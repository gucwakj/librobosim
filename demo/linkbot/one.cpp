#include <iostream>
#include "linkbot.h"

int main(int argc, char *argv[]) {
	CLinkbotI robot1, robot2;

	g_sim->setGoalSinusoid(1, 1, 0, 0.03625);

	robot1.setSinusoidGain(0.988463);
	robot1.setSinusoidFrequency(0.991112);
	robot1.setSinusoidPhaseShift(0.009770);
	robot1.moveJointSinusoid(rsLinkbot::JOINT3);

	/*robot2.setSinusoidGain(0.343820);
	robot2.setSinusoidFrequency(0.977132);
	robot2.setSinusoidPhaseShift(0);
	robot2.moveJointSinusoid(rsLinkbot::JOINT3);*/

	double x, y, z;
	g_sim->getCoM(x, y, z);
	std::cerr << "com: " << x << " " << y << " " << z << std::endl;
	robot1.delaySeconds(10);
	std::cerr << "fitness: " << g_sim->getFitness() << std::endl;

	return 0;
}
