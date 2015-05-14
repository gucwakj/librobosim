#include <iostream>
#include "linkbot.h"

int main(int argc, char *argv[]) {
	CLinkbotI robot1, robot2;

	g_sim->setGoalSinusoid(0.01, 0.5, 0, 0.03625);

	robot1.setSinusoidGain(1.009396);
	robot1.setSinusoidFrequency(0.232753);
	robot1.setSinusoidPhaseShift(0);
	robot1.moveJointSinusoid(rsLinkbot::JOINT1);

	robot2.setSinusoidGain(0.952470);
	robot2.setSinusoidFrequency(0.721266);
	robot2.setSinusoidPhaseShift(0);
	robot2.moveJointSinusoid(rsLinkbot::JOINT3);

	robot1.delaySeconds(10);

	return 0;
}
