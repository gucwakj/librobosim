#include <iostream>
#include "linkbot.h"

int main(int argc, char *argv[]) {
	CLinkbotI robot1, robot2, robot3, robot4;

	// set up
	g_sim->setLevel(0);
	//g_sim->setGoalSinusoid(0.025, 60, 0, 0.138433);

	// values
	// unconstrained initial guesses of values: fast movement of leg
	//double A = 0.247455, B = 95.4295, C = 0.0821624, D = 0.07058, E = -1.49511, F = 2.3734, G = 0.211783, H = -38.8801, I = 0.369536, J = -0.635153, K = 2.30506, L = 1.90406;
	// slowed down timing to allow body to catch up to infinity
	//double A = 0.095953, B = -68.9277, C = 1.89401, D = 2.30534, E = -91.0751, F = 1.6924, G = -0.229773, H = 63.6986, I = 3.28721, J = 0.854495, K = 18.866, L = 1.22491;
	// more slowing down with a larger step
	//double A = 0.026325, B = -68.0825, C = 0.0907516, D = 3.05822, E = -4.24552, F = 4.4967, G = 0.118626, H = 0.170086, I = 0.364345, J = 0.348795, K = 37.5407, L = 1.36464;
	// slow with a higher step
	//double A = 0.003054, B = -99.2783, C = 2.64538, D = 0.71308, E = 104.748, F = 0.06737, G = 1.32765, H = 0.0923193, I = 3.12107, J = -1.63795, K = -22.4725, L = 2.43603;
	// slow with a shallow step
	double A = 0.195044, B = -64.2348, C = 1.79226, D = 0.0091, E = -96.0445, F = 0.558185, G = 0.00926, H = -88.1334, I = 3.18695, J = -2.10646, K = -27.8312, L = 1.25367;

	// move joints in sinusoids
	robot1.setSinusoidGain(A);
	robot1.setSinusoidFrequency(B);
	robot1.setSinusoidPhaseShift(C);
	robot1.moveJointSinusoid(rsLinkbot::JOINT3);

	robot2.setSinusoidGain(D);
	robot2.setSinusoidFrequency(E);
	robot2.setSinusoidPhaseShift(F);
	robot2.moveJointSinusoid(rsLinkbot::JOINT1);

	robot3.setSinusoidGain(G);
	robot3.setSinusoidFrequency(H);
	robot3.setSinusoidPhaseShift(I);
	robot3.moveJointSinusoid(rsLinkbot::JOINT3);

	robot4.setSinusoidGain(J);
	robot4.setSinusoidFrequency(K);
	robot4.setSinusoidPhaseShift(L);
	robot4.moveJointSinusoid(rsLinkbot::JOINT1);

	robot1.delaySeconds(20);

	return 0;
}
