#include "robosim.hpp"

// global robot simulation object
RoboSim *g_sim = NULL;

RoboSim::RoboSim(char *name, int pause) : rsSim::Sim(pause), rsScene::Scene(), rsXML::Store(name) {
	Sim::setPause(Store::getPause());
	Scene::start(Store::getPause());
	for (int i = 1; i < Store::getNumGrounds(); i++) {
		//rsScene::drawGround();
		i = i;
	}
	for (int i = 0; i < Store::getNumMarkers(); i++) {
		//rsScene::drawMaker(]);
		i = i;
	}
}

RoboSim::~RoboSim(void) {
	std::cerr << "deleting RoboSim" << std::endl;
}

int RoboSim::addRobot(rsSim::ModularRobot *robot) {
	int ff;
	robot->getFormFactor(ff);
	rsXML::Robot *xmlbot = Store::getNextRobot(ff);

	Sim::addRobot3(robot, xmlbot->getID(), xmlbot->getPosition(), xmlbot->getQuaternion(), xmlbot->getJoints(), xmlbot->getGround(), 0);

	xmlbot->setConnect(1);

	Sim::_robot.back()->node = Scene::drawRobot(robot, ff, xmlbot->getPosition(), xmlbot->getQuaternion(), xmlbot->getTrace());

	// success
	return 0;
}


int RoboSim::deleteRobot(int loc) {
std::cout << "delete Robot" << std::endl;
	// pause simulation to view results only on first delete
	MUTEX_LOCK(&(_pause_mutex));
	static int paused = 0;
	if (!paused++ && _running) {
		Sim::_pause = 1;
		MUTEX_UNLOCK(&(_pause_mutex));

		// get HUD and set message
		Scene::getHUDText()->setText("Paused: Press any key to end");

		// sleep until pausing halted by user
		MUTEX_LOCK(&(_pause_mutex));
		while (Sim::_pause) {
			MUTEX_UNLOCK(&(_pause_mutex));
#ifdef _WIN32
			Sleep(1);
#else
			usleep(1000);
#endif
			MUTEX_LOCK(&(_pause_mutex));
		}
		MUTEX_UNLOCK(&(_pause_mutex));
	}
	MUTEX_UNLOCK(&(_pause_mutex));
	//Scene::stageForDelete(Sim::_robot[loc]->node);
	//return Sim::deleteRobot(loc);
	return 1;
}

void RoboSim::keyPressed(int key) {
	if (key == 'r') {
		Sim::setCollisions(2);
	}
	else {
		Sim::setPause(2);
		Scene::setPauseText(Sim::getPause());
	}
}
