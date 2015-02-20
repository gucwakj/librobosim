#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif // _WIN32

#include "robosim.hpp"

// global robot simulation object
RoboSim *g_sim = NULL;

RoboSim::RoboSim(char *name, int pause) : rsScene::Scene(), rsSim::Sim(pause), rsXML::Store(name), rsCallback::Callback() {
	Scene::setGrid(Store::getUnits(), Store::getGrid());
	Sim::setPause(Store::getPause());
	Scene::start(Store::getPause());
	Callback::setUnits(Store::getUnits());

	// draw ground objects
	for (int i = 0; i < Store::getNumGrounds(); i++) {
		// get xml ground object
		rsXML::Ground *ground = Store::getGround(i);
		// build ground object
		rsSim::Ground *simGround;
		switch (ground->getType()) {
			case rs::BOX:
				simGround = Sim::addGround(ground->getPosition(), ground->getQuaternion(), ground->getDimensions(), ground->getMass());
				break;
			case rs::CYLINDER:
				simGround = Sim::addGround(ground->getPosition(), ground->getQuaternion(), ground->getDimensions(), ground->getMass(), ground->getAxis());
				break;
			case rs::SPHERE:
				simGround = Sim::addGround(ground->getPosition(), ground->getDimensions(), ground->getMass());
				break;
		}
		// draw ground object
		rsScene::Ground *sceneGround = Scene::drawGround(ground->getType(), ground->getPosition(), ground->getColor(), ground->getDimensions(), ground->getQuaternion());
		Callback::attachCallback(sceneGround, simGround);
	}

	// draw marker objects
	for (int i = 0; i < Store::getNumMarkers(); i++) {
		rsXML::Marker *marker = Store::getMarker(i);
		Scene::drawMarker(marker->getType(), marker->getStart(), marker->getEnd(), marker->getColor(), marker->getSize(), marker->getLabel());
	}
}

RoboSim::~RoboSim(void) {
	std::cerr << "deleting RoboSim" << std::endl;
}

int RoboSim::addRobot(rsSim::ModularRobot *robot) {
	// find new robot of this type
	int ff;
	robot->getFormFactor(ff);
	rsXML::Robot *xmlbot = Store::getNextRobot(ff);

	// build simulation robot
	if (xmlbot->getBaseConnector()) {
		rsXML::Conn *conn = xmlbot->getBaseConnector();
		Sim::addRobot(robot, xmlbot->getID(), Sim::getRobot(conn->getRobot()), xmlbot->getJoints(), conn->getFace1(), conn->getFace2(), conn->getType(), conn->getSide(), xmlbot->getGround(), 0);
	}
	else {
		Sim::addRobot(robot, xmlbot->getID(), xmlbot->getPosition(), xmlbot->getQuaternion(), xmlbot->getJoints(), xmlbot->getGround(), 0);
	}

	// 'connect' xml version to prevent finding it again
	xmlbot->setConnect(1);

	// build connectors
	rsXML::ConnectorList conn = xmlbot->getConnectorList();
	for (int i = 0; i < conn.size(); i++) {
		robot->addConnector(conn[i]->getType(), conn[i]->getFace1(), conn[i]->getSize(), conn[i]->getSide(), conn[i]->getConn());
	}

	// draw graphical robot
	rsScene::Robot *sceneRobot = Scene::drawRobot(robot, ff, xmlbot->getPosition(), xmlbot->getQuaternion(), xmlbot->getLED(), xmlbot->getTrace());

	// draw connectors
	for (int i = 0; i < conn.size(); i++) {
		Scene::drawConnector(robot, sceneRobot, conn[i]->getType(), conn[i]->getFace1(), conn[i]->getSize(), conn[i]->getSide(), conn[i]->getConn());
	}

	// attach callback
	Callback::attachCallback(sceneRobot, robot, robot->getBodyList(), robot->getConnectorList());

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
	return Sim::deleteRobot(loc);
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
