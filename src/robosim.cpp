#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif // _WIN32

#include "robosim.h"

// global robot simulation object
RoboSim *g_sim = NULL;

RoboSim::RoboSim(char *name, int pause) : rsScene::Scene(), rsSim::Sim(pause, true), rsXML::Reader(name), rsCallback::Callback() {
	Sim::setPause(Reader::getPause());
	Scene::start(Reader::getPause());
	Callback::setUnits(Reader::getUnits());

	// draw ground objects
	for (int i = 0; i < Reader::getNumGrounds(); i++) {
		// get xml ground object
		rsXML::Ground *ground = Reader::getGround(i);
		// build ground object
		rsSim::Ground *simGround;
		switch (ground->getForm()) {
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
		rsScene::Ground *sceneGround = Scene::drawGround(0, ground->getForm(), ground->getPosition(), ground->getColor(), ground->getDimensions(), ground->getQuaternion());
		Callback::attachCallback(sceneGround, simGround);
	}

	// draw marker objects
	for (int i = 0; i < Reader::getNumMarkers(); i++) {
		rsXML::Marker *marker = Reader::getMarker(i);
		Scene::drawMarker(0, marker->getForm(), marker->getStart(), marker->getEnd(), marker->getColor(), marker->getSize(), marker->getLabel());
	}

	// set grid
	Scene::setGrid(Reader::getGrid());
	Scene::setUnits(Reader::getUnits());
}

int RoboSim::addRobot(rsSim::Robot *robot) {
	// find new robot of this type
	rsXML::Robot *xmlbot = Reader::getNextRobot(robot->getForm());
	robot->setForm(xmlbot->getForm());

	// set robot name
	robot->setName(xmlbot->getName());

	// build simulation robot
	Sim::addRobot(robot, xmlbot->getID(), xmlbot->getPosition(), xmlbot->getQuaternion(), xmlbot->getJoints(), xmlbot->getGround());

	// 'connect' xml version to prevent finding it again
	xmlbot->setConnect(1);

	// draw graphical robot
	rsScene::Robot *sceneRobot = Scene::drawRobot(robot, xmlbot->getPosition(), xmlbot->getQuaternion(), xmlbot->getJoints(), xmlbot->getLED(), xmlbot->getTrace());

	// attach callback
	Callback::attachCallback(sceneRobot, robot, robot->getBodyList());

	// success
	return 0;
}

int RoboSim::addRobot(rsSim::ModularRobot *robot) {
	// find new robot of this type
	rsXML::Robot *xmlbot = Reader::getNextRobot(robot->getForm());

	// set robot name
	robot->setName(xmlbot->getName());

	// build simulation robot
	if (xmlbot->getBaseConnector()) {
		rsXML::Conn *conn = xmlbot->getBaseConnector();
		Sim::addRobot(robot, xmlbot->getID(), Sim::getRobot(conn->getRobot()), xmlbot->getJoints(), conn->getFace1(), conn->getFace2(), conn->getType(), conn->getSide(), xmlbot->getOrientation(), xmlbot->getGround());
	}
	else {
		Sim::addRobot(robot, xmlbot->getID(), xmlbot->getPosition(), xmlbot->getQuaternion(), xmlbot->getJoints(), xmlbot->getGround());
	}

	// 'connect' xml version to prevent finding it again
	xmlbot->setConnect(1);

	// build connectors
	rsXML::ConnectorList conn = xmlbot->getConnectorList();
	for (unsigned int i = 0; i < conn.size(); i++) {
		robot->addConnector(conn[i]->getType(), conn[i]->getFace1(), conn[i]->getOrientation(), conn[i]->getSize(), conn[i]->getSide(), conn[i]->getConn());
	}

	// draw graphical robot
	rsScene::Robot *sceneRobot = Scene::drawRobot(robot, xmlbot->getPosition(), xmlbot->getQuaternion(), xmlbot->getJoints(), xmlbot->getLED(), xmlbot->getTrace());

	// draw connectors
	for (unsigned int i = 0; i < conn.size(); i++) {
		Scene::drawConnector(robot, sceneRobot, conn[i]->getType(), conn[i]->getFace1(), conn[i]->getOrientation(), conn[i]->getSize(), conn[i]->getSide(), conn[i]->getConn());
	}

	// attach callback
	Callback::attachCallback(sceneRobot, robot, robot->getBodyList(), robot->getConnectorList());

	// success
	return 0;
}

int RoboSim::deleteRobot(int id) {
	// pause simulation to view results only on first delete
	static int paused = 0;
	MUTEX_LOCK(&(_pause_mutex));
	MUTEX_LOCK(&(_running_mutex));
	if (!paused++ && _running) {
		Sim::_pause = 1;
		MUTEX_UNLOCK(&(_pause_mutex));
		MUTEX_UNLOCK(&(_running_mutex));

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

		// relock for new loop
		MUTEX_LOCK(&(_pause_mutex));
		MUTEX_LOCK(&(_running_mutex));
	}
	// unlock mutexes
	MUTEX_UNLOCK(&(_pause_mutex));
	MUTEX_UNLOCK(&(_running_mutex));

	// delete robot from modules
	Scene::deleteRobot(id);
	return Sim::deleteRobot(id);
}

void RoboSim::keyPressed(int key) {
	if (key == '1') { }
	else if (key == '2') { }
	else if (key == 'n') { }
	else if (key == 'r') {
		Sim::setCollisions(2);
	}
	else if (key == 't') { }
	else {
		Sim::setPause(2);
		Scene::setPauseText(Sim::getPause());
	}
}

void RoboSim::saveState(char *name) {
	//rsXML::Writer *writer = new rsXML::Writer("test.xml", Reader::getDoc());
}

