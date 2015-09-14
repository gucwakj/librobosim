#include "robosim.h"

// global robot simulation object
RoboSim *g_sim = NULL;

RoboSim::RoboSim(const char *name, bool pause) : rsScene::Scene(), rsSim::Sim(pause, true), rsXML::Reader(name), rsCallback::Callback() {
	// set pausing
	Sim::pause(Reader::getPause());
	Scene::start(Reader::getPause());

	// set units
	Callback::setUnits(Reader::getUnits());
	_units = Reader::getUnits();

	// set background
	for (int i = 0; i < 7; i++) {
		Scene::setBackgroundImage(i, Reader::getBackgroundImage(i));
	}
	Scene::setLevel(Reader::getLevel());
	Reader::addBackgroundObjects();

	// draw ground objects
	for (int i = 0; i < Reader::getNumObstacles(); i++) {
		// get xml ground object
		rsXML::Obstacle *obstacle = Reader::getObstacle(i);
		// build ground object
		rsSim::Obstacle *simObstacle = NULL;
		switch (obstacle->getForm()) {
			case rs::Box:
			case rs::WoodBlock:
				simObstacle = Sim::addObstacle(obstacle->getPosition(), obstacle->getQuaternion(), obstacle->getDimensions(), obstacle->getMass());
				break;
			case rs::Cylinder:
				simObstacle = Sim::addObstacle(obstacle->getPosition(), obstacle->getQuaternion(), obstacle->getDimensions(), obstacle->getMass(), obstacle->getAxis());
				break;
			case rs::Sphere:
			case rs::HackySack:
				simObstacle = Sim::addObstacle(obstacle->getPosition(), obstacle->getDimensions(), obstacle->getMass());
				break;
		}
		// draw ground object
		rsScene::Obstacle *sceneObstacle = Scene::drawObstacle(0, obstacle->getForm(), obstacle->getPosition(), obstacle->getColor(), obstacle->getDimensions(), obstacle->getQuaternion());
		Callback::attachCallback(sceneObstacle, simObstacle);
	}

	// draw marker objects
	for (int i = 0; i < Reader::getNumMarkers(); i++) {
		rsXML::Marker *marker = Reader::getMarker(i);
		Scene::drawMarker(0, marker->getForm(), marker->getStart(), marker->getEnd(), marker->getColor(), marker->getSize(), marker->getLabel());
	}

	// set grid
	Scene::setUnits(Reader::getUnits());
	Scene::setGrid(Reader::getGrid(), true);
}

int RoboSim::addRobot(rsSim::Robot *robot, rsScene::Robot *robot2) {
	// find new robot of this type
	rsXML::Robot *xmlbot = Reader::getNextRobot(robot->getForm());
	robot->setForm(xmlbot->getForm());

	// set robot name
	robot->setName(xmlbot->getName());

	// build simulation robot
	Sim::addRobot(robot, xmlbot->getID(), xmlbot->getPosition(), xmlbot->getQuaternion(), xmlbot->getJoints(), xmlbot->getWheels(), xmlbot->getGround());
	robot->setWheelRadius(xmlbot->getRadius());
	robot->setLED(xmlbot->getLED());

	// 'connect' xml version to prevent finding it again
	xmlbot->setConnect(1);

	// draw graphical robot
	rsScene::Group *sceneRobot = Scene::createRobot(robot2);
	robot2->draw(sceneRobot, xmlbot->getPosition(), xmlbot->getQuaternion(), xmlbot->getJoints(), xmlbot->getLED(), xmlbot->getTrace());
	rs::Vec wheel = xmlbot->getWheels();
	robot2->drawWheel(sceneRobot, wheel[0], 1);
	robot2->drawWheel(sceneRobot, wheel[1], 2);
	Scene::stageChild(sceneRobot);

	// attach callback
	Callback::attachCallback(sceneRobot, robot, robot->getBodyList());

	// success
	return 0;
}

int RoboSim::addRobot(rsSim::ModularRobot *robot, rsScene::ModularRobot *robot2) {
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
		Sim::addRobot(robot, xmlbot->getID(), xmlbot->getPosition(), xmlbot->getQuaternion(), xmlbot->getJoints(), xmlbot->getWheels(), xmlbot->getGround());
	}
	robot->setWheelRadius(xmlbot->getRadius());
	robot->setLED(xmlbot->getLED());

	// 'connect' xml version to prevent finding it again
	xmlbot->setConnect(1);

	// build connectors
	rsXML::ConnectorList conn = xmlbot->getConnectorList();
	for (unsigned int i = 0; i < conn.size(); i++) {
		Sim::mutexLock(0);
		robot->addConnector(conn[i]->getType(), conn[i]->getFace1(), conn[i]->getOrientation(), conn[i]->getSize(), conn[i]->getSide(), conn[i]->getConn());
		Sim::mutexUnlock(0);
	}
	robot->calculateTrackwidth();

	// draw graphical robot
	rsScene::Group *sceneRobot = Scene::createRobot(robot2);
	robot2->draw(sceneRobot, xmlbot->getPosition(), xmlbot->getQuaternion(), xmlbot->getJoints(), xmlbot->getLED(), xmlbot->getTrace());

	// draw connectors
	for (unsigned int i = 0; i < conn.size(); i++) {
		robot2->drawConnector(sceneRobot, conn[i]->getType(), conn[i]->getFace1(), conn[i]->getOrientation(), conn[i]->getSize(), conn[i]->getSide(), conn[i]->getConn());
	}

	// add to scene
	Scene::stageChild(sceneRobot);

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
		Scene::setHUD(true);
		Scene::getHUDText()->setText("Paused: Press any key to end");

		// sleep until pausing halted by user
		MUTEX_LOCK(&(_pause_mutex));
		while (Sim::_pause) {
			MUTEX_UNLOCK(&(_pause_mutex));
#ifdef RS_WIN32
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

bool RoboSim::getUnits(void) {
	return _units;
}

void RoboSim::keyPressed(int key) {
	if (key == '1') { }
	else if (key == '2') { }
	else if (key == 'c') { }
	else if (key == 'n') { }
	else if (key == 'r') {
		Sim::setCollisions(2);
	}
	else if (key == 't') { }
	else {
		Sim::pause(2);
		Scene::setPauseText(Sim::getPause());
	}
}

void RoboSim::saveState(char *name) {
	//rsXML::Writer *writer = new rsXML::Writer("test.xml", Reader::getDoc());
}

