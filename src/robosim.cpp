#include <osgText/Text>

#include "robosim.h"

// global robot simulation object
RoboSim *g_sim = NULL;

RoboSim::RoboSim(const char *name, bool pause) : rsScene::Scene(), rsSim::Sim(pause, true), rsXML::Reader(name), rsCallback::Callback() {
	// set pausing
	Sim::setPause(Reader::getPause());
	Scene::start(Reader::getPause());

	// set units
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
			case rs::HackySack:
				simObstacle = Sim::addObstacle(obstacle->getPosition(), obstacle->getQuaternion(), obstacle->getDimensions(), obstacle->getMass());
				break;
			case rs::CompetitionBorder:
				simObstacle = Sim::addCompetitionBorder(obstacle->getPosition(), obstacle->getQuaternion(), obstacle->getDimensions());
				break;
			case rs::Cylinder:
				simObstacle = Sim::addObstacle(obstacle->getPosition(), obstacle->getQuaternion(), obstacle->getDimensions(), obstacle->getMass(), obstacle->getAxis());
				break;
			case rs::Sphere:
				simObstacle = Sim::addObstacle(obstacle->getPosition(), obstacle->getDimensions(), obstacle->getMass());
				break;
			case rs::PullupBar:
				simObstacle = Sim::addPullupBar(obstacle->getPosition(), obstacle->getQuaternion(), obstacle->getDimensions());
				break;
		}
		// draw ground object
		rsScene::Obstacle *sceneObstacle = Scene::drawObstacle(0, obstacle->getForm(), obstacle->getPosition(), obstacle->getColor(), obstacle->getDimensions(), obstacle->getQuaternion());
		this->attachCallback(sceneObstacle, simObstacle);
	}

	// draw marker objects
	for (int i = 0; i < Reader::getNumMarkers(); i++) {
		rsXML::Marker *marker = Reader::getMarker(i);
		Scene::drawMarker(0, marker->getForm(), marker->getStart(), marker->getEnd(), marker->getPoint(), marker->getColor(), marker->getSize(), marker->getLabel());
	}

	// set grid
	Scene::setUnits(Reader::getUnits());
	Scene::setGrid(Reader::getGrid(), true);
}

int RoboSim::addRobot(rsSim::Robot *robot, rsScene::Robot *robot2, rsCallback::Robot *robot3) {
	// find new robot of this type
	rsXML::Robot *xmlbot = Reader::getNextRobot(robot->getForm());
	robot->setForm(xmlbot->getForm());

	// set robot name
	robot->setName(xmlbot->getName());

	// build simulation robot
	Sim::addRobot(robot, xmlbot->getID(), xmlbot->getPosition(), xmlbot->getQuaternion(), xmlbot->getJoints(), xmlbot->getWheels(), xmlbot->getGround());
	robot->setWheelRadius(xmlbot->getWheelRadius());
	robot->setRGB(xmlbot->getLED());
	robot->setTrace(xmlbot->getTrace());

	// 'connect' xml version to prevent finding it again
	xmlbot->setConnect(1);

	// draw graphical robot
	rsScene::Group *sceneRobot = Scene::createRobot(robot2);
	robot2->draw(sceneRobot, xmlbot->getPosition(), xmlbot->getQuaternion(), xmlbot->getJoints(), xmlbot->getLED(), xmlbot->getTrace());
	rs::Vec wheel = xmlbot->getWheels();
	robot2->drawWheel(sceneRobot, wheel[0], 1);
	robot2->drawWheel(sceneRobot, wheel[1], 2);
	Scene::stageChild(sceneRobot);

	// create and attach callback
	robot3->setCallbackParams(robot, robot->getBodyList(), _units);
	Scene::setRobotCallback(sceneRobot, robot3);

	// success
	return 0;
}

int RoboSim::addRobot(rsSim::ModularRobot *robot, rsScene::ModularRobot *robot2, rsCallback::ModularRobot *robot3) {
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
	robot->setWheelRadius(xmlbot->getWheelRadius());
	robot->setRGB(xmlbot->getLED());
	robot->setTrace(xmlbot->getTrace());

	// 'connect' xml version to prevent finding it again
	xmlbot->setConnect(1);

	// build connectors
	rsXML::ConnectorList conn = xmlbot->getConnectorList();
	for (unsigned int i = 0; i < conn.size(); i++) {
		Sim::mutexLock(Sim::AddRobot);
		if (conn[i]->getRobot() == xmlbot->getID()) {
			robot->addConnector(conn[i]->getType(), conn[i]->getFace1(), conn[i]->getOrientation(), conn[i]->getSize(), conn[i]->getSide(), conn[i]->getConn(), conn[i]->getOrientation2());
		}
		else {
			rsXML::Conn *base = xmlbot->getBaseConnector();
			if (base) {
				robot->fixConnectorToBody(	conn[i]->getFace2(),
											dynamic_cast<rsSim::ModularRobot*>(Sim::getRobot(base->getRobot()))->getConnectorBodyID(conn[i]->getFace1()),
											conn[i]->getType());
			}
			else
				_delay.push_back( std::make_tuple(conn[i], xmlbot->getID()));
		}
		Sim::mutexUnlock(Sim::AddRobot);
	}
	robot->calculateTrackwidth();

	// delayed fixing of looped connectors to bodies
	for (unsigned int i = 0; i < _delay.size(); i++) {
		rsXML::Conn *conn = std::get<0>(_delay[i]);
		int id = std::get<1>(_delay[i]);
		if (conn->getRobot() == xmlbot->getID()) {
			rsSim::ModularRobot *mrp = dynamic_cast<rsSim::ModularRobot*>(Sim::getRobot(id));
			mrp->fixBodyToConnector(dynamic_cast<rsSim::ModularRobot*>(Sim::getRobot(conn->getRobot()))->getConnectorBodyID(conn->getFace1()), conn->getFace2());
		}
	}

	// draw graphical robot
	rsScene::Group *sceneRobot = Scene::createRobot(robot2);
	robot2->draw(sceneRobot, xmlbot->getPosition(), xmlbot->getQuaternion(), xmlbot->getJoints(), xmlbot->getLED(), xmlbot->getTrace());

	// draw connectors
	for (unsigned int i = 0; i < conn.size(); i++) {
		if (conn[i]->getRobot() == xmlbot->getID()) {
			robot2->drawConnector(sceneRobot, conn[i]->getType(), conn[i]->getFace1(), conn[i]->getOrientation(), conn[i]->getSize(), conn[i]->getSide(), conn[i]->getConn());
		}
	}

	// add to scene
	Scene::stageChild(sceneRobot);

	// create and attach callback
	robot3->setCallbackParams(robot, robot->getBodyList(), robot->getConnectorList(), _units);
	Scene::setRobotCallback(sceneRobot, robot3);

	// success
	return 0;
}

int RoboSim::deleteRobot(int id) {
	// pause simulation to view results only on first delete
	static int paused = 0;
	if (!paused++ && Sim::getRunning()) {
		// pause simulation
		Sim::setPause(1);

		// get HUD and set message
		Scene::setHUD(true);
		Scene::getHUDText()->setText("Paused: Press any key to end");

		// sleep until pausing halted by user
		while (Sim::getPause()) {
#ifdef RS_WIN32
			Sleep(1);
#else
			usleep(1000);
#endif
		}
	}

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
		Sim::setPause(2);
		Scene::setPauseText(Sim::getPause());
	}
}

void RoboSim::saveState(char *name) {
	//rsXML::Writer *writer = new rsXML::Writer("test.xml", Reader::getDoc());
}

void RoboSim::updateClock(void) {
	// get time
	float time = Sim::getClock();

	// create string
	char buf[10];
	std::sprintf(buf, "%.2f", time);
	std::string s(buf);

	// set HUD
	Scene::getClockText()->setText(s);
}

