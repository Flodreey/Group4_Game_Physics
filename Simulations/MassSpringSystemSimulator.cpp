#include "MassSpringSystemSimulator.h"
//#include <cmath>

// Constructor
MassSpringSystemSimulator::MassSpringSystemSimulator() {
	h = 0.1f;
	sphereSize = 0.05f;
	number_massPoints = 0;
	number_springs = 0;
	m_fStiffness = 40;
	m_fMass = 10;
	m_fDamping = 0;
	m_iIntegrator = 0;

}

// UI Functions
const char* MassSpringSystemSimulator::getTestCasesStr() {
	return "Demo 1, Demo 2, Demo 3, Demo 4";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC) {
	this->DUC = DUC;
}

void MassSpringSystemSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {
	drawAllMassPoints();
	drawAllSprings();
}

void MassSpringSystemSimulator::setupDemo1() {
	m_fStiffness = 40;
	m_fMass = 10;

	int index1 = addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
	int index2 = addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);

	int springIndex1 = addSpring(index1, index2, 1);

	// print new velocity and new position for both points after one timestep
	Spring spring = getSpring(springIndex1);
	MassPoint point1 = getMassPoint(index1);
	MassPoint point2 = getMassPoint(index2);
	
	Vec3 oldPos1 = point1.position;
	Vec3 oldPos2 = point2.position;
	Vec3 oldVel1 = point1.velocity;
	Vec3 oldVel2 = point2.velocity;
	float mass1 = point1.mass;
	float mass2 = point2.mass;
	float stiffness = spring.stiffness;
	float initial_len = spring.initial_len;
	double timeStep = 0.1;
	
	std::cout << std::endl << "P1: " << oldPos1 << ", P2: " << oldPos2 << std::endl;
	std::cout << "EULER:-------------------------------------------------------" << std::endl;
	std::cout << "New positions: " << std::endl; 
	std::cout << "P1:" << calculateNextPosition(oldPos1, timeStep, oldVel1) << ", P2: " << calculateNextPosition(oldPos2, timeStep, oldVel2) << std::endl;
	std::cout << "New velocity: " << std::endl;
	std::cout << "V1: " << calculateNextVelocity(oldVel1, timeStep, oldPos1, oldPos2, stiffness, initial_len, mass1) << 
		", V2: " << calculateNextVelocity(oldVel2, timeStep, oldPos2, oldPos1, stiffness, initial_len, mass2) << std::endl;

	Vec3 posMidstep1 = calculateNextPosition(oldPos1, 0.5 * timeStep, oldVel1);
	Vec3 posMidstep2 = calculateNextPosition(oldPos2, 0.5 * timeStep, oldVel2);
	Vec3 velMidstep1 = calculateNextVelocity(oldVel1, 0.5 * timeStep, oldPos1, oldPos2, stiffness, initial_len, mass1);
	Vec3 velMidstep2 = calculateNextVelocity(oldVel2, 0.5 * timeStep, oldPos2, oldPos1, stiffness, initial_len, mass2);

	std::cout << "MIDPOINT:-----------------------------------------------------" << std::endl;
	std::cout << "New positions: " << std::endl;
	std::cout << "P1: " << calculateNextPosition(oldPos1, timeStep, velMidstep1) << ", P2: " << calculateNextPosition(oldPos2, timeStep, velMidstep2) << std::endl;
	std::cout << "New velocity: " << std::endl;
	std::cout << "V1: " << calculateNextVelocity(oldVel1, timeStep, posMidstep1, posMidstep2, stiffness, initial_len, mass1) << 
		", V2: " << calculateNextVelocity(oldVel2, timeStep, posMidstep2, posMidstep1, stiffness, initial_len, mass2) << std::endl;
}

void MassSpringSystemSimulator::setupDemo23() {
	m_fStiffness = 40;
	m_fMass = 10;

	int index1 = addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
	int index2 = addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);

	int springIndex1 = addSpring(index1, index2, 1);
}

void MassSpringSystemSimulator::drawAllMassPoints() {
	for (int i = 0; i < massPoint_list.size(); i++) {
		drawSphere(massPoint_list[i].position);
	}
}

void MassSpringSystemSimulator::drawAllSprings() {
	Vec3 color = Colors::Brown;
	DUC->beginLine();
	for (int i = 0; i < spring_list.size(); i++) {
		Vec3 massPoint1 = getPositionOfMassPoint(spring_list[i].massPointIndex1);
		Vec3 massPoint2 = getPositionOfMassPoint(spring_list[i].massPointIndex2);
		DUC->drawLine(massPoint1, color, massPoint2, color);
	}
	DUC->endLine();
}

void MassSpringSystemSimulator::drawSphere(Vec3 pos) {
	DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(0.97, 0.86, 1));
	DUC->drawSphere(pos, Vec3(sphereSize, sphereSize, sphereSize));
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase) {
	deleteAllPointsAndSprings();

	m_iTestCase = testCase;
	switch (m_iTestCase)
	{
	case 0:
		cout << "Demo 1\n";
		setupDemo1();
		break;
	case 1:
		cout << "Demo 2\n";
		setupDemo23();
		break;
	case 2:
		cout << "Demo 3\n";
		setupDemo23();
		break;
	case 3:
		cout << "Demo 4\n";
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed) {

}

void MassSpringSystemSimulator::simulateTimestep(float timeStep) {
	switch (m_iTestCase) {
	case 0:
		break;
	case 1:
		for (int i = 0; i < getNumberOfSprings(); i++) {
			doEulerStep(spring_list[i], timeStep);
		}
		break;
	case 2:
		for (int i = 0; i < getNumberOfSprings(); i++) {
			doMidpointStep(spring_list[i], timeStep);
		}
		break;
	case 3:
		break;
	}
}

void MassSpringSystemSimulator::doEulerStep(Spring s, float timeStep) {
	MassPoint p1 = getMassPoint(s.massPointIndex1);
	MassPoint p2 = getMassPoint(s.massPointIndex2);

	Vec3 newPos1 = calculateNextPosition(p1.position, timeStep, p1.velocity);
	Vec3 newPos2 = calculateNextPosition(p2.position, timeStep, p2.velocity);
	Vec3 newVel1 = calculateNextVelocity(p1.velocity, timeStep, p1.position, p2.position, s.stiffness, s.initial_len, p1.mass);
	Vec3 newVel2 = calculateNextVelocity(p2.velocity, timeStep, p2.position, p1.position, s.stiffness, s.initial_len, p2.mass);

	massPoint_list[s.massPointIndex1].position = newPos1;
	massPoint_list[s.massPointIndex2].position = newPos2;
	massPoint_list[s.massPointIndex1].velocity = newVel1;
	massPoint_list[s.massPointIndex2].velocity = newVel2;
}

void MassSpringSystemSimulator::doMidpointStep(Spring s, float timeStep) {
	MassPoint p1 = getMassPoint(s.massPointIndex1);
	MassPoint p2 = getMassPoint(s.massPointIndex2);

	Vec3 posMidstep1 = calculateNextPosition(p1.position, 0.5 * timeStep, p1.velocity);
	Vec3 posMidstep2 = calculateNextPosition(p2.position, 0.5 * timeStep, p2.position);
	Vec3 velMidstep1 = calculateNextVelocity(p1.velocity, 0.5 * timeStep, p1.position, p2.position, s.stiffness, s.initial_len, p1.mass);
	Vec3 velMidstep2 = calculateNextVelocity(p2.velocity, 0.5 * timeStep, p2.position, p1.position, s.stiffness, s.initial_len, p2.mass);
	Vec3 newPos1 = calculateNextPosition(p1.position, timeStep, velMidstep1);
	Vec3 newPos2 = calculateNextPosition(p2.position, timeStep, velMidstep2);
	Vec3 newVel1 = calculateNextVelocity(p1.velocity, timeStep, posMidstep1, posMidstep2, s.stiffness, s.initial_len, p1.mass);
	Vec3 newVel2 = calculateNextVelocity(p2.velocity, timeStep, posMidstep2, posMidstep1, s.stiffness, s.initial_len, p2.mass);

	massPoint_list[s.massPointIndex1].position = newPos1;
	massPoint_list[s.massPointIndex2].position = newPos2;
	massPoint_list[s.massPointIndex1].velocity = newVel1;
	massPoint_list[s.massPointIndex2].velocity = newVel2;
}

Vec3 MassSpringSystemSimulator::calculateNextPosition(Vec3 oldPos, float timeStep, Vec3 oldVel) {
	return oldPos + timeStep * oldVel;
}

Vec3 MassSpringSystemSimulator::calculateNextVelocity(Vec3 oldVel, float timeStep, Vec3 oldPos, Vec3 otherOldPos, float stiffness, float initial_len, float mass) {
	// calculate distance l between the two points
	Vec3 distanceVec = oldPos - otherOldPos;
	float l = sqrt(pow(distanceVec[0], 2) + pow(distanceVec[1], 2) + pow(distanceVec[2], 2));

	// calculate the spring force with hookes law
	Vec3 normalized = distanceVec / l;
	Vec3 forceVec = -stiffness * (l - initial_len) * normalized;

	// calculate the accelaration with the force and the mass
	Vec3 accVec = forceVec / mass;

	// calculate the actual next velocity
	return oldVel + timeStep * accVec;
}

void MassSpringSystemSimulator::onClick(int x, int y) {

}

void MassSpringSystemSimulator::onMouse(int x, int y) {

}

// Specific Functions

void MassSpringSystemSimulator::setMass(float mass) {
	m_fMass = mass;
}

void MassSpringSystemSimulator::setStiffness(float stiffness) {
	m_fStiffness = stiffness;
}

void MassSpringSystemSimulator::setDampingFactor(float damping) {
	m_fDamping = damping;
}

// returns index of mass point
int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 velocity, bool isFixed) {
	int index = getNumberOfMassPoints();

	MassPoint massPoint;
	massPoint.index = index;
	massPoint.mass = m_fMass;
	massPoint.position = position;
	massPoint.velocity = velocity;
	massPoint.isFixed = isFixed;

	//std::cout << "added a new point at " << position.toString() << " with ID " << index << std::endl;

	massPoint_list.push_back(massPoint);
	number_massPoints++;
	return index;
}

int MassSpringSystemSimulator::addSpring(int masspointIndex1, int masspointIndex2, float initialLength) {
	int index = getNumberOfSprings();

	Vec3 masspoint1 = getPositionOfMassPoint(masspointIndex1);
	Vec3 masspoint2 = getPositionOfMassPoint(masspointIndex2);

	Spring spring;
	spring.index = index;
	spring.stiffness = m_fStiffness;
	spring.initial_len = initialLength;
	spring.current_len = getLengthOfVec3(masspoint1 - masspoint2);
	spring.massPointIndex1 = masspointIndex1;
	spring.massPointIndex2 = masspointIndex2;

	//std::cout << "Added a new spring between " << spring.massPoint1.toString() << " and " << spring.massPoint2.toString() << std::endl;

	spring_list.push_back(spring);
	number_springs++;

	return index;
}

float MassSpringSystemSimulator::getLengthOfVec3(Vec3 vec) {
	return sqrt(pow(vec[0], 2) + pow(vec[1], 2) + pow(vec[2], 2));
}

void MassSpringSystemSimulator::deleteAllPointsAndSprings() {
	massPoint_list.clear();
	spring_list.clear();
	number_massPoints = 0;
	number_springs = 0;
}

int MassSpringSystemSimulator::getNumberOfMassPoints() {
	return number_massPoints;
}

int MassSpringSystemSimulator::getNumberOfSprings() {
	return number_springs;
}

Spring MassSpringSystemSimulator::getSpring(int index, bool& outSuccess) {
	for (int i = 0; i < number_springs; i++) {
		if (spring_list[i].index == index) {
			outSuccess = true;
			return spring_list[i];
		}
	}
	outSuccess = false;
	return spring_list[0];
}

Spring MassSpringSystemSimulator::getSpring(int index) {
	bool pointIsCreated;
	Spring s = getSpring(index, pointIsCreated);
	if (!pointIsCreated) {
		std::cout << "You tried to access a spring with index " << index << ", but it is not existent" << std::endl;
		Spring dummy;
		return dummy;
	}
	return s;
}

MassPoint MassSpringSystemSimulator::getMassPoint(int index, bool& outSuccess) {
	for (int i = 0; i < number_massPoints; i++) {
		if (massPoint_list[i].index == index) {
			outSuccess = true;
			return massPoint_list[i];
		}
	}
	outSuccess = false;
	return massPoint_list[0];
}

MassPoint MassSpringSystemSimulator::getMassPoint(int index) {
	bool pointIsCreated;
	MassPoint p = getMassPoint(index, pointIsCreated);
	if (!pointIsCreated) {
		std::cout << "You tried to access a masspoint with index " << index << ", but it is not existent" << std::endl;
		MassPoint dummy;
		return dummy;
	}
	return p;
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index) {
	MassPoint p = getMassPoint(index);
	return p.position;
}
Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index) {
	MassPoint p = getMassPoint(index);
	return p.velocity;
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force) {

}