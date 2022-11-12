#include "MassSpringSystemSimulator.h"
//#include <cmath>

// Constructor
MassSpringSystemSimulator::MassSpringSystemSimulator() {
	h = 0.1f;
	sphereSize = 0.05f;
	number_massPoints = 2;
	number_springs = 1;
	m_fStiffness = 40;
	m_fMass = 10;
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
	switch (m_iTestCase)
	{
	case 0: drawDemo1(); break;
	case 1: drawSphere(Vec3(0.5, 0.5, 0.5)); break;
	case 2: drawSphere(Vec3(-0.5, -0.5, -0.5)); break;
	case 3: drawSphere(Vec3(-0.2, -0.2, -0.2)); break;
	}
}

void MassSpringSystemSimulator::drawDemo1() {
	drawAllMassPoints();
	drawAllSprings();
}

void MassSpringSystemSimulator::setupDemo1() {
	int index1 = addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
	int index2 = addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);

	int springIndex1 = addSpring(index1, index2, 1);

	// print new velocity and new position for both points
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
	
	std::cout << "P1: " << oldPos1 << ", P2: " << oldPos2 << std::endl;
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
	std::cout << "V1: " << setprecision(3) << calculateNextVelocity(oldVel1, timeStep, posMidstep1, posMidstep2, stiffness, initial_len, mass1) << ", V2: " << calculateNextVelocity(oldVel2, timeStep, posMidstep2, posMidstep1, stiffness, initial_len, mass2) << std::endl;
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
		break;
	case 2:
		cout << "Demo 3\n";
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
		break;
	case 2:
		break;
	case 3:
		break;
	}
}

Vec3 MassSpringSystemSimulator::calculateNextPosition(Vec3 oldPos, float timeStep, Vec3 oldVel) {
	float rounded_timeStep = roundf(100 * timeStep) / 100;
	return oldPos + rounded_timeStep * oldVel;
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