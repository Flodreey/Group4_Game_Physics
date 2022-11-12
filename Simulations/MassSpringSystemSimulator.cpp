#include "MassSpringSystemSimulator.h"
//#include <cmath>

// Constructor
MassSpringSystemSimulator::MassSpringSystemSimulator() {
	h = 0.1f;
	sphereSize = 0.05f;
	number_massPoints = 2;
	number_springs = 1;
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
	//int index3 = addMassPoint(Vec3(0.2, -0.3, 1), Vec3(0, 0, 0), false);
	//int index4 = addMassPoint(Vec3(0.2, 0.1, 0), Vec3(0, 0, 0), false);
	//int index5 = addMassPoint(Vec3(0, 0.7, 0.3), Vec3(0, 0, 0), false);
	//int index6 = addMassPoint(Vec3(0, -2, 0), Vec3(0, 0, 0), false);

	addSpring(index1, index2, 2);
	//addSpring(index1, index3, 2);
	//addSpring(index4, index6, 2);
	//addSpring(index5, index4, 2);
	//addSpring(index2, index3, 2);
}

void MassSpringSystemSimulator::drawAllMassPoints() {
	for (MassPoint p : massPoint_list) {
		drawSphere(p.position);
	}
}

void MassSpringSystemSimulator::drawAllSprings() {
	Vec3 color = Colors::AliceBlue;
	DUC->beginLine();
	for (Spring s : spring_list) {
		DUC->drawLine(s.massPoint1, color, s.massPoint2, color);
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

void MassSpringSystemSimulator::addSpring(int masspointIndex1, int masspointIndex2, float initialLength) {

	Vec3 masspoint1 = getPositionOfMassPoint(masspointIndex1);
	Vec3 masspoint2 = getPositionOfMassPoint(masspointIndex2);

	Spring spring;
	spring.stiffness = m_fStiffness;
	spring.initial_len = initialLength;
	spring.current_len = getLengthOfVec3(masspoint1 - masspoint2);
	spring.massPoint1 = masspoint1;
	spring.massPoint2 = masspoint2;

	//std::cout << "Added a new spring between " << spring.massPoint1.toString() << " and " << spring.massPoint2.toString() << std::endl;

	spring_list.push_back(spring);
	number_springs++;
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

MassPoint MassSpringSystemSimulator::getMassPoint(int index, bool& outSuccess) {
	for (int i = 0; i < massPoint_list.size(); i++) {
		if (massPoint_list[i].index == index) {
			outSuccess = true;
			return massPoint_list[i];
		}
	}
	outSuccess = false;
	return massPoint_list[0];
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index) {
	bool pointIsCreated;
	MassPoint p = getMassPoint(index, pointIsCreated);
	if (!pointIsCreated) {
		std::cout << "You tried to access the position of a masspoint with index " << index << ", but it is not existent" << std::endl;
		return Vec3();
	}
	return p.position;
}
Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index) {
	bool pointIsCreated;
	MassPoint p = getMassPoint(index, pointIsCreated);
	if (!pointIsCreated) {
		std::cout << "You tried to access the velocity of a masspoint with index " << index << ", but it is not existent" << std::endl;
		return Vec3();
	}
	return p.velocity;
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force) {

}