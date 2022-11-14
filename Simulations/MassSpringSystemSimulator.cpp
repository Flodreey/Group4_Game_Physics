#include "MassSpringSystemSimulator.h"
#include<array>

// Constructor
MassSpringSystemSimulator::MassSpringSystemSimulator() {
	sphereSize = 0.05f;
	default_sphereColor = 0.6 * Vec3(0.97, 0.86, 1);
	number_massPoints = 0;
	number_springs = 0;
	m_fStiffness = 80;
	m_fMass = 1;
	m_fDamping = 1;
	m_iIntegrator = 0;
	gravity = 1;
	number_columns = 10;
	number_rows = 10;
	currentlyClicking = false;
	m_externalForce = Vec3();

	previousMass = -1;
	previousStiffness = -1;
	previousDamping = -1;
	previousGravity = -1;
	previousIntegrator = -1;
	previousNumberColumns = -1;
	previousNumberRows = -1;

	
}

// UI Functions
const char* MassSpringSystemSimulator::getTestCasesStr() {
	return "Demo 1, Demo 2, Demo 3, Demo 4";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC) {
	this->DUC = DUC;
	if (m_iTestCase == 3) {
		// Lets user choose between Euler-Integration and Midpoint-Integration, Euler: 0, Leap Frog:1, Midpoint: 2
		TwType TW_TYPE_TESTCASE = TwDefineEnumFromString("Integrator", "Euler, Leap Frog, Midpoint");
		TwAddVarRW(DUC->g_pTweakBar, "Integrator", TW_TYPE_TESTCASE, &m_iIntegrator, "");
		
		// Lets user manipulate the stiffness of springs, internal friction, gravity, mass of masspoints 
		//TwAddVarRW(DUC->g_pTweakBar, "Stiffness", TW_TYPE_FLOAT, &m_fStiffness, "min=1");
		//TwAddVarRW(DUC->g_pTweakBar, "Friction", TW_TYPE_FLOAT, &m_fDamping, "min=0");
		TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_FLOAT, &gravity, "min=0");
		//TwAddVarRW(DUC->g_pTweakBar, "Mass", TW_TYPE_FLOAT, &m_fMass, "min=1");

		// Lets user decide how big the cloth in the simulation should be
		TwAddVarRW(DUC->g_pTweakBar, "Colums", TW_TYPE_INT32, &number_columns, "min=3");
		TwAddVarRW(DUC->g_pTweakBar, "Rows", TW_TYPE_INT32, &number_rows, "min=3");
	}
}

void MassSpringSystemSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	clickedPos.x = clickedPos.y = 0;
	//m_trackmouse.x = m_trackmouse.y = 0;
	//m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {
	drawAllMassPoints();
	drawAllSprings();
}

void MassSpringSystemSimulator::setDefaultValues() {
	m_fStiffness = 80;
	m_fMass = 1;
	gravity = 1;
	m_fDamping = 0.5f;
}

void MassSpringSystemSimulator::setupDemo1() {
	m_fStiffness = 40;
	m_fMass = 10;
	gravity = 0;
	m_fDamping = 0;
	sphereSize = 0.05f;

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
	gravity = 0;
	m_fDamping = 0;
	sphereSize = 0.05f;

	if (m_iTestCase == 1)
		m_iIntegrator = EULER;
	else if (m_iTestCase == 2) 
		m_iIntegrator = MIDPOINT;

	int index1 = addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
	int index2 = addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);

	int springIndex1 = addSpring(index1, index2, 1);
}

void MassSpringSystemSimulator::setupDemo4() {
	m_fStiffness = 80;
	m_fMass = 1;
	//gravity = 1;
	m_fDamping = 0.5f;
	sphereSize = 0.02f;

	int columns = number_columns;
	int rows = number_rows;
	float distance = 0.2f;

	//array<array<int, columns>, rows> points{};
	vector<vector<int>> points;
	
	Vec3 startPos = Vec3(0, 0, 0) - Vec3(((columns-1) * distance) / 2.0, 0, ((rows-1) * distance) / 2.0);
	for (int i = 0; i < rows; i++) {
		vector<int> currentVector;
		for (int j = 0; j < columns; j++) {
			Vec3 pos = startPos + distance * Vec3(j, 0, i);
			bool isFixed = (i == 0 || i == rows - 1 || j == 0 || j == columns - 1) ? true : false;
			currentVector.push_back(addMassPoint(pos, Vec3(), isFixed));
		}
		points.push_back(currentVector);
	}

	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < columns; j++) {
			if (j == columns - 1 && i == rows - 1) {
				// do nothing
			}
			else if (i == rows - 1) {
				addSpring(points[i][j], points[i][j + 1], distance);
			}
			else if (j == 0) {
				addSpring(points[i][j], points[i][j + 1], distance);
				addSpring(points[i][j], points[i + 1][j], distance);
				addSpring(points[i][j], points[i + 1][j + 1], distance);
			}
			else if (j == columns - 1) {
				addSpring(points[i][j], points[i + 1][j - 1], distance);
				addSpring(points[i][j], points[i + 1][j], distance);
			}
			else {
				addSpring(points[i][j], points[i + 1][j - 1], distance);
				addSpring(points[i][j], points[i + 1][j], distance);
				addSpring(points[i][j], points[i + 1][j + 1], distance);
				addSpring(points[i][j], points[i][j + 1], distance);
			}
		}
	}
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
	drawSphere(pos, default_sphereColor);
}

void MassSpringSystemSimulator::drawSphere(Vec3 pos, Vec3 color) {
	//DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(0.97, 0.86, 1));
	DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, color);
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
		setupDemo4();
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
		// If Integration Method was changed, then display it
		if (previousIntegrator != m_iIntegrator) {
			if (m_iIntegrator == EULER)
				cout << "Integration Method: Euler" << endl;
			else if (m_iIntegrator == LEAPFROG)
				cout << "Integration Method: Leap Frog (CAUTION: This has not been implemented yet)" << endl;
			else if (m_iIntegrator == MIDPOINT)
				cout << "Integration Method: Midpoint" << endl;
			previousIntegrator = m_iIntegrator;
			deleteAllPointsAndSprings();
			setupDemo4();
		}

		// If any of these variables gets changed, load the setup again: 
		// m_fMass, m_fStiffness, m_fDamping, gravity, number_columns, number_rows
		if (m_fMass != previousMass || m_fStiffness != previousStiffness || m_fDamping != previousDamping ||
			gravity != previousGravity || number_columns != previousNumberColumns || number_rows != previousNumberRows) {
			deleteAllPointsAndSprings();
			setupDemo4();
			previousMass = m_fMass;
			previousStiffness = m_fStiffness;
			previousDamping = m_fDamping;
			previousGravity = gravity;
			previousNumberColumns = number_columns;
			previousNumberRows = number_rows;
		}

		// move the points in the middle (depending on number_colums and number_rows) according to the mouse


		for (int i = 0; i < getNumberOfSprings(); i++) {
			switch (m_iIntegrator) {
				case EULER: 
					doEulerStep(spring_list[i], timeStep);
					break;
				case LEAPFROG:
					break;
				case MIDPOINT:
					doMidpointStep(spring_list[i], timeStep);
					break;
				default:
					break;
			}
		}
		break;
	}
}

void MassSpringSystemSimulator::checkCollision(Vec3& newPos, Vec3& newVel) {
	if (newPos.y < -0.95 && m_iTestCase == 3) {
		newPos.y = -0.95;
		newVel *= -1;
	}
}

void MassSpringSystemSimulator::doEulerStep(Spring s, float timeStep) {
	MassPoint p1 = getMassPoint(s.massPointIndex1);
	MassPoint p2 = getMassPoint(s.massPointIndex2);

	Vec3 newPos1 = p1.position;
	Vec3 newVel1 = p1.velocity;
	Vec3 newPos2 = p2.position;
	Vec3 newVel2 = p2.velocity;

	if (!p1.isFixed) {
		newPos1 = calculateNextPosition(p1.position, timeStep, p1.velocity);
		newVel1 = calculateNextVelocity(p1.velocity, timeStep, p1.position, p2.position, s.stiffness, s.initial_len, p1.mass);
		checkCollision(newPos1, newVel1);
	}
	if (!p2.isFixed) {
		newPos2 = calculateNextPosition(p2.position, timeStep, p2.velocity);
		newVel2 = calculateNextVelocity(p2.velocity, timeStep, p2.position, p1.position, s.stiffness, s.initial_len, p2.mass);
		checkCollision(newPos2, newVel2);
	}

	massPoint_list[s.massPointIndex1].position = newPos1;
	massPoint_list[s.massPointIndex2].position = newPos2;
	massPoint_list[s.massPointIndex1].velocity = newVel1;
	massPoint_list[s.massPointIndex2].velocity = newVel2;
}

void MassSpringSystemSimulator::doMidpointStep(Spring s, float timeStep) {
	MassPoint p1 = getMassPoint(s.massPointIndex1);
	MassPoint p2 = getMassPoint(s.massPointIndex2);

	Vec3 posMidstep1 = calculateNextPosition(p1.position, 0.5 * timeStep, p1.velocity);
	Vec3 velMidstep1 = calculateNextVelocity(p1.velocity, 0.5 * timeStep, p1.position, p2.position, s.stiffness, s.initial_len, p1.mass);
	Vec3 posMidstep2 = calculateNextPosition(p2.position, 0.5 * timeStep, p2.position);
	Vec3 velMidstep2 = calculateNextVelocity(p2.velocity, 0.5 * timeStep, p2.position, p1.position, s.stiffness, s.initial_len, p2.mass);

	Vec3 newPos1 = p1.position;
	Vec3 newVel1 = p1.velocity;
	Vec3 newPos2 = p2.position;
	Vec3 newVel2 = p2.velocity;

	if (!p1.isFixed) {
		newPos1 = calculateNextPosition(p1.position, timeStep, velMidstep1);
		newVel1 = calculateNextVelocity(p1.velocity, timeStep, posMidstep1, posMidstep2, s.stiffness, s.initial_len, p1.mass);
		checkCollision(newPos1, newVel1);
	}
	if (!p2.isFixed) {
		newPos2 = calculateNextPosition(p2.position, timeStep, velMidstep2);
		newVel2 = calculateNextVelocity(p2.velocity, timeStep, posMidstep2, posMidstep1, s.stiffness, s.initial_len, p2.mass);
		checkCollision(newPos2, newVel2);
	}

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

	// internal friction removes energy
	forceVec -= m_fDamping * oldVel;

	// calculate the accelaration with the force and the mass
	Vec3 gravityVec = Vec3(0, -gravity, 0);
	Vec3 accVec = (forceVec / mass) + gravityVec;

	// calculate the actual next velocity
	return oldVel + timeStep * accVec;
}

void MassSpringSystemSimulator::onClick(int x, int y) {
	if (m_iTestCase != 3) {
		return;
	}
	if (!currentlyClicking) {
		clickedPos.x = x;
		clickedPos.y = y;
		currentlyClicking = true;
		//cout << "Click: " << x << ", " << y << endl;
	}
	m_mouse.x = x;
	m_mouse.y = y;

	middlePointY = clickedPos.y - m_mouse.y;
	int middleIndex = (number_rows / 2) * number_columns + number_columns / 2;
	massPoint_list[middleIndex].position.y = 0.001 * (clickedPos.y - m_mouse.y);

	//cout << "onClick: " << x << ", " << y << endl;
}

void MassSpringSystemSimulator::onMouse(int x, int y) {
	currentlyClicking = false;
	//cout << "onMouse: " << x << ", " << y << endl;
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