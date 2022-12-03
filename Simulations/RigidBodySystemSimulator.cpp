#include "RigidBodySystemSimulator.h"


RigidBodySystemSimulator::RigidBodySystemSimulator() {
	m_iTestCase = 0;
	selectedBodyIndex = 0;
	mouseForceVec = Vec3();
}

const char* RigidBodySystemSimulator::getTestCasesStr() {
	return "Demo 1, Demo 2, Demo 3, Demo 4";
}


void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC) {
	this->DUC = DUC;

	// user can decide which body he manipulates currently (only in demo 4)
	if (m_iTestCase == 3)
		TwAddVarRW(DUC->g_pTweakBar, "Selected Body", TW_TYPE_INT32, &selectedBodyIndex, "min=0, max=100");
}

void RigidBodySystemSimulator::reset() {
	
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {
	for (int i = 0; i < rigidbodies.size(); i++) {

		// the selected rigidbody gets a different color (only in demo 4)
		Vec3 color;
		if (i == selectedBodyIndex && m_iTestCase == 3)
			color = 0.6 * Vec3(1, 0.65, 0);
		else 
			color = 0.6 * Vec3(0.97, 0.86, 1);

		DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, color);

		Mat4 matrix_scalar;
		Mat4 matrix_rotation;
		Mat4 matrix_translation;
		matrix_scalar.initScaling(rigidbodies[i].size.x, rigidbodies[i].size.y, rigidbodies[i].size.z);
		matrix_rotation = rigidbodies[i].orientation.getRotMat();
		matrix_translation.initTranslation(rigidbodies[i].position.x, rigidbodies[i].position.y, rigidbodies[i].position.z);
		
		DUC->drawRigidBody(matrix_scalar * matrix_rotation * matrix_translation);
		
		//cout << rigidbodies[i].position.x << " X" << endl;
		//cout << rigidbodies[i].position.y << " Y" << endl;
		//cout << rigidbodies[i].position.z << " Z" << endl;
		
	}

	// draw red force vector (only in Demo 2 and Demo 4) from one of the bodiy's corners
	if ((m_iTestCase == 1 || m_iTestCase == 3) && selectedBodyIndex < getNumberOfRigidBodies()) {
		DUC->beginLine();

		Vec3 forcePoint;
		Vec3 size = rigidbodies[selectedBodyIndex].size;
		forcePoint = Vec3(size.x / 2, size.y / 2, size.z / 2); // local point
		Mat4 rotMatrix = rigidbodies[selectedBodyIndex].orientation.getRotMat();
		forcePoint = rigidbodies[selectedBodyIndex].position + rotMatrix.transformVector(forcePoint); // space point

		Vec3 start = forcePoint;
		Vec3 end = start + mouseForceVec;
		DUC->drawLine(start, Vec3(1, 0, 0), end, Vec3(1, 0, 0));
		DUC->endLine();
	}
    
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase) {
	m_iTestCase = testCase;
	switch (m_iTestCase)
	{
	case 0:
		cout << "Demo 1\n";
		setUpDemo1();
		break;
	case 1:
		cout << "Demo 2\n";
		setUpDemo2();
		break;
	case 2:
		cout << "Demo 3\n";
		setUpDemo3();
		break;
	case 3:
		cout << "Demo 4\n";
		setUpDemo4();
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

// calculating initial inertia tensor I_0 with given formular for rectengular box from wikipedia
Mat4 RigidBodySystemSimulator::calculateInitialInertiaTensor(double mass, Vec3 size) {
	double element1 = (1.0 / 12.0) * mass * (pow(size.z, 2) + pow(size.y, 2));
	double element2 = (1.0 / 12.0) * mass * (pow(size.x, 2) + pow(size.z, 2));
	double element3 = (1.0 / 12.0) * mass * (pow(size.x, 2) + pow(size.y, 2));
	Mat4 initial_inertia_tensor = Mat4(element1, 0, 0, 0, 0, element2, 0, 0, 0, 0, element3, 0, 0, 0, 0, 1);
	return initial_inertia_tensor;
}



void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed) {

}

void RigidBodySystemSimulator::simulateTimestep(float timeStep) {

	// nothing should be moving in Demo 1
	if (m_iTestCase == 0)
		return;

	for (int i = 0; i < rigidbodies.size(); i++) {

		rigidbodies[i].position += timeStep * rigidbodies[i].lin_velocity;
		rigidbodies[i].lin_velocity += timeStep * (rigidbodies[i].total_force / rigidbodies[i].mass);
		
		rigidbodies[i].orientation += (timeStep / 2) * Quat(rigidbodies[i].ang_velocity.x, rigidbodies[i].ang_velocity.y, rigidbodies[i].ang_velocity.z, 0) * rigidbodies[i].orientation;

		rigidbodies[i].ang_momentum += timeStep * rigidbodies[i].torque;

		Mat4 rotMatrix = rigidbodies[i].orientation.getRotMat();
		Mat4 rotMatrix_transposed = rotMatrix;
		rotMatrix_transposed.transpose();
		rigidbodies[i].current_inertiaTensor_inversed = rotMatrix * rigidbodies[i].initial_inertiaTensor_inversed * rotMatrix_transposed;

		rigidbodies[i].ang_velocity = rigidbodies[i].current_inertiaTensor_inversed * rigidbodies[i].ang_momentum;

		rigidbodies[i].torque = 0;
		rigidbodies[i].total_force = 0;
	}

	// check collisions between every possible pair of rigidbodies
	for (int i = 0; i < getNumberOfRigidBodies(); i++) {
		for (int j = i + 1; j < getNumberOfRigidBodies(); j++) {
			Mat4 matrixA = getTransformationMatrix(i);
			Mat4 matrixB = getTransformationMatrix(j);
			CollisionInfo info = checkCollisionSAT(matrixA, matrixB);
			if (info.isValid) {
				double bounciness = 1;
				double impulse = calculateImpulse(bounciness, rigidbodies[i], rigidbodies[j], info);
				//cout << "Collsion detected!" << endl;

				rigidbodies[i].lin_velocity += (impulse * info.normalWorld) / rigidbodies[i].mass;
				rigidbodies[j].lin_velocity -= (impulse * info.normalWorld) / rigidbodies[j].mass;

				Vec3 rel_collision_pointA = info.collisionPointWorld - rigidbodies[i].position;
				Vec3 rel_collision_pointB = info.collisionPointWorld - rigidbodies[j].position;
				rigidbodies[i].ang_momentum += cross(rel_collision_pointA, impulse * info.normalWorld);
				rigidbodies[j].ang_momentum -= cross(rel_collision_pointB, impulse * info.normalWorld);

				// maybe fixes the problem that rigidbodies are stuck together
				rigidbodies[i].position += info.normalWorld * info.depth;
			}
		}
	}

	/*
	// check for collisions in Demo 3
	if (m_iTestCase == 2) {

		Mat4 matrixA = getTransformationMatrix(0);
		Mat4 matrixB = getTransformationMatrix(1);
		CollisionInfo info = checkCollisionSAT(matrixA, matrixB);
		if (info.isValid) {
			double impulse = calculateImpulse(1, rigidbodies[0], rigidbodies[1], info);
			cout << "Collsion detected!" << endl;
			cout << "Normal: " << info.normalWorld << endl;

			rigidbodies[0].lin_velocity += (impulse * info.normalWorld) / rigidbodies[0].mass;
			rigidbodies[1].lin_velocity -= (impulse * info.normalWorld) / rigidbodies[1].mass;

			Vec3 rel_collision_pointA = info.collisionPointWorld - rigidbodies[0].position;
			Vec3 rel_collision_pointB = info.collisionPointWorld - rigidbodies[1].position;
			rigidbodies[0].ang_momentum += cross(rel_collision_pointA, impulse * info.normalWorld);
			rigidbodies[1].ang_momentum -= cross(rel_collision_pointB, impulse * info.normalWorld);

			// maybe fixes the problem that rigidbodies are stuck together
			rigidbodies[0].position += info.normalWorld * info.depth;
		}
	}
	*/

}

Mat4 RigidBodySystemSimulator::getTransformationMatrix(int i) {
	RigidBody rigid = getRigidBody(i);
	
	Mat4 scaleMat;
	scaleMat.initScaling(rigid.size.x, rigid.size.y, rigid.size.z);

	Mat4 rotMat = rigid.orientation.getRotMat();

	Mat4 translMat;
	translMat.initTranslation(rigid.position.x, rigid.position.y, rigid.position.z);

	return scaleMat * rotMat * translMat;
}

double RigidBodySystemSimulator::calculateImpulse(double bounciness, RigidBody bodyA, RigidBody bodyB, CollisionInfo info) {

	// collision points relative to the center of mass of bodyA and bodyB
	Vec3 x_a = info.collisionPointWorld - bodyA.position;
	Vec3 x_b = info.collisionPointWorld - bodyB.position;

	// velocity at collision point x_a of bodyA
	Vec3 vel_a = bodyA.lin_velocity + cross(bodyA.ang_velocity, x_a);
	// velocity at collision point x_b of bodyB
	Vec3 vel_b = bodyB.lin_velocity + cross(bodyB.ang_velocity, x_b);
	// relative velocity of the collision
	Vec3 rel_vel = vel_a - vel_b;

	Vec3 normal = info.normalWorld;

	double massA = bodyA.mass;
	double massB = bodyB.mass;

	Mat4 inertia_a = bodyA.current_inertiaTensor_inversed;
	Mat4 inertia_b = bodyB.current_inertiaTensor_inversed;

	// fill everything into the formula
	double numerator = dot(- (1 + bounciness) * rel_vel, normal);
	Vec3 factor1 = cross(inertia_a * cross(x_a, normal), x_a);
	Vec3 factor2 = cross(inertia_b * cross(x_b, normal), x_b);
	double denominator = (1.0 / massA) + (1.0 / massB) + dot(factor1 + factor2, normal);

	return numerator / denominator;
}

void RigidBodySystemSimulator::onClick(int x, int y) {
	//cout << "clicked: x: " << x << ", y: " << y << endl;

	if (m_iTestCase == 0 || m_iTestCase == 2) {
		return;
	}

	// user can't apply force if he clicks into the tweakbar UI
	if (x <= 220 && y <= 340) {
		mouseForceVec = Vec3();
		return;
	}

	// force gets applied when user clicks
	if (selectedBodyIndex < getNumberOfRigidBodies()) {
		// force should be applied at on of the corners
		Vec3 forcePoint;
		Vec3 size = rigidbodies[selectedBodyIndex].size;
		forcePoint = Vec3(size.x / 2, size.y / 2, size.z / 2); // local point
		Mat4 rotMatrix = rigidbodies[selectedBodyIndex].orientation.getRotMat();
		forcePoint = rigidbodies[selectedBodyIndex].position + rotMatrix.transformVector(forcePoint); // space point

		applyForceOnBody(selectedBodyIndex, forcePoint, mouseForceVec);
	}
}

void RigidBodySystemSimulator::onMouse(int x, int y) {
	//cout << "mouse: x: " << x << ", y: " << y << endl;

	// there is no red force vector if mouse is over tweakbar UI
	if (x <= 220 && y <= 340) {
		mouseForceVec = Vec3();
		return;
	}

	// rotation of the mouseForceVec is depending on position of mouse
	double degreeX = (x / 500.0) * 360.0;
	double degreeY = (y / 500.0) * 360.0;
	Mat4 rotMatrix;
	rotMatrix.initRotationXYZ(degreeY, degreeX, 0);
	mouseForceVec = Vec3(0, 0, 1);
	mouseForceVec = rotMatrix.transformVector(mouseForceVec);
}

int RigidBodySystemSimulator::getNumberOfRigidBodies() {
	return rigidbodies.size();
}

RigidBody RigidBodySystemSimulator::getRigidBody(int i) {
	for (int j = 0; j < rigidbodies.size(); j++) {
		if (rigidbodies[j].index == i) {
			return rigidbodies[j];
		}
	}
	cout << "You try to access a rigidbody with index " << i << ", but it doesn't exist!" << endl;
	RigidBody dummy;
	return dummy;
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i) {
	return getRigidBody(i).position;
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i) {
	return getRigidBody(i).lin_velocity;
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i) {
	return getRigidBody(i).ang_velocity;
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force) {
	//cout << "force applied" << endl;
	for (int j = 0; j < rigidbodies.size(); j++) {
		if (rigidbodies[j].index == i) {
			rigidbodies[j].torque += cross(loc - rigidbodies[j].position, force);
			rigidbodies[j].total_force += force;
		}
	}
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass) {
	RigidBody rigidbody;
	rigidbody.index = rigidbodies.size();
	rigidbody.mass = mass;
	rigidbody.position = position;
	rigidbody.size = size;
	rigidbody.lin_velocity = Vec3();
	rigidbody.ang_velocity = Vec3();
	rigidbody.ang_momentum = Vec3();
	rigidbody.orientation = Quat(0, 0, 0, 1);
	rigidbody.initial_inertiaTensor_inversed = calculateInitialInertiaTensor(mass, size).inverse();
	rigidbody.current_inertiaTensor_inversed = Mat4();
	rigidbody.torque = Vec3();
	rigidbody.total_force = Vec3();

	rigidbodies.push_back(rigidbody);
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation) {
	for (int j = 0; j < rigidbodies.size(); j++) {
		if (rigidbodies[j].index == i) {
			rigidbodies[j].orientation = orientation.unit();
		}
	}

}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity) {
	for (int j = 0; j < rigidbodies.size(); j++) {
		if (rigidbodies[j].index == i) {
			rigidbodies[j].lin_velocity = velocity;
		}
	}
}
	
void RigidBodySystemSimulator::setUpDemo1() {
	rigidbodies.clear();
	addRigidBody(Vec3(0, 0, 0), Vec3(1, 0.6, 0.5), 2);
	setOrientationOf(0, Quat(Vec3(0, 0, 1), M_PI * 0.5));
	applyForceOnBody(0, Vec3(0.3, 0.5, 0.25), Vec3(1, 1, 0));
	
	// calculation of basic test case and printing the solution
	Vec3 position = getRigidBody(0).position;
	Vec3 size = getRigidBody(0).size;
	double mass = getRigidBody(0).mass;
	Quat orientation = getRigidBody(0).orientation;
	Vec3 lin_velocity = getLinearVelocityOfRigidBody(0);
	Vec3 ang_velocity = getAngularVelocityOfRigidBody(0);
	Vec3 ang_momentum = getRigidBody(0).ang_momentum;
	Mat4 initial_inertiaTensor_inversed = getRigidBody(0).initial_inertiaTensor_inversed;
	Mat4 current_inertiaTensor_inversed = getRigidBody(0).current_inertiaTensor_inversed;
	Vec3 torque = getRigidBody(0).torque;
	Vec3 total_force = getRigidBody(0).total_force;

	double timestep = 2;

	Mat4 rotMatrix;
	Mat4 rotMatrix_transposed;

	// pre-computing inversed initial inertia tensor I_0 - happens already in addRigidBody(...)

	// initializing current inverse inertia tensor
	rotMatrix = orientation.getRotMat();
	rotMatrix_transposed = rotMatrix;
	rotMatrix_transposed.transpose();
	current_inertiaTensor_inversed = rotMatrix * initial_inertiaTensor_inversed * rotMatrix_transposed;

	// initializing angular velocity w (will be (0, 0, 0) because ang_momentum is (0, 0, 0))
	ang_velocity = current_inertiaTensor_inversed * ang_momentum;

	// calculating total force F and torque q - happens already in applyForceOnBody(...)

	// euler step for position and velocity
	position = position + timestep * lin_velocity;
	lin_velocity = lin_velocity + timestep * (total_force / mass);
	
	// update the orientation (wont change because angular velocity is (0,0,0)
	orientation = orientation + (timestep / 2) * Quat(ang_velocity.x, ang_velocity.y, ang_velocity.z, 0) * orientation;

	// update the angular momentum
	ang_momentum = ang_momentum + timestep * torque;

	// update current inverse inertia tensor
	rotMatrix = orientation.getRotMat();
	rotMatrix_transposed = rotMatrix;
	rotMatrix_transposed.transpose();
	Mat4 inertia_tensor = rotMatrix * initial_inertiaTensor_inversed * rotMatrix_transposed;

	// update angular velocity
	ang_velocity = inertia_tensor * ang_momentum;

	// calculate world space velocity at (-0.3, -0.5, -0.25)
	Vec3 pos = Vec3(-0.3, -0.5, -0.25);
	Vec3 pos_velocity = lin_velocity + cross(ang_velocity, pos);

	// right solution: 
	// linear velocity: (1, 1, 0)
	// angular velocity: (-2.40, 4.92, -1.76)
	// world space velocity at (-0.3, -0.5, -0.25): (-1.11, 0.93, 2.68)
	cout << "---------------------------------------------------------------------------------------------------------" << endl;
	cout << "Rigidbody with size (1, 0.6, 0.5) and mass 2 gets hit at (0.3 ,0.5, 0.25) with force (1, 1, 0)" << endl;
	cout << "Calculating properties after timestep h = 2 ......." << endl << endl;
	cout << "Linear velocity: v = " << lin_velocity << endl;
	cout << "Angular velocity w = " << ang_velocity << endl;
	cout << "World space velocity of point "<< pos << ": v_i = " << pos_velocity << endl;
	cout << "---------------------------------------------------------------------------------------------------------" << endl;


}

void RigidBodySystemSimulator::setUpDemo2() { 
	rigidbodies.clear();

	// setup for simple rigidbody simulation of worksheet
	addRigidBody(Vec3(0, 0, 0), Vec3(1, 0.6, 0.5), 2);
	setOrientationOf(0, Quat(Vec3(0, 0, 1), M_PI * 0.5));
	applyForceOnBody(0, Vec3(0.3, 0.5, 0.25), Vec3(1, 1, 0));
}


void RigidBodySystemSimulator::setUpDemo3() {
	rigidbodies.clear();

	// start conditions
	addRigidBody(Vec3(0, 0, 0), Vec3(1, 0.6, 0.5), 2);
	addRigidBody(Vec3(1.5, 0.2, 0), Vec3(1, 0.6, 0.5), 2);
	setOrientationOf(1, Quat(M_PI * 0.25, M_PI * 0.25, 0));
	applyForceOnBody(1, Vec3(1.5, 0.2, 0), Vec3(-2, 0, 0));

	// produces error!! -- not anymore, but not so sure...
	//applyForceOnBody(1, Vec3(-1.5, 0.1, 0), Vec3(5, -2, 0));
}

void RigidBodySystemSimulator::setUpDemo4() {
	rigidbodies.clear();

	// creates a big cube consisting of 3 x 3 x 3 smaller cubes
	double distance = 0.5;
	double box_size = 0.3;

	for (int i = -1; i <= 1; i++) {
		for (int j = -1; j <= 1; j++) {
			for (int k = -1; k <= 1; k++) {
				addRigidBody(Vec3(i * distance, j * distance, k * distance), Vec3(box_size, box_size, box_size), 1);
			}
		}
	}
}


