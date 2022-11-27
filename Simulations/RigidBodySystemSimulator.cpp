#include "RigidBodySystemSimulator.h"

RigidBodySystemSimulator::RigidBodySystemSimulator() {
	m_iTestCase = 0;
}

const char* RigidBodySystemSimulator::getTestCasesStr() {
	return "Demo 1, Demo 2, Demo 3, Demo 4";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC) {
	this->DUC = DUC;
}

void RigidBodySystemSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {
	DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(0.97, 0.86, 1));	
	for (int i = 0; i < rigidbodies.size(); i++) {

		Mat4 matrix_scalar;
		Mat4 matrix_rotation;
		Mat4 matrix_translation;
		matrix_scalar.initScaling(rigidbodies[i].size.x, rigidbodies[i].size.y, rigidbodies[i].size.z);
		matrix_rotation = rigidbodies[i].orientation.getRotMat();
		matrix_translation.initTranslation(rigidbodies[i].position.x, rigidbodies[i].position.y, rigidbodies[i].position.z);
		DUC->drawRigidBody(matrix_scalar * matrix_rotation * matrix_translation);
		/*
		cout << rigidbodies[i].position.x << " X" << endl;
		cout << rigidbodies[i].position.y << " Y" << endl;
		cout << rigidbodies[i].position.z << " Z" << endl;
		*/
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

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed) {

}

void RigidBodySystemSimulator::simulateTimestep(float timeStep) {

}

void RigidBodySystemSimulator::onClick(int x, int y) {

}

void RigidBodySystemSimulator::onMouse(int x, int y) {

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
	for (int j = 0; j < rigidbodies.size(); j++) {
		if (rigidbodies[j].index == i) {
			rigidbodies[j].torque += cross(loc, force);
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
	rigidbody.orientation = Quat();
	rigidbody.torque = Vec3();
	rigidbody.total_force = Vec3();

	rigidbodies.push_back(rigidbody);
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation) {
	for (int j = 0; j < rigidbodies.size(); j++) {
		if (rigidbodies[j].index == i) {
			rigidbodies[j].orientation = orientation;
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
	addRigidBody(Vec3(0, 0, 0), Vec3(1, 0.6, 0.5), 2);
	setOrientationOf(0, Quat(Vec3(0, 0, 1), M_PI * 0.5));
	applyForceOnBody(0, Vec3(0.3, 0.5, 0.25), Vec3(1, 1, 0));
	
	// calculation of basic test case and printing the solution
	Vec3 center = getRigidBody(0).position;
	Vec3 size = getRigidBody(0).size;
	double mass = getRigidBody(0).mass;
	Quat orientation = getRigidBody(0).orientation;
	Vec3 lin_velocity = getLinearVelocityOfRigidBody(0);
	Vec3 ang_velocity = getAngularVelocityOfRigidBody(0);
	Vec3 ang_momentum = getRigidBody(0).ang_momentum;
	Vec3 torque = getRigidBody(0).torque;
	Vec3 total_force = getRigidBody(0).total_force;

	double timestep = 2;

	// calculating initial inertia tensor I_0 with given formular for rectengular box from wikipedia
	double element1 = (1.0 / 12.0) * mass * (pow(size.y, 2) + pow(size.z, 2));
	double element2 = (1.0 / 12.0) * mass * (pow(size.x, 2) + pow(size.y, 2));
	double element3 = (1.0 / 12.0) * mass * (pow(size.x, 2) + pow(size.z, 2));
	Mat4 initial_inertia_tensor = Mat4(element1, 0, 0, 0, 0, element2, 0, 0, 0, 0, element3, 0, 0, 0, 0, 1);
	Mat4 initial_inertia_tensor_inverse = initial_inertia_tensor.inverse();

	// calculating torque q - happens already in applyForceOnBody(...)

	// euler step for position and velocity
	center = center + timestep * lin_velocity;
	lin_velocity = lin_velocity + timestep * (total_force / mass);
	
	// update the orientation (wont change because angular velocity is (0,0,0)
	orientation = orientation + (timestep / 2) * Quat(0, ang_velocity.x, ang_velocity.y, ang_velocity.z) * orientation;

	// update the angular momentum
	ang_momentum = ang_momentum + timestep * torque;

	// calculate current inverse inertia tensor
	Mat4 rotMatrixTransposed = orientation.getRotMat();
	rotMatrixTransposed.transpose();
	Mat4 inertia_tensor = orientation.getRotMat() * initial_inertia_tensor_inverse * rotMatrixTransposed;

	// calculate angular velocity
	ang_velocity = inertia_tensor * ang_momentum;

	// calculate world position and velocitiy at world position
	Vec3 pos = Vec3(-0.3, -0.5, -0.25);
	Vec3 world_pos = center + orientation.getRotMat() * pos;
	Vec3 world_velocity = lin_velocity + cross(ang_velocity, pos);

	// right solution: 
	// linear velocity: (1, 1, 0)
	// angular velocity: (-2.40, 4.92, -1.76)
	// world space velocity at (-0.3, -0.5, -0.25): (-1.11, 0.93, 2.68)
	cout << "Rigidbody with size (1, 0.6, 0.5) and mass 2 gets hit at (0.3 ,0.5, 0.25) with force (1, 1, 0)" << endl;
	cout << "Calculating properties after timestep h = 2 ......." << endl;
	cout << "Linear velocity: v = " << lin_velocity << endl;
	cout << "Angular velocity w = " << ang_velocity << endl;
	cout << "World space velocity of point "<< world_pos << ": v_i = " << world_velocity;


}