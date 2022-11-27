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
		Mat4 matrix_rotation = Mat4(1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1);
		Mat4 matrix_translation;
		matrix_translation.initTranslation(rigidbodies[i].position.x, rigidbodies[i].position.y, rigidbodies[i].position.z);
		matrix_scalar.initScaling(rigidbodies[i].size.x, rigidbodies[i].size.y, rigidbodies[i].size.z);
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

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i) {
	if (i < getNumberOfRigidBodies()) {
		return rigidbodies[i].position;
	}
	return Vec3();
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i) {
	if (i < getNumberOfRigidBodies()) {
		return rigidbodies[i].linearVelocity;
	}
	return Vec3();
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i) {
	if (i < getNumberOfRigidBodies()) {
		return rigidbodies[i].angularVelocity;
	}
	return Vec3();
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force) {
	if (i < getNumberOfRigidBodies()) {
	 rigidbodies[i].force += force;
	}

}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass) {
	RigidBody rigidbody;
	rigidbody.mass = mass;
	rigidbody.position = position;
	rigidbody.size = size;
	rigidbodies.push_back(rigidbody);
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation) {

}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity) {

}

void RigidBodySystemSimulator::setUpDemo1() {
	addRigidBody(Vec3(0, 0, 0), Vec3(1, 0.6, 0.5), 2);
	 // I0 berechnung
	vector<Vec3> corners;
	corners.push_back(Vec3(-0.5,-0.3,0.25));
	corners.push_back(Vec3(-0.5, 0.3, 0.25));
	corners.push_back(Vec3(0.5, 0.3, 0.25));
	corners.push_back(Vec3(0.5, -0.3, 0.25));
	corners.push_back(Vec3(-0.5, -0.3, -0.25));
	corners.push_back(Vec3(0.5, -0.3, -0.25));
	corners.push_back(Vec3(0.5, 0.3, -0.25));
	corners.push_back(Vec3(-0.5, 0.3, -0.25));
	float mass = 2.0 / 8.0;
	
	float xx = 0;
	float yy = 0;
	float zz = 0;
	float xy = 0;
	float xz = 0;
	float yz = 0;

	for (Vec3 c : corners) {
		xx += c.x * c.x * mass;
		yy += c.y * c.y * mass;
		zz += c.z * c.z * mass;
		xy += c.x * c.y * mass;
		xz += c.x * c.z * mass;
		yz += c.y * c.z * mass;
	}

	Mat4 covariance = Mat4(xx, xy, xz, 0, xy, yy, yz, 0, xz, yz, zz, 0, 0, 0, 0, 0);
	float trace = xx + yy + zz;
	Mat4 trace_matrix = Mat4(trace, 0, 0, 0, 
				0,trace, 0, 0,
				0, 0, trace, 0, 
				0, 0, 0, 0);
	Mat4 inertia_0 =   trace_matrix - covariance;

	// torgue 
	Vec3 torque;
	Vec3 position = Vec3(0.3, 0.5, 0.25);
	Vec3 force = Vec3(1, 1, 0);
		for (auto& corner : corners) {
			torque += cross(position - corner, force);
	}
		Mat4 rotate;
		rotate.initRotationZ(90);
		Quat angles(rotate);
		// Momentum L
		Vec3 momentum = 2 * torque;

		Mat4 update_tensor;
		update_tensor = rotate * inertia_0.inverse() * rotate.inverse();
		// update angular velocity
		Vec3 angular_velocity = update_tensor * momentum;

}