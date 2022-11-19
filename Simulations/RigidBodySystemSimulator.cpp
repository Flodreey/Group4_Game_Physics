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

}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase) {
	m_iTestCase = testCase;
	switch (m_iTestCase)
	{
	case 0:
		cout << "Demo 1\n";
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
	return 0;
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i) {
	return Vec3();
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i) {
	return Vec3();
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i) {
	return Vec3();
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force) {

}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass) {

}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation) {

}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity) {

}