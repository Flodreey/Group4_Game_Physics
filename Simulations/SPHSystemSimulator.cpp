#include "SPHSystemSimulator.h"

SPHSystemSimulator::SPHSystemSimulator() {
	particle_size = 0.05;
	kernel_radius = 0.1;
	k = 0.5;
	rest_density = 0.5;
}

const char* SPHSystemSimulator::getTestCasesStr() {
	return "2D, 3D";
}

void SPHSystemSimulator::initUI(DrawingUtilitiesClass* DUC) {
	this->DUC = DUC;
}

void SPHSystemSimulator::reset() {

}

void SPHSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {
	for (int i = 0; i < particles.size(); i++) {
		Vec3 pos = particles[i].position;
		DUC->drawSphere(pos, Vec3(particle_size, particle_size, particle_size));
	}
}

void SPHSystemSimulator::notifyCaseChanged(int testCase) {
	m_iTestCase = testCase;
	switch (m_iTestCase)
	{
	case 0:
		setupDemo1();
		break;
	case 1:
		break;
	default:
		break;
	}
}

void SPHSystemSimulator::externalForcesCalculations(float timeElapsed) {

}

void SPHSystemSimulator::simulateTimestep(float timeStep) {
	for (int i = 0; i < particles.size(); i++) {

		// determine density at each particles position
		particles[i].density = 0;
		for (int j = 0; j < particles.size(); j++) {
			particles[i].density += particles[j].mass * W(particles[i].position, particles[j].position);
		}

		// calculate pressure
		particles[i].pressure = k * (pow(particles[i].density / rest_density, 7) - 1);

		// Determine the pressure force at each particles position
		particles[i].force = Vec3();
		for (int j = 0; j < particles.size(); j++) {
			// calculation of pressure force doesn't include the current particle
			if (i == j)
				continue;

			particles[i].force += ((particles[j].pressure + particles[i].pressure) / 2) * (particles[j].mass / particles[j].density) * nabla_W(particles[i].position, particles[j].position);
		}
		particles[i].force *= -1;

		// euler integration
		particles[i].position += timeStep * particles[i].velocity;

		particles[i].velocity += timeStep * particles[i].accelartion;

		Vec3 gravity = Vec3(0, -10, 0);
		double damping = 3;
		particles[i].force -= damping * particles[i].velocity;
		particles[i].accelartion = (particles[i].force / particles[i].mass) + gravity;

		// collision with boundary
		if (particles[i].position.x > 2) {
			particles[i].position.x = 2;
			particles[i].velocity *= -1;
		}
		else if (particles[i].position.x < -2) {
			particles[i].position.x = -2;
			particles[i].velocity *= -1;
		}
		if (particles[i].position.y > 3) {
			particles[i].position.y = 3;
			particles[i].velocity *= -1;
		}
		else if (particles[i].position.y < -1) {
			particles[i].position.y = -1;
			particles[i].velocity *= -1;
		}
	}
}

void SPHSystemSimulator::onClick(int x, int y) {

}

void SPHSystemSimulator::onMouse(int x, int y) {

}

void SPHSystemSimulator::applyExternalForce(Vec3 force) {

}

// kernel function
double SPHSystemSimulator::W(Vec3 position1, Vec3 position2) {
	Vec3 distanceVec = position1 - position2;
	double distance = sqrt(pow(distanceVec.x, 2) + pow(distanceVec.y, 2) + pow(distanceVec.z, 2));

	if (distance <= kernel_radius)
		return kernel_radius - distance;
	else
		return 0;
}


Vec3 SPHSystemSimulator::nabla_W(Vec3 position1, Vec3 position2) {
	Vec3 distanceVec = position1 - position2;
	double distance = sqrt(pow(distanceVec.x, 2) + pow(distanceVec.y, 2) + pow(distanceVec.z, 2));

	return Vec3(-(distanceVec.x / distance), -(distanceVec.y / distance), -(distanceVec.z / distance));
}

void SPHSystemSimulator::setupDemo1() {
	particles.clear();

	// create all the particles
	double mass = 1;
	int size = 10;
	for (int i = -size / 2; i <= size / 2; i++) {
		for (int j = -size / 2; j <= size / 2; j++) {
			Particle p;
			p.mass = mass;
			p.position = Vec3(i * particle_size, j * particle_size, 0);
			particles.push_back(p);
		}
	}
}