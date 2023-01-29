#ifndef SPHSYSTEMSIMULATOR_h
#define SPHSYSTEMSIMULATOR_h
#include "Simulator.h"

struct Particle {
	double mass;
	Vec3 position;
	Vec3 velocity;
	Vec3 accelartion;
	double density;
	double pressure;
	Vec3 force;
};

class SPHSystemSimulator :public Simulator {
public:
	// Construtors
	SPHSystemSimulator();

	// UI Functions
	const char* getTestCasesStr();
	void initUI(DrawingUtilitiesClass* DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// Specific Functions
	void applyExternalForce(Vec3 force);

	void setupDemo1();
	double W(Vec3 position, Vec3 position2);
	Vec3 nabla_W(Vec3 position1, Vec3 position2);

private:


	// UI Attributes
	Vec3 m_externalForce;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;

	vector<Particle> particles;
	double particle_size;
	double kernel_radius;
	double k;
	double rest_density;
};
#endif