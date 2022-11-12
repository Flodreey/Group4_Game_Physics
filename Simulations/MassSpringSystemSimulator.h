#ifndef MASSSPRINGSYSTEMSIMULATOR_h
#define MASSSPRINGSYSTEMSIMULATOR_h
#include "Simulator.h"

// Do Not Change
#define EULER 0
#define LEAPFROG 1
#define MIDPOINT 2
// Do Not Change

struct MassPoint {
	int index;
	float mass;
	Vec3 position;
	Vec3 velocity;
	bool isFixed;
};

struct Spring {
	float stiffness;
	float initial_len;
	float current_len;
	Vec3 massPoint1;
	Vec3 massPoint2;
};

class MassSpringSystemSimulator:public Simulator{
public:
	// Construtors
	MassSpringSystemSimulator();
	
	// UI Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void drawDemo1();
	void setupDemo1();
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// Specific Functions
	void setMass(float mass);
	void setStiffness(float stiffness);
	void setDampingFactor(float damping);
	int addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed);
	void addSpring(int masspoint1, int masspoint2, float initialLength);
	void deleteAllPointsAndSprings();
	int getNumberOfMassPoints();
	int getNumberOfSprings();
	MassPoint getMassPoint(int index, bool& outSuccess);
	Vec3 getPositionOfMassPoint(int index);
	Vec3 getVelocityOfMassPoint(int index);
	void applyExternalForce(Vec3 force);
	void drawSphere(Vec3 pos);
	void drawLine(Vec3 pos1, Vec3 pos2);
	void drawAllMassPoints();
	void drawAllSprings();
	float getLengthOfVec3(Vec3 vec);
	
	// Do Not Change
	void setIntegrator(int integrator) {
		m_iIntegrator = integrator;
	}

private:
	// Data Attributes
	float m_fMass;
	float m_fStiffness;
	float m_fDamping;
	int m_iIntegrator;

	float h;

	float sphereSize;
	int number_massPoints;
	int number_springs;

	std::vector<MassPoint> massPoint_list;
	std::vector<Spring> spring_list;


	// UI Attributes
	Vec3 m_externalForce;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
};
#endif