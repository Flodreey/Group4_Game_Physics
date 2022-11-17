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
	int index;
	float stiffness;
	float initial_len;
	float current_len;
	int massPointIndex1;
	int massPointIndex2;
};

class MassSpringSystemSimulator:public Simulator{
public:
	// Construtors
	MassSpringSystemSimulator();
	
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();

	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void drawAllMassPoints();
	void drawAllSprings();
	void drawSphere(Vec3 pos);
	void drawSphere(Vec3 pos, Vec3 color);
	
	void setupDemo1();
	void setupDemo23();
	void setupDemo4();
	
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void applyExternalForce(Vec3 force);
	void simulateTimestep(float timeStep);
	void checkCollision(Vec3& newPos, Vec3& newVel);
	void doEulerStep(Spring s, float timeStep);
	void doMidpointStep(Spring s, float timeStep);
	Vec3 calculateNextPosition(Vec3 oldPos, float timeStep, Vec3 oldVel);
	Vec3 calculateNextVelocity(Vec3 oldVel, float timeStep, Vec3 oldPos, Vec3 otherOldPos, float stiffness, float initial_len, float mass);
	
	void onClick(int x, int y);
	void onMouse(int x, int y);

	void setMass(float mass);
	void setStiffness(float stiffness);
	void setDampingFactor(float damping);
	
	int addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed);
	int addSpring(int masspoint1, int masspoint2, float initialLength);
	void deleteAllPointsAndSprings();
	int getNumberOfMassPoints();
	int getNumberOfSprings();
	Spring getSpring(int index, bool& outSuccess);
	Spring getSpring(int index);
	MassPoint getMassPoint(int index, bool& outSuccess);
	MassPoint getMassPoint(int index);
	Vec3 getPositionOfMassPoint(int index);
	Vec3 getVelocityOfMassPoint(int index);

	float getLengthOfVec3(Vec3 vec);
	
	// Do Not Change
	void setIntegrator(int integrator) {
		m_iIntegrator = integrator;
	}

private:
	float m_fMass;
	float m_fStiffness;
	float m_fDamping;
	float gravity;
	int m_iIntegrator;

	int number_columns;
	int number_rows;

	// previous variables to detect when they were changed in the UI
	float previousGravity;
	int previousIntegrator;
	int previousNumberColumns;
	int previousNumberRows;
		
	float sphereSize;
	int number_massPoints;
	int number_springs;

	// vectors to store all the mass points and springs
	std::vector<MassPoint> massPoint_list;
	std::vector<Spring> spring_list;

	// not used
	Vec3 m_externalForce;

	// creating a 2d vector to move the yellow mass point with the mouse
	Point2D m_mouse;
	Point2D clickedPos;
	bool currentlyClicking;
};
#endif