#ifndef DIFFUSIONSIMULATOR_h
#define DIFFUSIONSIMULATOR_h

#include "Simulator.h"
#include "vectorbase.h"

// The grid class stores a (width x height)-sized temperature field with max temparatures of 100°C and min temperatures of -100°C 
class Grid {
public:
	// Construtors
	Grid();
	Grid(int width, int height);
	Grid(int width, int height, double distance, double sphereSize, vector<vector<double>> tempField);

	// Getter and Setter
	int getWidth();
	int getHeight();
	double getTempAt(int x, int y);
	void setTempAt(int x, int y, double temp);
	void setAttributes(int width, int height, double distance, double sphereSize, vector<vector<double>> tempField);

	void drawTempField(DrawingUtilitiesClass* DUC);
	vector<vector<double>> createTempField(int width, int height);


private:
	// width and height of the temperature field
	int width;
	int height;
	// distance between the points
	double distance;
	// size of the spheres
	double sphereSize;
	// storing all temperatures in a 2-dimensional vector
	vector<vector<double>> tempField;
};



class DiffusionSimulator:public Simulator{
public:
	// Construtors
	DiffusionSimulator();

	// Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void simulateTimestep(float timeStep);
	void externalForcesCalculations(float timeElapsed) {};
	void onClick(int x, int y);
	void onMouse(int x, int y);
	// Specific Functions
	void drawObjects();
	Grid* diffuseTemperatureExplicit();
	void diffuseTemperatureImplicit();

	void setupDemo1();
	void setupDemo2();

private:
	// Attributes
	Vec3  m_vfMovableObjectPos;
	Vec3  m_vfMovableObjectFinalPos;
	Vec3  m_vfRotate;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;

	Grid *T; //save results of every time step

	double alpha;
	int grid_width;
	int grid_height;
	
	// these variables keep track if User changed any of the above variables in the UI
	double previous_alpha;
	int previous_grid_width;
	int previous_grid_height;
};

#endif