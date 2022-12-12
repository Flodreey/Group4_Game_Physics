#include "DiffusionSimulator.h"
#include "pcgsolver.h"
using namespace std;

Grid::Grid() {
	int width = 15;
	int height = 15;
	vector<vector<double>> tempField = createTempField(width, height);
	setAttributes(width, height, 0.1, 0.1, tempField);

}

Grid::Grid(int width, int height) {
	vector<vector<double>> tempField = createTempField(width, height);
	setAttributes(width, height, 0.1, 0.1, tempField);
}

Grid::Grid(int width, int height, double distance, double sphereSize, vector<vector<double>> tempField) {
	setAttributes(width, height, distance, sphereSize, tempField);
}

void Grid::setAttributes(int width, int height, double distance, double sphereSize, vector<vector<double>> tempField) {
	this->width = width;
	this->height = height;
	this->distance = distance;
	this->sphereSize = sphereSize;
	this->tempField = tempField;
}

double Grid::getTempAt(int x, int y) {
	if (x >= width || y >= height || x < 0 || y < 0) {
		cout << "You're trying to access a point on the temperature field that does not exist: " << x << ", " << y << endl;
		return 0;
	}
	return tempField[x][y];
}

int Grid::getWidth() {
	return width;
}

int Grid::getHeight() {
	return height;
}

void Grid::setTempAt(int x, int y, double temp) {
	if (x >= width || y >= height || x < 0 || y < 0) {
		cout << "You're trying to set a point on the temperature field that does not exist: " << x << ", " << y << endl;
		return;
	}
	if (x == 0 || y == 0 || x == width - 1 || y == height - 1) {
		cout << "The temperatures on the border should stay 0°C! You tried to change it." << endl;
		return;
	}

	tempField[x][y] = temp;
}

void Grid::drawTempField(DrawingUtilitiesClass* DUC) {
	//Vec3 color = 0.6 * Vec3(0.97, 0.86, 1);
	Vec3 color;

	// determine the range [-xStart, xStart] and [-yStart, yStart] where the spheres should be drawn
	double xStart, yStart;
	if (width % 2 == 0)
		xStart = (width - 1) / 2.0;
	else
		xStart = floor(width / 2.0);
	if (height % 2 == 0)
		yStart = (height - 1) / 2.0;
	else
		yStart = floor(height / 2.0);

	int index1 = 0;
	for (double x = -xStart; x <= xStart; x++, index1++) {
		int index2 = 0;
		for (double y = -yStart; y <= yStart; y++, index2++) {

			// determining color depending on the temperature (max temperature is 100°C)
			double currentTemp = tempField[index1][index2];
			if (currentTemp == 0) {
				color = Vec3();
			} else if (currentTemp > 0) {
				color = Vec3(currentTemp / 100);
			} else if (currentTemp < 0) {
				color = Vec3(abs(currentTemp) / 100, 0, 0);
			}
			DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 10, color);
			DUC->drawSphere(Vec3(x * distance, y * distance, 0), Vec3(sphereSize, sphereSize, sphereSize));
		}
	}
}

// creates a temperature field with a given width and height and sets all temperatures to 0°C
vector<vector<double>> Grid::createTempField(int width, int height) {
	vector<vector<double>> tempField;
	for (int x = 0; x < width; x++) {
		vector<double> currentVector;
		for (int y = 0; y < height; y++) {
			currentVector.push_back(0);
		}
		tempField.push_back(currentVector);
	}
	return tempField;
}


DiffusionSimulator::DiffusionSimulator()
{
	m_iTestCase = 0;
	m_vfMovableObjectPos = Vec3();
	m_vfMovableObjectFinalPos = Vec3();
	m_vfRotate = Vec3();
	
	T = new Grid();

	alpha = 0.5;
	grid_width = T->getWidth();
	grid_height = T->getHeight();
	
	previous_alpha = alpha;
	previous_grid_width = grid_width;
	previous_grid_height = grid_height;
}

const char * DiffusionSimulator::getTestCasesStr(){
	return "Explicit_solver, Implicit_solver";
}

void DiffusionSimulator::reset(){
		m_mouse.x = m_mouse.y = 0;
		m_trackmouse.x = m_trackmouse.y = 0;
		m_oldtrackmouse.x = m_oldtrackmouse.y = 0;

}

void DiffusionSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
	// user can change the width and height of the temperature field and the alpha value of the heat equation
	TwAddVarRW(DUC->g_pTweakBar, "Alpha", TW_TYPE_DOUBLE, &alpha, "min=0 max=1 step=0.05");
	TwAddVarRW(DUC->g_pTweakBar, "Grid Width", TW_TYPE_INT32, &grid_width, "min=3");
	TwAddVarRW(DUC->g_pTweakBar, "Grid Height", TW_TYPE_INT32, &grid_height, "min=3");
}

void DiffusionSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	m_vfMovableObjectPos = Vec3(0, 0, 0);
	m_vfRotate = Vec3(0, 0, 0);
	//
	//to be implemented
	//
	switch (m_iTestCase)
	{
	case 0:
		setupDemo1();
		cout << "Explicit solver!\n";
		break;
	case 1:
		setupDemo2();
		cout << "Implicit solver!\n";
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

Grid* DiffusionSimulator::diffuseTemperatureExplicit() {//add your own parameters
	Grid* newT = new Grid();
	// to be implemented
	//make sure that the temperature in boundary cells stays zero
	return newT;
}

void setupB(std::vector<Real>& b) {//add your own parameters
	// to be implemented
	//set vector B[sizeX*sizeY]
	for (int i = 0; i < 25; i++) {
		b.at(i) = 0;
	}
}

void fillT() {//add your own parameters
	// to be implemented
	//fill T with solved vector x
	//make sure that the temperature in boundary cells stays zero
}

void setupA(SparseMatrix<Real>& A, double factor) {//add your own parameters
	// to be implemented
	//setup Matrix A[sizeX*sizeY*sizeZ, sizeX*sizeY*sizeZ]
	// set with:  A.set_element( index1, index2 , value );
	// if needed, read with: A(index1, index2);
	// avoid zero rows in A -> set the diagonal value for boundary cells to 1.0
	for (int i = 0; i < 25; i++) {
			A.set_element(i, i, 1); // set diagonal
	}
}


void DiffusionSimulator::diffuseTemperatureImplicit() {//add your own parameters
	// solve A T = b
	// to be implemented
	const int N = 25;//N = sizeX*sizeY*sizeZ
	SparseMatrix<Real> *A = new SparseMatrix<Real> (N);
	std::vector<Real> *b = new std::vector<Real>(N);

	setupA(*A, 0.1);
	setupB(*b);

	// perform solve
	Real pcg_target_residual = 1e-05;
	Real pcg_max_iterations = 1000;
	Real ret_pcg_residual = 1e10;
	int  ret_pcg_iterations = -1;

	SparsePCGSolver<Real> solver;
	solver.set_solver_parameters(pcg_target_residual, pcg_max_iterations, 0.97, 0.25);

	std::vector<Real> x(N);
	for (int j = 0; j < N; ++j) { x[j] = 0.; }

	// preconditioners: 0 off, 1 diagonal, 2 incomplete cholesky
	solver.solve(*A, *b, x, ret_pcg_residual, ret_pcg_iterations, 0);
	// x contains the new temperature values
	fillT();//copy x to T
}



void DiffusionSimulator::simulateTimestep(float timeStep)
{
	// to be implemented
	// update current setup for each frame
	switch (m_iTestCase)
	{
	case 0:
		//T = diffuseTemperatureExplicit();
		break;
	case 1:
		diffuseTemperatureImplicit();
		break;
	}

	// user changed something in the UI so the scene needs to reload
	if (grid_width != previous_grid_width || grid_height != previous_grid_height || alpha != previous_alpha) {
		previous_grid_width = grid_width;
		previous_grid_height = grid_height;
		previous_alpha = alpha;
		setupDemo1();
	}
}

void DiffusionSimulator::setupDemo1() {
	T = new Grid(grid_width, grid_height);
	T->setTempAt(grid_width / 2, grid_height / 2, 80);
}

void DiffusionSimulator::setupDemo2() {

}

void DiffusionSimulator::drawObjects()
{
	T->drawTempField(DUC);
}


void DiffusionSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	drawObjects();
}

void DiffusionSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void DiffusionSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}
