#include "DiffusionSimulator.h"
#include "pcgsolver.h"
using namespace std;

Grid::Grid() {
	int width = 15;
	int height = 15;
	vector<vector<Real>> tempField = createTempField(width, height);
	vector<vector<Real>> tempFieldNew = createTempField(width, height);
	setAttributes(width, height, 0.1, 0.1, tempField,tempFieldNew);

}

Grid::Grid(int width, int height) {
	vector<vector<Real>> tempField = createTempField(width, height);
	vector<vector<Real>> tempFieldNew = createTempField(width, height);
	setAttributes(width, height, 0.1, 0.1, tempField,tempFieldNew);
}

Grid::Grid(int width, int height, Real distance, double sphereSize, vector<vector<Real>> tempField, vector<vector<Real>> tempFieldNew) {
	setAttributes(width, height, distance, sphereSize, tempField, tempFieldNew);
}

void Grid::setAttributes(int width, int height, Real distance, double sphereSize, vector<vector<Real>> tempField, vector<vector<Real>> tempFieldNew) {
	this->width = width;
	this->height = height;
	this->distance = distance;
	this->sphereSize = sphereSize;
	this->tempField = tempField;
	this->tempFieldNew = tempFieldNew;
}

void Grid::setNewValuesForTemp() {
	for (int i = 0; i < width; i++) {
		for (int j = 0; j < height; j++) {
			tempField[i][j] = tempFieldNew[i][j];
		}
	}
}

Real Grid::getTempAt(int x, int y) {
	if (x >= width || y >= height || x < 0 || y < 0) {
		cout << "You're trying to access a point on the temperature field that does not exist: " << x << ", " << y << endl;
		return 0;
	}
	return tempField[x][y];
}

int Grid::getWidth() {
	return width;
}

Real Grid::getDistance() {
	return distance;
}

int Grid::getHeight() {
	return height;
}

void Grid::setTempAt(int x, int y, Real temp) {
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

void Grid::setTempAtNew(int x, int y, Real temp) {
	if (x >= width || y >= height || x < 0 || y < 0) {
		cout << "You're trying to set a point on the temperature field that does not exist: " << x << ", " << y << endl;
		return;
	}
	if (x == 0 || y == 0 || x == width - 1 || y == height - 1) {
		cout << "The temperatures on the border should stay 0°C! You tried to change it." << endl;
		return;
	}

	tempFieldNew[x][y] = temp;
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
vector<vector<Real>> Grid::createTempField(int width, int height) {
	vector<vector<Real>> tempField;
	for (int x = 0; x < width; x++) {
		vector<Real> currentVector;
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
/*
Grid* DiffusionSimulator::diffuseTemperatureExplicit(float timeStep) {//add your own parameters
	Grid* newT = new Grid(grid_width,grid_height);
	for (int i = 1; i < grid_width - 1; i++) {
		for (int j = 1; j < grid_height - 1; j++) {
			Real newtemp;
			newtemp = T->getTempAt(i, j) + timeStep * alpha * ((T->getTempAt(i + 1, j) - 2 * T->getTempAt(i, j) + T->getTempAt(i - 1, j)) / pow(T->getDistance(), 2) +
				(T->getTempAt(i , j + 1) - 2 * T->getTempAt(i, j) + T->getTempAt(i , j - 1)) / pow(T->getDistance(), 2));
			newT->setTempAt(i, j, newtemp);

		}
	}
	// to be implemented
	//make sure that the temperature in boundary cells stays zero
	return newT;
}

*/


Grid* DiffusionSimulator::diffuseTemperatureExplicit(float timeStep) {//add your own parameters
	//Grid* newT = new Grid(grid_width, grid_height);
	for (int i = 1; i < grid_width - 1; i++) {
		for (int j = 1; j < grid_height - 1; j++) {
			Real factor1 = (T->getTempAt(i + 1, j) - 2 * T->getTempAt(i, j) + T->getTempAt(i - 1, j));
			Real factor2 = (T->getTempAt(i, j + 1) - 2 * T->getTempAt(i, j) + T->getTempAt(i, j - 1));
			Real newtemp;
			newtemp = T->getTempAt(i, j) + timeStep * alpha * (factor1 + factor2);
			T->setTempAtNew(i, j, newtemp);

		}
	}
	// to be implemented
	//make sure that the temperature in boundary cells stays zero
	T->setNewValuesForTemp();
	
	return T;
}

void setupB(std::vector<Real>& b, Grid* T) {//add your own parameters
	// to be implemented
	//set vector B[sizeX*sizeY]
	int width = T->getWidth();
	int height = T->getHeight();
	for (int x = 0; x < width; x++) {
		for (int y = 0; y < height; y++) {
			b.at(x + y * width) = T->getTempAt(x, y);
		}
	}

	// print vector b
	/*
	cout << "Vector b: " << endl;
	cout << "[ " << endl;
	for (int i = 0; i < b.size(); i++) {
		cout << b[i] << endl;
	}
	cout << " ]" << endl;
	*/
}

void fillT(vector<Real> t, Grid& T) {//add your own parameters
	// to be implemented
	//fill T with solved vector x
	//make sure that the temperature in boundary cells stays zero

	int width = T.getWidth();
	int height = T.getHeight();
	for (int x = 0; x < width; x++) {
		for (int y = 0; y < height; y++) {

			// we don't change boundary values of T
			if (x == 0 || x == T.getWidth() - 1 || y == 0 || y == T.getHeight() - 1)
				continue;

			T.setTempAt(x, y, t[x + y * T.getWidth()]);
		}
	}

	// print vector t
	/*
	cout << "Vector t: " << endl;
	cout << "[ " << endl;
	for (int i = 0; i < t.size(); i++) {
		cout << t[i] << endl;
	}
	cout << " ]" << endl;
	*/
}

void setupA(SparseMatrix<Real>& A, double factor, Grid* T) {//add your own parameters
	// to be implemented
	//setup Matrix A[sizeX*sizeY*sizeZ, sizeX*sizeY*sizeZ]
	// set with:  A.set_element( index1, index2 , value );
	// if needed, read with: A(index1, index2);
	// avoid zero rows in A -> set the diagonal value for boundary cells to 1.0
	A.zero();
	for (int i = 0; i < A.n; i++) {
		for (int j = 0; j < A.n; j++) {

			// all rows that correspond to a boundary cell get a 1 on the diagonal line
			if (i < T->getWidth() || i >= A.n - T->getWidth() || i % T->getWidth() == 0 || i % T->getWidth() == T->getWidth() - 1) {
				A.set_element(i, i, 1);
				goto OUTER;
				continue;
			}

			if (i == j) {
				// set diagonal to (1 + 4 * delta t * alpha)
				A.set_element(i, j, 1 + 4 * factor);
				// set left and right from diagonal to (- delta t * alpha)
				A.set_element(i, j + 1, -factor);
				A.set_element(i, j - 1, -factor);
				// set width cells to the right and width cells to the left from the diagonal to (- delta t * alpha)
				A.set_element(i, j + T->getWidth(), -factor);
				A.set_element(i, j - T->getWidth(), -factor);
			}
		}
	OUTER: continue;
	}

	// print A
	/*
	cout << "Matrix A:" << endl;
	for (int i = 0; i < A.n; i++) {
		for (int j = 0; j < A.n; j++) {
			cout << "  " << A(i, j) << "  ";
		}
		cout << endl;
	}
	*/
}


void DiffusionSimulator::diffuseTemperatureImplicit(float timeStep) {//add your own parameters
	// solve A T = b
	// to be implemented
	const int N = T->getHeight() * T->getWidth();//N = sizeX*sizeY*sizeZ
	SparseMatrix<Real>* A = new SparseMatrix<Real>(N);
	std::vector<Real>* b = new std::vector<Real>(N);

	setupA(*A, alpha * timeStep, T);
	setupB(*b, T);

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
	
	fillT(x, *T);//copy x to T
}


void DiffusionSimulator::simulateTimestep(float timeStep)
{
	// user changed something in the UI so the scene needs to reload
	if (grid_width != previous_grid_width || grid_height != previous_grid_height || alpha != previous_alpha) {
		previous_grid_width = grid_width;
		previous_grid_height = grid_height;
		previous_alpha = alpha;
		if (m_iTestCase == 0) {
			setupDemo1();
		}
		else setupDemo2();
		
	}
	// to be implemented
	// update current setup for each frame
	switch (m_iTestCase)
	{
	case 0:
		T = diffuseTemperatureExplicit(timeStep);
		break;
	case 1:
		diffuseTemperatureImplicit(timeStep);
		break;
	}

}

void DiffusionSimulator::setupDemo1() {
	T = new Grid(grid_width, grid_height);

	//T->setTempAt(grid_width / 2, grid_height / 2, -1000);
	//T->setTempAt((grid_width / 2) + 1, grid_height / 2, -1000);

	std::mt19937 eng;
	std::uniform_real_distribution<Real> randTemp(-100, 100);
	for (int i = 1; i < grid_width - 1; i++) {
		for (int j = 1; j < grid_height - 1; j++) {
			T->setTempAt(i, j, randTemp(eng));
		}
	}
}

void DiffusionSimulator::setupDemo2() {
	T = new Grid(grid_width, grid_height);

	//T->setTempAt(grid_width / 2, grid_height / 2, -1000);
	//T->setTempAt((grid_width / 2) + 1, grid_height / 2, -1000);
	//T->setTempAt((grid_width / 2) - 1, grid_height / 2, 1000);

	std::mt19937 eng;
	std::uniform_real_distribution<Real> randTemp(-100, 100);
	for (int i = 1; i < grid_width - 1 ; i++) {
		for (int j = 1; j < grid_height - 1; j++) {
			T->setTempAt(i, j, randTemp(eng));
			//T->setTempAt(i, j, -100);
		}
	}

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
