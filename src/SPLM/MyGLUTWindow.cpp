#include "MyGLUTWindow.h"
#include "Colors.h"
#include <iostream>
#include <math.h>

static int listRobot = 0;
static int listGrid = 0;
static int bkColor = C_BLACK;

#ifndef PI
#define PI  	3.141592654f
#endif

#ifndef TWOPI
#define TWOPI	6.283185307f
#endif

#ifndef HALFPI
#define HALFPI 	1.570796327f
#endif

#ifndef DEG2RAD
#define DEG2RAD	0.017453293f
#endif

#ifndef RAD2DEG
#define RAD2DEG	57.295779513f
#endif

// Poit of view variables:
float alfa = HALFPI / 2; // ...rad = 45º
float beta = HALFPI / 2; // ...rad = 45º
float cosAlfa = cos(alfa);
float sinAlfa = sin(alfa);
float cosBeta = cos(beta);
float sinBeta = sin(beta);
float distEye = 15;
// Desired viewpoint
float eyeX = distEye * cosBeta * cosAlfa;
float eyeY = distEye * cosBeta * sinAlfa;
float eyeZ = distEye * sinBeta;
// Point in the center of the scene being looked at
float centerX = 0;
float centerY = 0;
float centerZ = 0;
// Which direction is up
float upX = 0;
float upY = 0;
float upZ = 1;
// Screen reference coordinates for mouse operations
int refX = 0;
int refY = 0;

#define STATE_NONE	0
#define STATE_PAN 	1
#define STATE_ZOOM 	2
#define STATE_ORBIT	3

int wndState = STATE_NONE;

float robotX = 0;
float robotY = 0;
float robotAng = 0;

CMapBuilder* glpBuilder = NULL;
int glSetMapBuilder(CMapBuilder* builder) {
	glpBuilder = builder;
	return 0;
}

int CreateGlutWindow(const char* name, int width, int height, bool fullscreen) {
	// specify the display mode to be RGB and single buffering
	// we use single buffering since this will be non animated
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE); /* set display mode */
	// define the size
	glutInitWindowSize(width, height);
	// the position where the window will appear
	glutInitWindowPosition(100, 100);
	if (fullscreen)
		glutFullScreen();
	// create the window, set the title and keep the
	// window identifier.
	int id = glutCreateWindow(name);
	// CALLBACK
	glutDisplayFunc(Draw);
	glutKeyboardFunc(Keyb);
	glutReshapeFunc(Reshape); /* register callback for window resize */
	glutMouseFunc(MouseFunc);
	glutMotionFunc(MouseMovesFunc);
	// define the color we use to clear screen
	glClearColor(0.0, 0.0, 0.0, 1.0);
	gluLookAt(eyeX, eyeY, eyeZ, centerX, centerY, centerZ, upX, upY, upZ);
/*
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_AUTO_NORMAL);
	glEnable(GL_NORMALIZE);
	glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
*/


	return id;
}

void Reshape(int width, int height) {
	/* set the viewport to the window width and height */
	glViewport(0, 0, width, height);
	/* load a projection matrix that matches the window aspect ratio */
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0, (double) width / (double) height, 1.0, 100.0);
	/* reset the modelview matrix */
	glMatrixMode(GL_MODELVIEW);
}

// http://www.tayloredmktg.com/rgb
void SetColor(int color) {
	if (bkColor == C_WHITE) {
		switch (color) {
		case C_RED:
			glColor3ub(255, 0, 0);
			break;
		case C_GREEN:
			glColor3ub(0, 255, 0);
			break;
		case C_BLUE:
			glColor3ub(0, 0, 255);
			break;
		case C_CYAN:
			glColor3ub(0, 255, 255);
			break;
		case C_MAGENTA:
			glColor3ub(255, 0, 255);
			break;
		case C_YELLOW:
			glColor3ub(255, 255, 0);
			break;
		case C_ORANGE:
			glColor3ub(255, 165, 0);
			break;
		case C_GREY_L:
			glColor3ub(150, 150, 150);
			break;
		case C_GREY_D:
			glColor3ub(50, 50, 50);
			break;
		case C_BLACK:
			glColor3ub(0, 0, 0);
			break;
		case C_WHITE:
			glColor3ub(255, 255, 255);
			break;
		}
	} else {
		switch (color) {
		case C_RED:
			glColor3ub(255, 0, 0);
			break;
		case C_GREEN:
			glColor3ub(0, 255, 0);
			break;
		case C_BLUE:
			glColor3ub(0, 0, 255);
			break;
		}
	}
}

void DrawRobot(float x, float y, float ang, int color) {
	glPushMatrix();
	SetColor(color);
	glTranslatef(x, y, 0.0f);
	glRotatef(ang * RAD2DEG, 0, 0, 1);
	DrawRobot();
	DrawFrame2D(0.0, 0.0, 0.0, C_RED, 1);
	glPopMatrix();
}

void DrawRobot() {
	if (listRobot == 0) {
		listRobot = LIST_ROBOT;
		glNewList(listRobot, GL_COMPILE);
		SetColor(C_RED);
		GLUquadricObj* q = gluNewQuadric();
		//glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		//glEnable(GL_LIGHTING);
		gluCylinder(q, 0.25F, 0.25F, 1.0F, 8, 1);
		glTranslatef(0.0F, 0.0F, 1.0f);
		gluDisk(q, 0, 0.25f, 8, 1);
		glTranslatef(0.15F, 0.0F, 0.0f);
		SetColor(C_BLUE);
		gluCylinder(q, 0.1F, 0.1F, 0.15F, 8, 1);
		glTranslatef(0.0F, 0.0F, 0.15f);
		gluDisk(q, 0, 0.1f, 8, 1);
		glTranslatef(0.0F, 0.0F, -0.15f);
		glTranslatef(-0.15F, 0.0F, 0.0f);
		glTranslatef(0.0F, 0.0F, -1.0f);
		//glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		//glDisable(GL_LIGHTING);
		gluDeleteQuadric(q);
		glEndList();
	}
	glPushMatrix();
	glTranslatef(robotX, robotY, 0.0f);
	//glRotatef(robotAng, 0, 0, 1);
	glCallList(listRobot);
	glPopMatrix();
}

void DrawGrid() {
	float width = 2;
	int number = 20;
	if (listGrid == 0) {
		listGrid = LIST_GRID;
		glNewList(listGrid, GL_COMPILE);
		glLineWidth(width);
		glBegin(GL_LINES);
		glNormal3f(0, 0, 1);
		int c;
		for (c = 0; c < number; c++) {
			glVertex3f(c * width, -number * width, 0);
			glVertex3f(c * width, number * width, 0);
			glVertex3f(-c * width, -number * width, 0);
			glVertex3f(-c * width, number * width, 0);

			glVertex3f(-number * width, c * width, 0);
			glVertex3f(number * width, c * width, 0);
			glVertex3f(-number * width, -c * width, 0);
			glVertex3f(number * width, -c * width, 0);
		}
		glEnd();
		glEndList();
	}
	SetColor(C_WHITE);
	glCallList(listGrid);
}

void DrawFrame2D(float x, float y, float ang, int color, int width) {
	SetColor((color == C_FRAME) ? C_RED : color);
	glLineWidth(width);
	float ctheta = (float) cos(ang);
	float stheta = (float) sin(ang);
	glBegin(GL_LINE_STRIP);
	glVertex3f((float) (x - 2 * stheta), (float) (y + 2 * ctheta), 0.0f);
	glVertex3f(x, y, 0.0f);
	glVertex3f((float) (x + ctheta), (float) (y + stheta), 0.0f);
	glEnd();
}

void Draw(void) {
	//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClear(GL_COLOR_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(eyeX, eyeY, eyeZ, centerX, centerY, centerZ, upX, upY, upZ);
	//glMatrixMode(GL_MODELVIEW);
	//glEnable(GL_LIGHTING);
	DrawGrid();
	DrawFrame2D(0, 0, 0);
	glPointSize(10.0f);
	glBegin(GL_POINTS);
	glVertex3f(centerX, centerY, centerZ);
	glEnd();
	if (glpBuilder != NULL) {
		glPushMatrix();
		SetColor(C_GREEN);
		float x = glpBuilder->m_Robot.x;
		float y = glpBuilder->m_Robot.y;
		float ang = glpBuilder->m_Robot.ang;
		glTranslatef(x, y, 0.0f);
		glRotatef(ang * RAD2DEG, 0, 0, 1);
		DrawRobot();
		DrawLaser();

		glPopMatrix();
	}
	DrawObsMap(true);
	DrawMap();
	glutSwapBuffers();
}

void Keyb(unsigned char key, int x, int y) {
	switch (key) {
	case 27: /* escape key */
		exit(0);
		break;
	case 'r':
		ResetView();
		break;
	}
}

void InitGLUT() {
	int nnn = 0;
	glutInit(&nnn, 0);
}

void MouseFunc(int button, int state, int x, int y) {
	refX = x;
	refY = y;
	if (state == GLUT_UP) {
		wndState = STATE_NONE;
		return;
	}
	if (button == GLUT_LEFT_BUTTON) {
		if (glutGetModifiers() == GLUT_ACTIVE_CTRL) {
			wndState = STATE_PAN;
			return;
		} else {
			wndState = STATE_ORBIT;
			return;
		}
	}
	if (button == GLUT_RIGHT_BUTTON) {
		wndState = STATE_ZOOM;
		return;
	}
	//http://www.opengl.org/discussion_boards/ubbthreads.php?ubb=showflat&Number=247293
	if (button == 3) // mouse wheel up
	{
		wndState = STATE_ZOOM;
		MouseMovesFunc(refX, refY + 10);
		wndState = STATE_NONE;
	}
	if (button == 4) // mouse wheel up

	{
		wndState = STATE_ZOOM;
		MouseMovesFunc(refX, refY - 10);
		wndState = STATE_NONE;
	}
}

void MouseMovesFunc(int x, int y) {
	switch (wndState) {
	case STATE_ORBIT:
		beta -= DEG2RAD * (refY - y);
		if (beta > HALFPI)
			beta = HALFPI;
		if (beta < 0.0f)
			beta = 0.0f;
		alfa += DEG2RAD * (refX - x);
		refX = x;
		refY = y;
		RecomputePOV();
		Draw();
		break;
	case STATE_ZOOM:
		distEye += (refY - y) / 10.0f;
		if (distEye < 0.0f)
			distEye = 0.0f;
		if (distEye > 300.0f)
			distEye = 300.0f;
		refX = x;
		refY = y;
		RecomputePOV();
		Draw();
		break;
	case STATE_PAN:
		centerY -= cosAlfa * (float) (refX - x) / 10.0f + sinAlfa
				* (float) (refY - y) / 10.0f;
		centerX -= -sinAlfa * (float) (refX - x) / 10.0f + cosAlfa
				* (float) (refY - y) / 10.0f;
		refX = x;
		refY = y;
		RecomputePOV();
		Draw();
		break;
	}

}

void ResetView() {
	alfa = HALFPI / 2.0f; // rad = 45º
	beta = HALFPI / 2.0f; // rad = 45º
	cosAlfa = cos(alfa);
	sinAlfa = sin(alfa);
	cosBeta = cos(beta);
	sinBeta = sin(beta);
	distEye = 15;
	// Desired viewpoint
	eyeX = distEye * cosBeta * cosAlfa;
	eyeY = distEye * cosBeta * sinAlfa;
	eyeZ = distEye * sinBeta;
	// Point in the center of the scene being looked at
	centerX = 0;
	centerY = 0;
	centerZ = 0;
	// Which direction is up
	upZ = 0;
	upY = 0;
	upZ = 1;
	// Screen reference coordinates for mouse operations
	refX = 0;
	refY = 0;
	Draw();
}

void RecomputePOV() {
	cosBeta = cos(beta);
	sinBeta = sin(beta);
	cosAlfa = cos(alfa);
	sinAlfa = sin(alfa);
	eyeX = centerX + distEye * cosBeta * cosAlfa;
	eyeY = centerY + distEye * cosBeta * sinAlfa;
	eyeZ = centerZ + distEye * sinBeta;
}

void DrawLaser() {
	if (glpBuilder == NULL)
		return; // nothing to do
	int i, j; // counter variables
	glPointSize(2);
	SetColor(C_RED);
	glBegin(GL_POINTS);

	for (i = 0; i < glpBuilder->m_Robot.m_Laser.nData; i++) {
		glVertex2f(glpBuilder->m_Robot.m_Laser.x[i],
				glpBuilder->m_Robot.m_Laser.y[i]);
	}

	glEnd();
	/*
	 glPointSize(4);
	 Color( VERDE);
	 glBegin(GL_POINTS);
	 for (i = 0; i < m_builder->m_Features.size(); i++) // for all matched points
	 {
	 for (j = 0; j < m_builder->m_Features[i]->x.size(); j++) {
	 glVertex2f(m_builder->m_Features[i]->x[j],
	 m_builder->m_Features[i]->y[j]);
	 }
	 }
	 glEnd();

	 Color( GRIS1);
	 glLineWidth(0.01F);

	 glBegin(GL_LINES);
	 for (i = 0; i < m_builder->m_Features.size(); i++) // for detected objects
	 {
	 for (j = 0; j < m_builder->m_Features[i]->x.size(); j++) {
	 glVertex2f(0, 0);
	 glVertex2f(m_builder->m_Features[i]->x[j],
	 m_builder->m_Features[i]->y[j]);
	 }
	 }
	 glEnd();
	 */
}
int DrawObsMap(bool extrude) {
	if (glpBuilder == NULL)
		return 0;
	GLUnurbsObj* nurb;
	nurb = gluNewNurbsRenderer();
	gluNurbsProperty(nurb, GLU_SAMPLING_TOLERANCE, 50.0);
	gluNurbsProperty(nurb, GLU_DISPLAY_MODE, GLU_FILL); //GLU_FILL, GLU_OUTLINE_POLYGON, GLU_OUTLINE_PATCH
	gluNurbsProperty(nurb, GLU_CULLING, GL_TRUE);
	glLineWidth(1.0);
	CBspline* spline;
	int i, j, cpn;
	float xini, yini;
	for (i = 0; i < glpBuilder->m_Features.size(); i++) // for all map observed features...
	{
		spline = glpBuilder->m_Features[i]->spline;
		SetColor((spline->isnew) ? C_RED : C_BLUE);
		cpn = spline->m_iNumber; // Number of control points
		spline->Eval(0, &xini, &yini);
		GLfloat knots[cpn + 4];
		for (j = 0; j < cpn + 4; j++) {
			knots[j] = spline->m_Knots[j];
		}

		if (extrude) {
			float height = 1.0f;
			GLfloat knotsv[4] = { 0, 0, 1, 1 };
			GLfloat ctlpoints[50][2][3]; // Control points of the NURBS
			for (j = 0; j < cpn; j++) {
				ctlpoints[j][1][0] = spline->m_Bx[j];
				ctlpoints[j][1][1] = spline->m_By[j];
				ctlpoints[j][1][2] = 0.0;
				ctlpoints[j][0][0] = spline->m_Bx[j];
				ctlpoints[j][0][1] = spline->m_By[j];
				ctlpoints[j][0][2] = height;
			}
			gluNurbsSurface(nurb, // NURBS object
					cpn + 4, // number of knots (u)
					knots, // array of nondecreasing knot values (u)
					4, // number of knots (v)
					knotsv, // array of nondecreasing knot values (v)
					6, // offset between control points (u)
					3, // offset between control points (v)
					&ctlpoints[0][0][0], 4, // order (u)
					2, // order (v)
					GL_MAP2_VERTEX_3); // nonrational GL_MAP2_VERTEX_3, GL_MAP2_COLOR_4
			gluEndSurface(nurb);
			glBegin(GL_LINES);
			glVertex3f(xini, yini, -0.5F);
			glVertex3f(xini, yini, height + 0.5F);
			glEnd();

		} else {
			GLfloat ctlpoints[cpn][3]; // Control points of the NURBS
			for (j = 0; j < cpn; j++) {
				ctlpoints[j][0] = spline->m_Bx[j];
				ctlpoints[j][1] = spline->m_By[j];
				ctlpoints[j][2] = 0.0;
			}
			gluBeginCurve(nurb);
			gluNurbsCurve(nurb, // NURBS object
					cpn + 4, // number of knots
					knots, // array of nondecreasing knot values
					3, // offset between successive curve control points.
					&ctlpoints[0][0], 4, // order
					GL_MAP1_VERTEX_3); //
			gluEndCurve(nurb);
			glPointSize(5);
			glBegin(GL_POINTS);
			glVertex2f(xini, yini);
			glEnd();
		}
	}
	gluDeleteNurbsRenderer(nurb);
	/*
	 glBegin(GL_LINES);
	 for (i = 0; i < glpBuilder->m_Features.size(); i++) // for detected features
	 {
	 SLAMfeature* feat = glpBuilder->m_Features[i];
	 CBspline* spo = feat->spline;
	 for (int l = 0; l < feat->links.size(); l++) {
	 CBspline* spm = feat->links[l].spline;
	 float x, y;
	 spo->Eval(feat->links[l].Keys[0], &x, &y);
	 glVertex2f(x, y);
	 spm->Eval(feat->links[l].Keys[2], &x, &y);
	 glVertex2f(x, y);
	 spo->Eval(feat->links[l].Keys[1], &x, &y);
	 glVertex2f(x, y);
	 spm->Eval(feat->links[l].Keys[3], &x, &y);
	 glVertex2f(x, y);
	 }
	 }
	 glEnd();
	 */

	return 0;
}
/** Draws the map consisting on Bsplines **/
int DrawMap(bool extrude, int color) {
	if (glpBuilder == NULL)
		return 0;
	CMapSP* map = glpBuilder->m_Map;
	if (map == NULL)
		return 0;
	GLUnurbsObj* nurb;
	nurb = gluNewNurbsRenderer();
	gluNurbsProperty(nurb, GLU_SAMPLING_TOLERANCE, 50.0);
	gluNurbsProperty(nurb, GLU_DISPLAY_MODE, GLU_FILL); //GLU_FILL, GLU_OUTLINE_POLYGON, GLU_OUTLINE_PATCH
	glLineWidth(1.0);

	float height = 1.0f;
	float xini, yini;
	SetColor(color);
	CBspline* spline;
	int i, j;
	for (i = 0; i < map->m_nSP; i++) // for all map splines ...
	{
		spline = map->m_Splines[i];
		int cpn = spline->m_iNumber; // Number of control points
		GLfloat* knots = new GLfloat[cpn + 4];
		for (j = 0; j < cpn + 4; j++) {
			knots[j] = spline->m_Knots[j];
		}
		if (extrude) {
			GLfloat knotsv[4] = {0, 0, 1, 1};
			GLfloat ctlpoints[cpn][2][3]; // Control points of the NURBS
			for (int j = 0; j < cpn; j++) {
				ctlpoints[j][1][0] = spline->m_Bx[j];
				ctlpoints[j][1][1] = spline->m_By[j];
				ctlpoints[j][1][2] = 0.0;
				ctlpoints[j][0][0] = spline->m_Bx[j];
				ctlpoints[j][0][1] = spline->m_By[j];
				ctlpoints[j][0][2] = height;
			}
			gluBeginSurface(nurb);
			gluNurbsSurface(nurb, // NURBS object
					cpn + 4, // number of knots (u)
					knots, // array of nondecreasing knot values (u)
					4, // number of knots (v)
					knotsv, // array of nondecreasing knot values (v)
					6, // offset between successive curve control points (u)
					3, // offset between successive curve control points (v)
					&ctlpoints[0][0][0], 4, // order (u)
					2, // order (v)
					GL_MAP2_VERTEX_3); // nonrational GL_MAP2_VERTEX_3, GL_MAP2_COLOR_4
			gluEndSurface(nurb);

			spline->Eval(0, &xini, &yini);
			glBegin(GL_LINES);
			glVertex3f(xini, yini, -0.5F);
			glVertex3f(xini, yini, height + 0.5F);
			glEnd();

		} else {

			GLfloat ctlpoints[cpn][3]; // Control points of the NURBS
			for (j = 0; j < cpn; j++) {
				ctlpoints[j][0] = spline->m_Bx[j];
				ctlpoints[j][1] = spline->m_By[j];
				ctlpoints[j][2] = 0.0;
			}

			gluBeginCurve(nurb);
			gluNurbsCurve(nurb, // NURBS object
					cpn + 4, // number of knots
					knots, // array of nondecreasing knot values
					3, // offset between successive curve control points.
					&ctlpoints[0][0], 4, // order
					GL_MAP1_VERTEX_3); // nonrational
			gluEndCurve(nurb);
		}
		delete[] knots;
	}
	gluDeleteNurbsRenderer(nurb);
	return 0;
}
