#ifndef MYGLUTWINDOW_H_
#define MYGLUTWINDOW_H_

#include <iostream>
#include <cstdlib>

#include <GL/freeglut.h>
//#include <GL/glut.h>
#include <GL/gl.h>
#include <stdio.h>
#include "Colors.h"

#define LIST_ROBOT 	1
#define LIST_GRID 	2

using namespace std;

// function prototypes



#include "MapBuilder.h"

// Robot functions
int glSetMapBuilder(CMapBuilder* builder);
void DrawRobot(float x, float y, float ang, int color=C_RED);
void DrawLaser();
int DrawObsMap(bool extrude);
int DrawMap(bool extrude = true, int color = C_YELLOW);
void DrawRobot();


// Window functions
void ResetView();
void SetColor(int color);
void Draw(void);
void DrawGrid();
void Keyb(unsigned char key, int x, int y);
void Reshape(int width, int height);
void MouseFunc(int button, int state, int x, int y);
void MouseMovesFunc(int x, int y);
void DrawFrame2D(float x, float y, float ang, int color = C_FRAME, int width = 3);
void RecomputePOV();

// Init function
int CreateGlutWindow(const char* name, int width, int height, bool fullscreen=false);
void InitGLUT();

#endif
