/*
 * $Id: main.cpp,v 1.28 2006/11/02 21:50:34 nathanst Exp $
 *
 * This file is part of HOG.
 *
 * HOG is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * HOG is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with HOG; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifdef NO_OPENGL
#include "glut.h"
#else
#ifdef OS_MAC
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
#endif

#include "unitSimulation.h"
#include "trackball.h"
#include "common.h"

#include "main.h"
#include "TextBox.h"

#include <cstring>
#include <cstdlib>
#include "limits.h"
#include "time.h"

using namespace std;

pRecContext pContextInfo;
GLint gDollyPanStartPoint[2] = {0, 0};
GLfloat gTrackBallRotation [4] = {0.0f, 0.0f, 0.0f, 0.0f};
GLboolean gDolly = GL_FALSE;
GLboolean gPan = GL_FALSE;
GLboolean gTrackball = GL_FALSE;
pRecContext gTrackingContextInfo = NULL;
int gCurrButton = -1;
//bool pointpath = false;
//int ppMouseClicks = 0;
pRecContext backup;
double fps = 30.0;
pRecContext getCurrentContext()
{
	return pContextInfo;
}

void cleanup (void)
{
	return; // ignore this for now. let the OS clean things up
	delete pContextInfo->unitLayer;
	delete pContextInfo;
	assert(graph_object::gobjCount == 0);
	free(getHome());
	cleanupHandlers();
}



int main(int argc, char** argv)
{	
	/* where are we? */
	char* hh = 0;
	char* val = getenv("HOGHOME");
	if(val != NULL)
	{
		hh = (char*)malloc(sizeof(char*)*strlen(val));
		strcpy(hh, val);
	}
	else 
		hh = getcwd(val, PATH_MAX);

	setHome(hh);

	// Init traj global
	startTrajRecap = false;

	srand(time(0));
	
	installCommandLineHandler(processFramesPerSecond, "-fps", "-fps <int>", "[System Option] Specifies the maximum frames per second.");
	initializeHandlers();
	processCommandLineArgs(argc, argv);
	
	pContextInfo = new recContext;
	//resetCamera(&(pContextInfo->camera));
	atexit (cleanup);

	if(!getDisableGUI())
	{
		glutInit(&argc, argv);
		glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
		glutInitWindowPosition(100, 100);
		glutInitWindowSize(700, 700);
		glutCreateWindow("Map Abstraction");
		glutReshapeFunc(resizeWindow);
		glutDisplayFunc(renderScene);
		glutIdleFunc(renderScene);
		glutMouseFunc(mousePressedButton);
		glutMotionFunc(mouseMovedButton);
		glutKeyboardFunc(keyPressed);
		
		initialConditions(pContextInfo);
		buildGL();
		createMenus();
		glutMainLoop();
	}
	else
	{
		createSimulation(pContextInfo->unitLayer);
	}
	
	return 0;
}

void createMenus()
{
	int menu, submenu;
	
	// create the menu and
	// tell glut that "processMenuEvents" will 
	// handle the events
	submenu = glutCreateMenu(processMenuEvents);
	glutAddMenuEntry((char*)"Map1",1);
	glutAddMenuEntry((char*)"Map2",2);
	glutAddMenuEntry((char*)"Map3",3);
	menu = glutCreateMenu(processMenuEvents);
	
	//add entries to our menu
	glutAddSubMenu((char*)"Open Map",submenu);
	glutAddMenuEntry((char*)"Clear Map...",0);
	
	// attach the menu to the right button
	glutAttachMenu(GLUT_MIDDLE_BUTTON);
}

void processMenuEvents(int option)
{
	switch (option) {
		case 1:
			sprintf(gDefaultMap, "/Users/nathanst/Development/hog/maps/bgmaps/AR0011SR.map");
			processStats(pContextInfo->unitLayer->getStats());
			delete pContextInfo->unitLayer;
			createSimulation(pContextInfo->unitLayer);
			break;
		case 2:
			sprintf(gDefaultMap, "/Users/nathanst/Development/hog/maps/wc3maps/divideandconquer.map");
			processStats(pContextInfo->unitLayer->getStats());
			delete pContextInfo->unitLayer;
			createSimulation(pContextInfo->unitLayer);
			break;
		case 3:
			sprintf(gDefaultMap, "/Users/nathanst/Development/hog/maps/wc3maps/mysticisles.map");
			processStats(pContextInfo->unitLayer->getStats());
			delete pContextInfo->unitLayer;
			createSimulation(pContextInfo->unitLayer);
			break;
		case 0:
			gDefaultMap[0] = 0;
			processStats(pContextInfo->unitLayer->getStats());
			delete pContextInfo->unitLayer;
			createSimulation(pContextInfo->unitLayer);
			break;
		default: printf("Unknown menu event: %d\n", option);
			break;
	}
}

int processFramesPerSecond(char *argument[], int maxNumArgs)
{
	if (maxNumArgs <= 1)
		return 0;
	fps = atof(argument[1]);
	if (fps == 0)
		fps = 1.0;
	return 2;
}

//void pointPath() {
//	
//	ppMouseClicks = 0;
//
//	if (!pointpath) {
//		glutSetCursor(GLUT_CURSOR_RIGHT_ARROW);
//		backup = new recContext;
//		*backup = *pContextInfo;
//		
//		// Reset the view so that you are looking straight down on the map
//		resetCamera (&pContextInfo->camera);
////		pContextInfo->worldRotation[0] = 180;
////		pContextInfo->worldRotation[1] = 0;
////		pContextInfo->worldRotation[2] = -.5;
////		pContextInfo->worldRotation[3] = .5;
//	}
//	else {
//		glutSetCursor(GLUT_CURSOR_INHERIT);
//		*pContextInfo = *backup;
//		if (backup) {
//			delete backup;
//			backup = 0;
//		}
//		
//	}
//	
//	pointpath = 1 - pointpath;
//}



/**
 * Called when a key is pressed, and no other keys are held down.
 */
void keyPressed(unsigned char key, int x, int y)
{
	x+=y;
	bool shift = (glutGetModifiers() == GLUT_ACTIVE_SHIFT);
	bool alt = (glutGetModifiers() == GLUT_ACTIVE_ALT);
	bool cntrl = (glutGetModifiers() == GLUT_ACTIVE_CTRL);
	doKeyboardCommand(pContextInfo, key, shift, cntrl, alt);
}



/**
 * Called when the mouse is moved with a button pressed down.
 */
void mouseMovedButton(int x, int y)
{
	point3d p = GetOGLPos(x, y);
	tButtonType bType = kLeftButton;
	switch (gCurrButton)
	{
		case GLUT_RIGHT_BUTTON: bType = kRightButton; break;
		case GLUT_LEFT_BUTTON: bType = kLeftButton; break;
		case GLUT_MIDDLE_BUTTON: bType = kMiddleButton; break;
	}
	if (handleMouseClick(pContextInfo, x, y, p, bType, kMouseDrag))
		return;
	
	/*if (currentButton == GLUT_LEFT_BUTTON)
		mousePan(x, y, pContextInfo);
	else
		mouseDolly(x, y, pContextInfo); */
	if (gTrackball) {
		rollToTrackball((long) x, (long) y, gTrackBallRotation);
	} 
	else if (gDolly) {
		mouseDolly(x, y, pContextInfo);
	} 
	else if (gPan) {
		mousePan(x, y, pContextInfo);
	}
	
}


/**
 * Called when a mouse button is pressed.
 */
void mousePressedButton(int button, int state, int x, int y)
{
	gCurrButton = button;
	int modifiers = glutGetModifiers();
	
	//printf("Button = %d\n", button);
	if (state == GLUT_DOWN) {
		point3d p = GetOGLPos(x, y);
		tButtonType bType = kLeftButton;
		switch (gCurrButton)
		{
			case GLUT_RIGHT_BUTTON: bType = kRightButton; break;
			case GLUT_LEFT_BUTTON: bType = kLeftButton; break;
			case GLUT_MIDDLE_BUTTON: bType = kMiddleButton; break;
		}
		if (handleMouseClick(pContextInfo, x, y, p, bType, kMouseDown))
			return;
		//		if (pointpath) {
//			ppMouseClicks++;
//			
//			if (ppMouseClicks > 1) {
//				pointPath();
//			}
//			
//			return;
//		}
	

		if (button == GLUT_LEFT_BUTTON)
		{
			if(modifiers == GLUT_ACTIVE_SHIFT) // zoom
			{ 
				if (gTrackball) { // if we are currently tracking, end trackball
					gTrackball = GL_FALSE;
					if (gTrackBallRotation[0] != 0.0)
						addToRotationTrackball (gTrackBallRotation, pContextInfo->worldRotation);
					gTrackBallRotation [0] = gTrackBallRotation [1] = gTrackBallRotation [2] = gTrackBallRotation [3] = 0.0f;
				} 
				else if (gPan) { // if we are currently panning, end pan
					gPan = GL_FALSE;
				}
				gDollyPanStartPoint[0] = (long) x;
				gDollyPanStartPoint[1] = (long) y;
				gDolly = GL_TRUE;
				gTrackingContextInfo = pContextInfo;
			}
			else if(modifiers == GLUT_ACTIVE_CTRL) // rotate
			{
				if (gDolly) { // if we are currently dollying, end dolly
					gDolly = GL_FALSE;
					gTrackingContextInfo = NULL;
				}
				else if (gPan) { // if we are currently panning, end pan
					gPan = GL_FALSE;
					gTrackingContextInfo = NULL;
				}
				startTrackball((long) x, (long) y, (long)pContextInfo->camera.viewOriginX, (long)pContextInfo->camera.viewOriginY, pContextInfo->camera.viewWidth, pContextInfo->camera.viewHeight);
				gTrackball = GL_TRUE;
				gTrackingContextInfo = pContextInfo;
			}
			else // pan
			{
				if (gTrackball) { // if we are currently tracking, end trackball
					gTrackball = GL_FALSE;
					if (gTrackBallRotation[0] != 0.0)
						addToRotationTrackball (gTrackBallRotation, pContextInfo->worldRotation);
					gTrackBallRotation [0] = gTrackBallRotation [1] = gTrackBallRotation [2] = gTrackBallRotation [3] = 0.0f;
				} 
				else if (gDolly) { // if we are currently dollying, end dolly
						gDolly = GL_FALSE;
				}
				gDollyPanStartPoint[0] = (GLint) x;
				gDollyPanStartPoint[1] = (GLint) y;
				gPan = GL_TRUE;
				gTrackingContextInfo = pContextInfo;
			}
		} 
	}
	// stop trackball, pan, or dolly
	else {
		point3d p = GetOGLPos(x, y);
		tButtonType bType = kLeftButton;
		switch (gCurrButton)
		{
			case GLUT_RIGHT_BUTTON: bType = kRightButton; break;
			case GLUT_LEFT_BUTTON: bType = kLeftButton; break;
			case GLUT_MIDDLE_BUTTON: bType = kMiddleButton; break;
		}
		if (handleMouseClick(pContextInfo, x, y, p, bType, kMouseUp))
			return;

		if (gDolly) { // end dolly
			gDolly = GL_FALSE;
		} 
		else if (gPan) { // end pan
			gPan = GL_FALSE;
		} 
		else if (gTrackball) { // end trackball
			gTrackball = GL_FALSE;
			if (gTrackBallRotation[0] != 0.0)
				addToRotationTrackball (gTrackBallRotation, pContextInfo->worldRotation);
			gTrackBallRotation [0] = gTrackBallRotation [1] = gTrackBallRotation [2] = gTrackBallRotation [3] = 0.0f;
		} 
		gTrackingContextInfo = NULL;
	}
}


// move camera in x/y plane
static void mousePan (int x, int y, pRecContext pContextInfo)
{
	GLfloat panX = (gDollyPanStartPoint[0] - x) / (900.0f / -pContextInfo->camera.viewPos.z);
	GLfloat panY = (gDollyPanStartPoint[1] - y) / (900.0f / -pContextInfo->camera.viewPos.z);
	pContextInfo->camera.viewPos.x += panX;
	pContextInfo->camera.viewPos.y += panY;
	gDollyPanStartPoint[0] = (long) x;
	gDollyPanStartPoint[1] = (long) y;
}


// move camera in z axis
static void mouseDolly (int x, int y, pRecContext pContextInfo)
{
	GLfloat dolly = (gDollyPanStartPoint[1] - y) * -pContextInfo->camera.viewPos.z / 300.0f;
	pContextInfo->camera.viewPos.z += dolly;
	if (pContextInfo->camera.viewPos.z == 0.0) // do not let z = 0.0
		pContextInfo->camera.viewPos.z = 0.0001;
	updateProjection (pContextInfo);  // update projection matrix
	gDollyPanStartPoint[0] = (long) x;
	gDollyPanStartPoint[1] = (long) y;
}


/**
 * Renders the scene.  Used by GLUT for it's display function.
 * Wraps the drawGL() function.
 */
void renderScene(void)
{

  // Update the tank model frame (basically the treads)
  //tankModelFrameUpdate();
  
  static double lastTime = ((double)clock()/CLOCKS_PER_SEC);
  double currTime = ((double)clock()/CLOCKS_PER_SEC);

  if ((currTime - lastTime) > (1/fps)) {
    lastTime = currTime;
    drawGL(pContextInfo);
  }

}


/**
 * Called when the window is resized.  Specific format for GLUT.
 */
void resizeWindow(int x, int y)
{
	CGRect rect;
	rect.size.width = x;
	rect.size.height = y;
	rect.origin.x = 0;
	rect.origin.y = 0;
	resizeGL(pContextInfo, rect);
}


/**
 * Handles resizing of GL need context update and if the window dimensions change,
 * a window dimension update, reseting of viewport and an update of the projection matrix
 */
void resizeGL(pRecContext pContextInfo, CGRect viewRect)
{
    	if (!pContextInfo)
        	return;

	pContextInfo->camera.viewOriginX = viewRect.origin.x;
	pContextInfo->camera.viewOriginY = viewRect.origin.y;
	pContextInfo->camera.viewWidth = (GLint)viewRect.size.width;
	pContextInfo->camera.viewHeight = (GLint)viewRect.size.height;
	glViewport (0, 0, pContextInfo->camera.viewWidth, pContextInfo->camera.viewHeight);
	updateProjection (pContextInfo);  // update projection matrix
}



/**
 * Update the projection matrix based on camera and view info.
 * Should be called when viewport size, eye z position, or camera aperture changes.
 * Also call if far or near changes which is determined by shape size in this case.
 */
void updateProjection(pRecContext pContextInfo)
{
	GLdouble ratio, radians, wd2;
	GLdouble left, right, top, bottom, near, far;


	// set projection
	glMatrixMode (GL_PROJECTION);
	glLoadIdentity ();
	near = -pContextInfo->camera.viewPos.z - pContextInfo->shapeSize * 0.5;
	far = -pContextInfo->camera.viewPos.z + pContextInfo->shapeSize * 0.5;
	if (far < 4.0)
	{
		far = sqrt(pContextInfo->camera.viewDir.x*pContextInfo->camera.viewDir.x+
							 pContextInfo->camera.viewDir.y*pContextInfo->camera.viewDir.y+
							 pContextInfo->camera.viewDir.z+pContextInfo->camera.viewDir.z);
		far *= 2;
	}
	if (near < 1.0)
		near = 0.125; //far = 3;
		
	radians = 0.0174532925 * pContextInfo->camera.aperture / 2; // half aperture degrees to radians 
	wd2 = near * tan(radians);
	ratio = pContextInfo->camera.viewWidth / (float) pContextInfo->camera.viewHeight;
	if (ratio >= 1.0) {
		left  = -ratio * wd2;
		right = ratio * wd2;
		top = wd2;
		bottom = -wd2;	
	} else {
		left  = -wd2;
		right = wd2;
		top = wd2 / ratio;
		bottom = -wd2 / ratio;	
	}
	glFrustum (left, right, bottom, top, near, far);
}


/**
 * Updates the viewpoint of the model.
 */
void updateModelView (pRecContext pContextInfo)
{
	
	// move view
	glMatrixMode (GL_MODELVIEW);
	glLoadIdentity ();
	gluLookAt (pContextInfo->camera.viewPos.x, pContextInfo->camera.viewPos.y, pContextInfo->camera.viewPos.z,
			   pContextInfo->camera.viewPos.x + pContextInfo->camera.viewDir.x,
			   pContextInfo->camera.viewPos.y + pContextInfo->camera.viewDir.y,
			   pContextInfo->camera.viewPos.z + pContextInfo->camera.viewDir.z,
			   pContextInfo->camera.viewUp.x, pContextInfo->camera.viewUp.y ,pContextInfo->camera.viewUp.z);
	
	//Camera trackball rotation		   		
	if ((gTrackingContextInfo == pContextInfo) && gTrackBallRotation[0] != 0.0f) // if we have trackball rotation to map (this IS the test I want as it can be explicitly 0.0f)
		glRotatef (gTrackBallRotation[0], gTrackBallRotation[1], gTrackBallRotation[2], gTrackBallRotation[3]);
	else {
	}
	
	// accumlated world rotation via trackball
	glRotatef (pContextInfo->worldRotation[0], pContextInfo->worldRotation[1], pContextInfo->worldRotation[2], pContextInfo->worldRotation[3]);
	// object itself rotating applied after camera rotation
	glRotatef (pContextInfo->objectRotation[0], pContextInfo->objectRotation[1], pContextInfo->objectRotation[2], pContextInfo->objectRotation[3]);
	pContextInfo->fRot[0] = 0.0f; // reset animation rotations (do in all cases to prevent rotating while moving with trackball)
	pContextInfo->fRot[1] = 0.0f;
	pContextInfo->fRot[2] = 0.0f;
}


/**
 * Draws a CString in OpenGL
 */
void drawCStringGL (char * cstrOut, GLuint fontList)
{
	GLint i = 0;
	if (!cstrOut)
		return;
	while (cstrOut [i])
		glCallList (fontList + cstrOut[i++]);
}

TextBox *myTextBox = 0;

void appendTextToBuffer(char *tempStr)
{
	int ind = strlen(pContextInfo->message);
	pContextInfo->message[ind] = ' ';
	sprintf(&pContextInfo->message[ind+1], "%s", tempStr);

	delete myTextBox;
	point3d a(-.95, .95, -.95), b(.95, -.95, .95);
	recColor rc = {1, 1, 1};
	myTextBox = new TextBox(pContextInfo->message, 120, a, b, 1000, true);
	myTextBox->setColor(rc);
}

void submitTextToBuffer(const char *val)
{
	strncpy(pContextInfo->message, val, 255);
	delete myTextBox;
	point3d a(-.95, .95, -.95), b(.95, -.95, .95);
	recColor rc = {1, 1, 1};
	myTextBox = new TextBox(pContextInfo->message, 120, a, b, 1000, true);
	myTextBox->setColor(rc);
}

/**
 * Displays the info on the screen.
 */
static void drawInfo (pRecContext /*pContextInfo*/)
{
	if (myTextBox)
	{
		myTextBox->draw();
	}

#if defined (OS_MAC) && ! defined (NO_OPENGL)
	//static float msgPresistance = 10.0f;
	char cstr [256];
	GLint matrixMode, line = 1;
	GLboolean depthTest = glIsEnabled (GL_DEPTH_TEST);
	GLfloat height, width;
	
	if (!pContextInfo)
		return;
	
	height = pContextInfo->camera.viewHeight;
	width = pContextInfo->camera.viewWidth;

	glDisable (GL_DEPTH_TEST); // ensure text is not remove by deoth buffer test.
	glEnable (GL_BLEND); // for text fading
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); // ditto
	
	// set orthograhic 1:1  pixel transform in local view coords
	glGetIntegerv (GL_MATRIX_MODE, &matrixMode);
	glMatrixMode (GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity ();
	glMatrixMode (GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity ();
	glScalef (2.0 / width, -2.0 /  height, 1.0);
	glTranslatef (-width / 2.0, -height / 2.0, 0.0);
	// output strings
	glColor3f (1.0, 1.0, 1.0);
	sprintf (cstr, "Camera at (%0.1f, %0.1f, %0.1f) looking at (%0.1f, %0.1f, %0.1f) with %0.1f aperture", 
					 pContextInfo->camera.viewPos.x, pContextInfo->camera.viewPos.y, pContextInfo->camera.viewPos.z,
					 pContextInfo->camera.viewDir.x, pContextInfo->camera.viewDir.y, pContextInfo->camera.viewDir.z,
					 pContextInfo->camera.aperture);
	glRasterPos3d (10, line++ * 12, 0); 
	drawCStringGL (cstr, pContextInfo->boldFontList);
	//sprintf (cstr, "Trackball Rotation: (%0.1f, %0.2f, %0.2f, %0.2f)", gTrackBallRotation[0], gTrackBallRotation[1], gTrackBallRotation[2], gTrackBallRotation[3]);
	glRasterPos3d (10, line++ * 12, 0); 
	drawCStringGL (cstr, pContextInfo->regFontList);
	sprintf (cstr, "World Rotation: (%0.1f, %0.2f, %0.2f, %0.2f)", pContextInfo->worldRotation[0], pContextInfo->worldRotation[1], pContextInfo->worldRotation[2], pContextInfo->worldRotation[3]);
	glRasterPos3d (10, line++ * 12, 0); 
	drawCStringGL (cstr, pContextInfo->regFontList);
	sprintf (cstr, "Vertices: %ld, Color Scheme: %ld", pContextInfo->subdivisions * pContextInfo->xyRatio * pContextInfo->subdivisions, pContextInfo->colorScheme);
	glRasterPos3d (10, line++ * 12, 0); 
	drawCStringGL (cstr, pContextInfo->regFontList);
	{
		GLboolean twoSidedLighting, localViewer;
		glGetBooleanv (GL_LIGHT_MODEL_LOCAL_VIEWER, &localViewer);
		glGetBooleanv (GL_LIGHT_MODEL_TWO_SIDE, &twoSidedLighting);
		if (!pContextInfo->lighting) {
			sprintf (cstr, "-- Lighting off");
		} else {
			if (!twoSidedLighting)
				sprintf (cstr, "-- Single Sided Lighting");
			else
				sprintf (cstr, "-- Two Sided Lighting");
			if (localViewer)
				sprintf (cstr, "%s: Local Viewer", cstr);
		}	
		glRasterPos3d (10, line++ * 12, 0); 
		drawCStringGL (cstr, pContextInfo->regFontList);
	}

	glRasterPos3d (10, line++ * 12, 0); 
#ifndef kUseMultiSample
	sprintf (cstr, "-- FSAA: Off");
#else
	switch (pContextInfo->modeFSAA) {
	case kFSAAOff:
		sprintf (cstr, "-- %d x FSAA: Disabled", kSamples);
		break;
	case kFSAAFast:
		sprintf (cstr, "-- %d x FSAA: Fastest hint", kSamples);
		break;
	case kFSAANice:
		sprintf (cstr, "-- %d x FSAA: Nicest hint", kSamples);
		break;
	}
#endif
	drawCStringGL (cstr, pContextInfo->regFontList);

	// message string
	/*if (pContextInfo->message[0]) {
		float currDelta = getElapsedTime () - pContextInfo->msgTime;
		glColor4f (1.0, 1.0, 1.0, (msgPresistance - currDelta) * 0.1);
		glRasterPos3d (10, line++ * 12, 0); 
		drawCStringGL (pContextInfo->message, pContextInfo->boldFontList);
		if (currDelta > msgPresistance)
		pContextInfo->message[0] = 0;
		}*/
	// global error message
	/*if (gErrorMessage[0]) {
		float currDelta = getElapsedTime () - gErrorTime;
		glColor4f (1.0, 0.2, 0.2, (msgPresistance - currDelta) * 0.1);
		glRasterPos3d (10, line++ * 12, 0); 
		drawCStringGL (gErrorMessage, pContextInfo->boldFontList);
		if (currDelta > msgPresistance)
		gErrorMessage[0] = 0;
		}*/
	if (pContextInfo->showCredits) {
		char *strName=0, *strAuthor=0, *strX=0, *strY=0, *strZ=0, *strRange=0;
		//GetStrings(pContextInfo->surface, &strName, &strAuthor, &strX, &strY, &strZ, &strRange);
		line = 10;
		glColor3f (1.0f, 1.0f, 0.0f);
		glRasterPos3d (10, line++ * 12, 0); 
		drawCStringGL (strName, pContextInfo->boldFontList);
		glRasterPos3d (10, line++ * 12, 0); 
		drawCStringGL (strAuthor, pContextInfo->regFontList);
		glColor3f (0.7f, 0.7f, 0.0f);
		glRasterPos3d (10, line++ * 12, 0); 
		drawCStringGL (strX, pContextInfo->regFontList);
		glRasterPos3d (10, line++ * 12, 0); 
		drawCStringGL (strY, pContextInfo->regFontList);
		glRasterPos3d (10, line++ * 12, 0); 
		drawCStringGL (strZ, pContextInfo->regFontList);
		glRasterPos3d (10, line++ * 12, 0); 
		drawCStringGL (strRange, pContextInfo->regFontList);
	}
	if (pContextInfo->drawHelp) {
		line = 4;
		glColor3f (0.8f, 0.8f, 0.8f);
		glRasterPos3d (10, height - line++ * 12, 0); 
		drawCStringGL ((char*)"\\: cycle surface type", pContextInfo->regFontList);
		glRasterPos3d (10, height - line++ * 12, 0); 
		drawCStringGL ((char*)"; & ': decrease/increase subdivisions", pContextInfo->regFontList);
		glRasterPos3d (10, height - line++ * 12, 0); 
		drawCStringGL ((char*)"[ & ]: cycle color scheme", pContextInfo->regFontList);
		glRasterPos3d (10, height - line++ * 12, 0); 
		drawCStringGL ((char*)"'l': cycle lighting  'm': cycle full scene anti-aliasing", pContextInfo->regFontList);
		glRasterPos3d (10, height - line++ * 12, 0); 
		drawCStringGL ((char*)"'p': toggle points   'w': toggle wireframe   'f': toggle fill", pContextInfo->regFontList);
		glRasterPos3d (10, height - line++ * 12, 0); 
		drawCStringGL ((char*)"'q': toggle credits  'c': toggle OpenGL caps", pContextInfo->regFontList);
		glRasterPos3d (10, height - line++ * 12, 0); 
		drawCStringGL ((char*)"Cmd-A: animate       Cmd-I: show info", pContextInfo->regFontList);
		glRasterPos3d (10, height - line++ * 12, 0); 
		drawCStringGL ((char*)"Wheel: zoom camera", pContextInfo->boldFontList);
		glRasterPos3d (10, height - line++ * 12, 0); 
		drawCStringGL ((char*)"Middle Button Drag: dolly object", pContextInfo->boldFontList);
		glRasterPos3d (10, height - line++ * 12, 0); 
		drawCStringGL ((char*)"Right Button Drag: pan object", pContextInfo->boldFontList);
		glRasterPos3d (10, height - line++ * 12, 0); 
		drawCStringGL ((char*)"Left Button Drag: rotate object", pContextInfo->boldFontList);
		glRasterPos3d (10, height - line++ * 12, 0); 
		drawCStringGL ((char*)"-- Help ('h') --", pContextInfo->boldFontList);
	}

	glColor3f (1.0, 1.0, 1.0);
	glRasterPos3d (10, height - 27, 0); 
	sprintf (cstr, "(%0.0f x %0.0f)", width, height);
	drawCStringGL (cstr, pContextInfo->boldFontList);
	// render and vendor info
	glRasterPos3d (10, height - 15, 0); 
	drawCStringGL ((char*) glGetString (GL_RENDERER), pContextInfo->boldFontList);
	glRasterPos3d (10, height - 3, 0); 
	drawCStringGL ((char*) glGetString (GL_VERSION), pContextInfo->regFontList);
	/*if (pContextInfo->drawCaps) {
		drawCaps (pContextInfo);
		}*/
	// reset orginal martices
	glPopMatrix(); // GL_MODELVIEW
	glMatrixMode (GL_PROJECTION);
	glPopMatrix();
	glMatrixMode (matrixMode);
	if (depthTest)
		glEnable (GL_DEPTH_TEST);
	//glReportError ();
#endif
}


/**
 * Main OpenGL drawing function.
 */
void drawGL (pRecContext pContextInfo)
{
 if (!pContextInfo)
        return;

	// clear our drawable
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	// projection matrix already set	
	updateModelView (pContextInfo);

	glDisable(GL_LIGHTING);
	pContextInfo->unitLayer->getMap()->openGLDraw(kPolygons);
	if (pContextInfo->unitLayer->getMapAbstractionDisplay())
	{
		pContextInfo->unitLayer->getMapAbstractionDisplay()->openGLDraw();
	}
	glEnable(GL_LIGHTING);

	static double lastTime = ((double)clock()/CLOCKS_PER_SEC);
	double currTime = ((double)clock()/CLOCKS_PER_SEC);

	pContextInfo->unitLayer->advanceTime(currTime-lastTime);
	lastTime = currTime;
	if (pContextInfo->drawing)
	{
		pContextInfo->unitLayer->openGLDraw();
		if (pContextInfo->info) {
			glDisable(GL_LIGHTING);
			drawInfo (pContextInfo);
		}
	}
	frameCallback(pContextInfo->unitLayer);
	
	/*
	if (pContextInfo->unitLayer->getUnitGroup(2) != NULL) {
	  if (static_cast<probGroup *>(pContextInfo->unitLayer->getUnitGroup(2))->getStartTrajRecap())
	    static_cast<probGroup *>(pContextInfo->unitLayer->getUnitGroup(2))->drawRecap();
	    }*/

	//glFlush();
	glutSwapBuffers();

}


/**
 * End OpenGL drawing function - for visualizing trajectory merging
 */
/*void trajectoryDrawGL (pRecContext pContextInfo)
{
 if (!pContextInfo)
        return;

	// clear our drawable
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	// projection matrix already set	
	updateModelView (pContextInfo);

	glDisable(GL_LIGHTING);
	pContextInfo->unitLayer->getMap()->openGLDraw(kPolygons);
	if (pContextInfo->unitLayer->getMapAbstractionDisplay())
	{
		pContextInfo->unitLayer->getMapAbstractionDisplay()->openGLDraw();
	}
	glEnable(GL_LIGHTING);

	static double lastTime = ((double)clock()/CLOCKS_PER_SEC);
	double currTime = ((double)clock()/CLOCKS_PER_SEC);

	pContextInfo->unitLayer->advanceTime(currTime-lastTime);
	lastTime = currTime;
	if (pContextInfo->drawing)
	{
		pContextInfo->unitLayer->openGLDraw();
		if (pContextInfo->info) {
			glDisable(GL_LIGHTING);
			drawInfo (pContextInfo);
		}
	}
	frameCallback(pContextInfo->unitLayer);

	//glFlush();
	glutSwapBuffers();

}*/


/**
 * Initializes OpenGL.
 */
void buildGL(void)
{ 
    	if (NULL == pContextInfo)
        	return;
		
	// build context
	CGRect viewRect = {{0.0f, 0.0f}, {0.0f, 0.0f}};
	
	switch (pContextInfo->modeFSAA) {
		case kFSAAOff:
#ifndef WIN32
			//glDisable (GL_MULTISAMPLE_ARB);
#endif
			break;
		case kFSAAFast:
		        #ifndef WIN32
			//glEnable (GL_MULTISAMPLE_ARB);
			//glHint (GL_MULTISAMPLE_FILTER_HINT_NV, GL_FASTEST);
                        #endif
			break;
		case kFSAANice:
		        #ifndef WIN32
			//glEnable (GL_MULTISAMPLE_ARB);
			//glHint (GL_MULTISAMPLE_FILTER_HINT_NV, GL_NICEST);
			#endif
			break;
	}
		
	// init GL stuff here
	glEnable(GL_DEPTH_TEST);

	glShadeModel(GL_SMOOTH);    
	glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
	glFrontFace(GL_CCW);
	glPolygonOffset (1.0, 1.0);
		
	glClearColor(0.0,0.0,0.0,0.0);
	pContextInfo->shapeSize = 3.0f; // max radius of of objects

	/*GetFNum ("\pGeneva", &fNum); // build font
	pContextInfo->boldFontList = buildFontGL (pContextInfo->aglContext, fNum, bold, 9);
	pContextInfo->regFontList = buildFontGL (pContextInfo->aglContext, fNum, normal, 9);
	*/
	
	// setup viewport and prespective
	/*GetWindowPortBounds (window, &rectPort);
	viewRect.size.width = (float) (rectPort.right - rectPort.left);
	viewRect.size.height = (float) (rectPort.bottom - rectPort.top);
	*/
	
	viewRect.size.width = glutGet(GLUT_WINDOW_WIDTH);
	viewRect.size.height = glutGet(GLUT_WINDOW_HEIGHT);
	
	// setup viewport and prespective
	resizeGL(pContextInfo, viewRect); // forces projection matrix update
		
	SetLighting (4);
// 	BuildGeometry (pContextInfo->surface, pContextInfo->colorScheme, pContextInfo->subdivisions, pContextInfo->xyRatio,
// 				   &(pContextInfo->polyList), &(pContextInfo->lineList), &(pContextInfo->pointList));
}
