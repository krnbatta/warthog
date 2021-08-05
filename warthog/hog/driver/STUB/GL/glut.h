#ifndef GLUT_H
#define GLUT_H

#include "gl.h"

#define GLUT_DOUBLE        0
#define GLUT_RGBA          0
#define GLUT_DEPTH         0
#define GLUT_ACTIVE_SHIFT  0
#define GLUT_ACTIVE_ALT    1
#define GLUT_ACTIVE_CTRL   2
#define GLUT_MIDDLE_BUTTON 0
#define GLUT_RIGHT_BUTTON  1
#define GLUT_LEFT_BUTTON   2
#define GLUT_DOWN          0
#define GLUT_WINDOW_WIDTH  0
#define GLUT_WINDOW_HEIGHT 0

void glutAddMenuEntry (char *label, int value) ;
void glutAddSubMenu (char *label, int submenu) ;
void glutAttachMenu (int button) ;
int glutCreateMenu (void (*) (int)) ;
int glutCreateWindow (char *title) ;
void glutDisplayFunc (void (*) (void)) ;
int glutGet (GLenum type) ;
int glutGetModifiers (void) ;
int glutIdleFunc (void (*)(void)) ;
void glutInit (int *argcp, char **argv) ;
void glutInitDisplayMode (unsigned int mode) ;
void glutInitWindowPosition (int x, int y) ;
void glutInitWindowSize (int width, int height) ;
void glutKeyboardFunc (void (*) (unsigned char key, int x, int y)) ;
void glutMainLoop (void);

void glutMotionFunc (void (*) (int x, int y)) ;
void glutMouseFunc (void (*) (int button, int state, int x, int y)) ;
void glutReshapeFunc (void (*) (int width, int height)) ;
void glutSwapBuffers (void) ;
void gluLookAt (GLdouble eyeX, GLdouble eyeY, GLdouble eyeZ, 
		GLdouble centerX, GLdouble centerY, GLdouble centerZ,
		GLdouble upX, GLdouble upY, GLdouble upZ) ;
#endif
