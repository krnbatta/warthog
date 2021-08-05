/*
 *  MapOverlay.cpp
 *  hog
 *
 *  Created by Nathan Sturtevant on 3/21/06.
 *  Copyright 2006 Nathan Sturtevant. All rights reserved.
 *
 */

#include "mapOverlay.h"
#include "fpUtil.h"
#include "glUtil.h"

MapOverlay::MapOverlay(Map *_m)
:m(_m), maxVal(0), minVal(0.0)
{
	values.resize(m->getMapWidth()*m->getMapHeight());
	colorMap = 10;
	displayList = 0;
}

void MapOverlay::resetValues()
{
	if (displayList)
	{
		glDeleteLists(displayList, 1);
		displayList = 0;
	}
	maxVal = DBL_MIN;
	minVal = DBL_MAX;
	for (unsigned int t = 0; t < values.size(); t++)
	{
		if (!fequal(values[t], ignoreVal))
		{
			if (fgreater(values[t], maxVal))
				maxVal = values[t];
			if (fless(values[t], minVal))
				minVal = values[t];
		}
	}
}

void MapOverlay::setOverlayValue(int x, int y, double value)
{
	if ((x < 0) || (x >= m->getMapWidth()) || (y < 0) || (y >= m->getMapHeight()))
		return;
	if (displayList)
	{
		glDeleteLists(displayList, 1);
		displayList = 0;
	}
	
	values[y*m->getMapWidth()+x] = value;
	if (value > maxVal)
		maxVal = value;
	if (value < minVal)
		minVal = value;
	//resetValues();
}

double MapOverlay::getOverlayValue(int x, int y)
{
	if ((x < 0) || (x >= m->getMapWidth()) || (y < 0) || (y >= m->getMapHeight()))
		return 0;
	return values[y*m->getMapWidth()+x];
}

void MapOverlay::openGLDraw()
{
	if (displayList)
	{
		glCallList(displayList);
	}
	else {
		displayList = glGenLists(1);
		glNewList(displayList, GL_COMPILE_AND_EXECUTE);
				
		glBegin(GL_QUADS);
		glNormal3f(0, 0, -1);
		for (unsigned int t = 0; t < values.size(); t++)
		{
			if (fequal(values[t], ignoreVal))
			{
				//glColor4f(0.5, 0.5, 0.5, 0.5);
				continue;
			}
			else {
				recColor r = getColor(values[t], minVal, maxVal, colorMap);
				glColor3f(r.r, r.g, r.b);
			}
			unsigned int last;
			for (last = t+1;
					 last < values.size() && (last%(m->getMapWidth()) != 0) &&
					 fequal(values[t], values[last]); last++)
			{}
			last -= 1;
			GLdouble coverage = 1.0;
			GLdouble a, b, c, radius;
			m->getOpenGLCoord(t%(m->getMapWidth()), t/m->getMapWidth(), a, b, c, radius);
			glVertex3f(a-coverage*radius, b+coverage*radius, c-4*radius);
			glVertex3f(a-coverage*radius, b-coverage*radius, c-4*radius);
			m->getOpenGLCoord(last%(m->getMapWidth()), last/m->getMapWidth(), a, b, c, radius);
			glVertex3f(a+coverage*radius, b-coverage*radius, c-4*radius);
			glVertex3f(a+coverage*radius, b+coverage*radius, c-4*radius);
			t = last;
		}
		glEnd();

		// black background
		glBegin(GL_QUADS);
		glNormal3f(0, 0, -1);
		GLdouble a, b, c, radius;
		m->getOpenGLCoord(0, 0, a, b, c, radius);
		double wide = radius*2.0*m->getMapWidth()/100.0;
		glColor3f(0.2, 0.2, 0.2);
		glVertex3f(a-wide, b+2*m->getMapHeight()*radius+wide, c-4*radius);
		glVertex3f(a-4*wide, b+2*m->getMapHeight()*radius+wide, c-4*radius);
		glVertex3f(a-4*wide, b-wide, c-4*radius);
		glVertex3f(a-wide, b-wide, c-4*radius);
		glEnd();

		// white border
		glBegin(GL_LINE_LOOP);
		glColor3f(1.0, 1.0, 1.0);
		glVertex3f(a-wide, b+2*m->getMapHeight()*radius+wide, c-4*radius-radius/10.0);
		glVertex3f(a-4*wide, b+2*m->getMapHeight()*radius+wide, c-4*radius-radius/10.0);
		glVertex3f(a-4*wide, b-wide, c-4*radius);
		glVertex3f(a-wide, b-wide, c-4*radius);
		glEnd();

		// color bar on the side
		glBegin(GL_QUAD_STRIP);
		glNormal3f(0, 0, -1);
		m->getOpenGLCoord(0, 0, a, b, c, radius);
		const int stripResolution = 20;
		for (int x = 0; x <= stripResolution; x++)
		{
			recColor r = getColor(minVal + (maxVal-minVal)*((double)x/stripResolution),
														minVal, maxVal, colorMap);
			glColor3f(r.r, r.g, r.b);
			glVertex3f(a-2*wide, b+2*m->getMapHeight()*radius*(1.0-(double)x/stripResolution), c-4*radius-radius/8.0);
			glVertex3f(a-3*wide, b+2*m->getMapHeight()*radius*(1.0-(double)x/stripResolution), c-4*radius-radius/8.0);
		}
		glEnd();

		if (maxVal-minVal != 0)
		{
			double mult = 1.0;
			double range = 1.0;
			while (1)
			{
				if (!(fless((maxVal-minVal)*mult, 10.0)))
					mult /= 10;
				else if (fless((maxVal-minVal)*mult, 1.0))
					mult *= 10;
				else {
					if ((maxVal-minVal)*mult >= 5)
						range = 1.0;
					else if ((maxVal-minVal)*mult >= 3)
						range = 0.5;
					else
						range = 0.1;
					break;
				}
			}
			glBegin(GL_LINES);
			glColor3f(1.0, 1.0, 1.0);
			//double top = (double)((int)(maxVal*mult))/mult;
			double bottom = (double)((int)(minVal*mult))/mult;
//			printf("min/max %f, %f, stepped min/max %f, %f\n",
//						 minVal, maxVal, bottom, top);
//			printf("Range: %f, mult %f\n", range, mult);
			for (double x = bottom; !fgreater(x, maxVal); x+=range/mult)
			{
				if (fequal(x, 0))
					glColor3f(1.0, 0, 0);
				glVertex3f(a-1*wide, b+2*m->getMapHeight()*radius*(1.0-(x-minVal)/(maxVal-minVal)), c-4*radius-radius/9.0);
				glVertex3f(a-4*wide, b+2*m->getMapHeight()*radius*(1.0-(x-minVal)/(maxVal-minVal)), c-4*radius-radius/9.0);
				if (fequal(x, 0))
					glColor3f(1.0, 1.0, 1.0);
			}
			for (double x = bottom-range/mult; !fless(x, minVal); x-=range/mult)
			{
				if (fequal(x, 0))
					glColor3f(1.0, 0, 0);
				glVertex3f(a-1*wide, b+2*m->getMapHeight()*radius*(1.0-(x-minVal)/(maxVal-minVal)), c-4*radius-radius/9.0);
				glVertex3f(a-4*wide, b+2*m->getMapHeight()*radius*(1.0-(x-minVal)/(maxVal-minVal)), c-4*radius-radius/9.0);
				if (fequal(x, 0))
					glColor3f(1.0, 1.0, 1.0);
			}
			glEnd();
		}
		
		glEndList();
	}
}
