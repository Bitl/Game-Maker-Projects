/*
 * Copyright 2009-2011 Maarten Baert
 * maarten-baert@hotmail.com
 * http://www.maartenbaert.be/
 * 
 * This file is part of ExtremePhysics.
 * 
 * ExtremePhysics is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * ExtremePhysics is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with ExtremePhysics. If not, see <http://www.gnu.org/licenses/>.
 * 
 * File: gm.cpp
 * Initialization of ep_Main and implementation of ep_Callback.
 */

#include "gm.h"

#define WIN32_LEAN_AND_MEAN
#define NOGDI
#define WINVER          0x0500
#define _WIN32_WINNT    0x0500
#define _WIN32_WINDOWS  0x0500
#define _WIN32_IE       0x0500
#include <windows.h>

class DisableFPExceptions {
	public:
	DisableFPExceptions() {
		_control87(_EM_INVALID|_EM_DENORMAL|_EM_ZERODIVIDE|_EM_OVERFLOW|_EM_UNDERFLOW|_EM_INEXACT, _MCW_EM);
		/*
		This will disable floating point exceptions.
		(it disables invalid, NaN, denormal, divide-by-zero, overflow, underflow and inexact)
		These exceptions are disabled by default in C++, but apparently at least some of them are enabled in GM.
		I'm not entirely sure whether changing this has any side effects, but I have no choice. Even if I use epsilons
		everywhere I can still get exceptions in some cases. For example:
			double a, b, c;
			if(f) { a = 1.0; }
			if(g) { b = 2.0; }
			if(f && g) { c = a+b; }
		If you compile this, the compiler will convert it to something like this:
			if(f) { load 1.0; } else { load [address of a]; }
			if(g) { load 2.0; } else { load [address of b]; }
			if(f && g) {
				add; store [address of c];
			} else {
				pop; pop;
			}
		This means that if f or g is false, the compiler will load a random uninitialized value from the stack and use it instead.
		If this random value is NaN or infinity, this will result in an exception. I've actually seen this happen in InitVelocityConstraints!
		*/
	}
} disable_fp_exceptions;

__gmcallback gmcallback;
ep_Main epmain(&gmcallback);

__gmcallback::__gmcallback() {
	showerrors = true;
	logfile = NULL;
	returnstring = NULL;
	translation_x = 0.0;
	translation_y = 0.0;
	rotation = 0.0;
	scale = 1.0;
	m[0][0] = 1.0;
	m[0][1] = 0.0;
	m[1][0] = 0.0;
	m[1][1] = 1.0;
}

__gmcallback::~__gmcallback() {
	free(logfile);
	free(returnstring);
}

char* __gmcallback::MakeReturnString(unsigned long length) {
	free(returnstring);
	returnstring = (char*)(malloc(length));
	return returnstring;
}

void __gmcallback::DebugMessage(int level, const char* string) throw() {
	if(logfile!=NULL) {
		FILE *f;
#ifdef _MSC_VER
		// VC++ complains about deprecation
		fopen_s(&f, logfile, "a");
#else
		f = fopen(logfile, "a");
#endif
		if(f!=NULL) {
			fprintf(f, "[%d] %s\n", level, string);
			fclose(f);
		}
	}
	if(level==1 && showerrors) {
		char messagebuffer[EP_MESSAGE_BUFFERSIZE+100+1];
#ifdef _MSC_VER
		// VC++ complains about deprecation
		_snprintf_s(messagebuffer, EP_MESSAGE_BUFFERSIZE+100+1, EP_MESSAGE_BUFFERSIZE+100, "ExtremePhysics error:\n\n%s\n\n(Press cancel to disable ExtremePhysics error messages)", string);
#else
		snprintf(messagebuffer, EP_MESSAGE_BUFFERSIZE, "ExtremePhysics error:\n\n%s\n\n(Press cancel to disable ExtremePhysics error messages)", string);
#endif
		messagebuffer[EP_MESSAGE_BUFFERSIZE+100] = '\0';
		if(MessageBoxA(NULL, messagebuffer, "ExtremePhysics error", MB_ICONERROR|MB_OKCANCEL)==IDCANCEL) {
			showerrors = false;
		}
	}
}

void __gmcallback::DrawBody(bool isstatic, bool issleeping, double x, double y) {
	double xx = x*m[0][0]+y*m[0][1]+translation_x;
	double yy = x*m[1][0]+y*m[1][1]+translation_y;
	GM_draw_set_color((isstatic)? d1 : d2);
	if(isstatic) {
		GM_draw_rectangle(xx-4.0, yy-4.0, xx+4.0, yy+4.0, GM_TRUE);
		GM_draw_line(xx-4.0, yy-4.0, xx+4.0, yy+4.0);
		GM_draw_line(xx+4.0, yy-4.0, xx-4.0, yy+4.0);
	} else {
		GM_draw_rectangle(xx-3.0, yy-3.0, xx+3.0, yy+3.0, GM_TRUE);
		if(issleeping) {
			GM_draw_rectangle(xx-5.0, yy-5.0, xx+5.0, yy+5.0, GM_TRUE);
		}
	}
}

void __gmcallback::DrawSegment(double x1, double y1, double x2, double y2) {
	double xx1 = x1*m[0][0]+y1*m[0][1]+translation_x;
	double yy1 = x1*m[1][0]+y1*m[1][1]+translation_y;
	double xx2 = x2*m[0][0]+y2*m[0][1]+translation_x;
	double yy2 = x2*m[1][0]+y2*m[1][1]+translation_y;
	GM_draw_line(xx1, yy1, xx2, yy2);
}

void __gmcallback::DrawCircle(double x, double y, double r) {
	double xx = x*m[0][0]+y*m[0][1]+translation_x;
	double yy = x*m[1][0]+y*m[1][1]+translation_y;
	GM_draw_circle(xx, yy, r*scale, GM_TRUE);
}

void __gmcallback::DrawShapeLink(bool isstatic, double x1, double y1, double x2, double y2) {
	if(!isstatic) {
		double xx1 = x1*m[0][0]+y1*m[0][1]+translation_x;
		double yy1 = x1*m[1][0]+y1*m[1][1]+translation_y;
		double xx2 = x2*m[0][0]+y2*m[0][1]+translation_x;
		double yy2 = x2*m[1][0]+y2*m[1][1]+translation_y;
		GM_draw_set_color(d1);
		GM_draw_line_width(xx1, yy1, xx2, yy2, 2.0);
	}
}

void __gmcallback::DrawContactLink(bool isstatic, double x1, double y1, double x2, double y2) {
	if(!isstatic) {
		double xx1 = x1*m[0][0]+y1*m[0][1]+translation_x;
		double yy1 = x1*m[1][0]+y1*m[1][1]+translation_y;
		double xx2 = x2*m[0][0]+y2*m[0][1]+translation_x;
		double yy2 = x2*m[1][0]+y2*m[1][1]+translation_y;
		GM_draw_set_color(d2);
		GM_draw_line_width(xx1, yy1, xx2, yy2, 2.0);
	}
}

void __gmcallback::DrawJointLink(bool isstatic, double x1, double y1, double x2, double y2) {
	if(!isstatic) {
		double xx1 = x1*m[0][0]+y1*m[0][1]+translation_x;
		double yy1 = x1*m[1][0]+y1*m[1][1]+translation_y;
		double xx2 = x2*m[0][0]+y2*m[0][1]+translation_x;
		double yy2 = x2*m[1][0]+y2*m[1][1]+translation_y;
		GM_draw_set_color(d3);
		GM_draw_line_width(xx1, yy1, xx2, yy2, 2.0);
	}
}

void __gmcallback::DrawView(double x1, double y1, double x2, double y2) {
	double xx1 = x1*m[0][0]+y1*m[0][1]+translation_x;
	double yy1 = x1*m[1][0]+y1*m[1][1]+translation_y;
	double xx2 = x2*m[0][0]+y1*m[0][1]+translation_x;
	double yy2 = x2*m[1][0]+y1*m[1][1]+translation_y;
	double xx3 = x2*m[0][0]+y2*m[0][1]+translation_x;
	double yy3 = x2*m[1][0]+y2*m[1][1]+translation_y;
	double xx4 = x1*m[0][0]+y2*m[0][1]+translation_x;
	double yy4 = x1*m[1][0]+y2*m[1][1]+translation_y;
	GM_draw_line(xx1, yy1, xx2, yy2);
	GM_draw_line(xx2, yy2, xx3, yy3);
	GM_draw_line(xx3, yy3, xx4, yy4);
	GM_draw_line(xx4, yy4, xx1, yy1);
}

void __gmcallback::DrawVelocity(double x, double y, double xvel, double yvel, double rotvel) {
	double xx = x*m[0][0]+y*m[0][1]+translation_x;
	double yy = x*m[1][0]+y*m[1][1]+translation_y;
	if(xvel!=0.0 || yvel!=0.0) {
		double vx = (x+xvel*d1)*m[0][0]+(y+yvel*d1)*m[0][1]+translation_x;
		double vy = (x+xvel*d1)*m[1][0]+(y+yvel*d1)*m[1][1]+translation_y;
		GM_draw_arrow(xx, yy, vx, vy, 5.0);
	}
	if(rotvel!=0.0) {
		GM_draw_circle(xx, yy, rotvel*d2*scale, GM_TRUE);
	}
}

void __gmcallback::DrawForce(double x, double y, double xforce, double yforce, double torque) {
	double xx = x*m[0][0]+y*m[0][1]+translation_x;
	double yy = x*m[1][0]+y*m[1][1]+translation_y;
	if(xforce!=0.0 || yforce!=0.0) {
		double vx = (x+xforce*d1)*m[0][0]+(y+yforce*d1)*m[0][1]+translation_x;
		double vy = (x+xforce*d1)*m[1][0]+(y+yforce*d1)*m[1][1]+translation_y;
		GM_draw_arrow(xx, yy, vx, vy, 5.0);
	}
	if(torque!=0.0) {
		GM_draw_circle(xx, yy, torque*d2*scale, GM_TRUE);
	}
}

void __gmcallback::DrawContactPoint(double x1, double y1, double x2, double y2) {
	double xx1 = x1*m[0][0]+y1*m[0][1]+translation_x;
	double yy1 = x1*m[1][0]+y1*m[1][1]+translation_y;
	double xx2 = x2*m[0][0]+y2*m[0][1]+translation_x;
	double yy2 = x2*m[1][0]+y2*m[1][1]+translation_y;
	GM_draw_set_color(d1);
	GM_draw_line(xx1, yy1, xx2, yy2);
	GM_draw_rectangle(xx1-1.5, yy1-1.5, xx1+1.5, yy1+1.5, GM_FALSE);
	GM_draw_rectangle(xx2-1.5, yy2-1.5, xx2+1.5, yy2+1.5, GM_FALSE);
}

void __gmcallback::DrawHingeJoint(double x1, double y1, double x2, double y2) {
	double xx1 = x1*m[0][0]+y1*m[0][1]+translation_x;
	double yy1 = x1*m[1][0]+y1*m[1][1]+translation_y;
	double xx2 = x2*m[0][0]+y2*m[0][1]+translation_x;
	double yy2 = x2*m[1][0]+y2*m[1][1]+translation_y;
	GM_draw_set_color(d2);
	GM_draw_line(xx1, yy1, xx2, yy2);
	GM_draw_rectangle(xx1-1.5, yy1-1.5, xx1+1.5, yy1+1.5, GM_FALSE);
	GM_draw_rectangle(xx2-1.5, yy2-1.5, xx2+1.5, yy2+1.5, GM_FALSE);
}

void __gmcallback::DrawDistanceJoint(double x1, double y1, double x2, double y2) {
	double xx1 = x1*m[0][0]+y1*m[0][1]+translation_x;
	double yy1 = x1*m[1][0]+y1*m[1][1]+translation_y;
	double xx2 = x2*m[0][0]+y2*m[0][1]+translation_x;
	double yy2 = x2*m[1][0]+y2*m[1][1]+translation_y;
	GM_draw_set_color(d2);
	GM_draw_line(xx1, yy1, xx2, yy2);
	GM_draw_rectangle(xx1-1.5, yy1-1.5, xx1+1.5, yy1+1.5, GM_FALSE);
	GM_draw_rectangle(xx2-1.5, yy2-1.5, xx2+1.5, yy2+1.5, GM_FALSE);
}

void __gmcallback::DrawRailJoint(double x1, double y1, double x2a, double y2a, double x2b, double y2b) {
	double xx1 = x1*m[0][0]+y1*m[0][1]+translation_x;
	double yy1 = x1*m[1][0]+y1*m[1][1]+translation_y;
	double xx2a = x2a*m[0][0]+y2a*m[0][1]+translation_x;
	double yy2a = x2a*m[1][0]+y2a*m[1][1]+translation_y;
	double xx2b = x2b*m[0][0]+y2b*m[0][1]+translation_x;
	double yy2b = x2b*m[1][0]+y2b*m[1][1]+translation_y;
	GM_draw_set_color(d2);
	GM_draw_rectangle(xx1-1.5, yy1-1.5, xx1+1.5, yy1+1.5, GM_FALSE);
	GM_draw_line(xx2a, yy2a, xx2b, yy2b);
	GM_draw_rectangle(xx2a-1.5, yy2a-1.5, xx2a+1.5, yy2a+1.5, GM_FALSE);
	GM_draw_rectangle(xx2b-1.5, yy2b-1.5, xx2b+1.5, yy2b+1.5, GM_FALSE);
}

void __gmcallback::DrawSliderJoint(double x1, double y1, double x2a, double y2a, double x2b, double y2b) {
	double xx1 = x1*m[0][0]+y1*m[0][1]+translation_x;
	double yy1 = x1*m[1][0]+y1*m[1][1]+translation_y;
	double xx2a = x2a*m[0][0]+y2a*m[0][1]+translation_x;
	double yy2a = x2a*m[1][0]+y2a*m[1][1]+translation_y;
	double xx2b = x2b*m[0][0]+y2b*m[0][1]+translation_x;
	double yy2b = x2b*m[1][0]+y2b*m[1][1]+translation_y;
	GM_draw_set_color(d2);
	GM_draw_rectangle(xx1-1.5, yy1-1.5, xx1+1.5, yy1+1.5, GM_FALSE);
	GM_draw_line(xx2a, yy2a, xx2b, yy2b);
	GM_draw_rectangle(xx2a-1.5, yy2a-1.5, xx2a+1.5, yy2a+1.5, GM_FALSE);
	GM_draw_rectangle(xx2b-1.5, yy2b-1.5, xx2b+1.5, yy2b+1.5, GM_FALSE);
}

//JOINTS//

