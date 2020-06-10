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
 * File: gm.h
 * Header file for the gm-related part of ExtremePhysics
 */

#ifndef GM_H
#define GM_H

#include "ExtremePhysics.h"
#include "GMaker.h"

#include <cfloat>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>

// number of variables in the userdata struct
#define GM_USERVARS 5

// function exported to Game Maker
#define gmexport extern "C" __declspec (dllexport)

// structs
struct gmuserdata {
	double var[GM_USERVARS];
	inline void Clear() {
		for(int i = 0; i<GM_USERVARS; ++i) {
			var[i] = 0;
		}
	}
};

// gm constants
#define GM_TRUE   1.0
#define GM_FALSE  0.0

// gm_cast - cast without throwing exceptions
template<typename A>
inline A gm_cast(double d) {
	A minval = std::numeric_limits<A>::min();
	A maxval = std::numeric_limits<A>::max();
	if(d<(double)(minval)) {
		return minval;
	}
	if(d>(double)(maxval)) {
		return maxval;
	}
	return (A)(d);
}
template<>
inline bool gm_cast<bool>(double d) {
	return (d>=0.5);
}

/*
template<typename A> A gm_cast(double);
template<> inline unsigned long gm_cast<unsigned long>(double d) {
	if((double)(unsigned long)(0)<=d && d<=(double)(unsigned long)(ULONG_MAX)) return (unsigned long)(d);
	else return 0;
}
template<> inline int gm_cast<int>(double d) {
	if((double)(int)(INT_MIN)<=d && d<=(double)(int)(INT_MAX)) return (int)(d);
	else return 0;
}
template<> inline bool gm_cast<bool>(double d) {
	return (d>=0.5);
}
template<> inline float gm_cast<float>(double d) {
	if((double)(float)(-FLT_MAX)<=d && d<=(double)(float)(FLT_MAX)) return (float)(d);
	else return (float)(0.0);
}
*/

// classes
class __gmcallback : public ep_Callback {
	public:
	double d1, d2, d3;
	double translation_x, translation_y, rotation, scale;
	double m[2][2];
	bool showerrors;
	char *logfile;
	char *returnstring;
	
	public:
	__gmcallback();
#ifdef __GNUC__
	virtual ~__gmcallback();
#else
	~__gmcallback();
#endif
	char* MakeReturnString(unsigned long length);
	virtual void DebugMessage(int level, const char* string) throw();
	virtual void DrawBody(bool isstatic, bool issleeping, double x, double y);
	virtual void DrawSegment(double x1, double y1, double x2, double y2);
	virtual void DrawCircle(double x, double y, double r);
	virtual void DrawShapeLink(bool isstatic, double x1, double y1, double x2, double y2);
	virtual void DrawContactLink(bool isstatic, double x1, double y1, double x2, double y2);
	virtual void DrawJointLink(bool isstatic, double x1, double y1, double x2, double y2);
	virtual void DrawView(double x1, double y1, double x2, double y2);
	virtual void DrawVelocity(double x, double y, double xvel, double yvel, double rotvel);
	virtual void DrawForce(double x, double y, double xforce, double yforce, double torque);
	virtual void DrawContactPoint(double x1, double y1, double x2, double y2);
	virtual void DrawHingeJoint(double x1, double y1, double x2, double y2);
	virtual void DrawDistanceJoint(double x1, double y1, double x2, double y2);
	virtual void DrawRailJoint(double x1, double y1, double x2a, double y2a, double x2b, double y2b);
	virtual void DrawSliderJoint(double x1, double y1, double x2a, double y2a, double x2b, double y2b);
	//JOINTS//
};

// variables
extern __gmcallback gmcallback;
extern ep_Main epmain;

// gm functions
extern bool gmfunctions_initialized;
extern GMProc GM_draw_set_color;   // draw_set_color(color);
extern GMProc GM_draw_circle;      // draw_circle(x, y, r, outline);
extern GMProc GM_draw_line;        // draw_line(x1, y1, x2, y2);
extern GMProc GM_draw_line_width;  // draw_line_width(x1, y1, x2, y2, width);
extern GMProc GM_draw_arrow;       // draw_arrow(x1, y1, x2, y2, size);
extern GMProc GM_draw_rectangle;   // draw_rectangle(x1, y1, x2, y2, outline);

#endif // GM_H

