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
 * File: ep_Macros.h
 * This file contains a list of all macros used by ExtremePhysics.
 * Included in ExtremePhysics.h.
 */

#include "ep_Definitions.h"

#ifndef EP_CALLBACK_H
#define EP_CALLBACK_H

/// ExtremePhysics callback class.
/// Implement this class with your own functions.
/// Whatever you do, DON'T make ANY change to ANY
/// ExtremePhysics object in the callback functions
/// or things will go wrong.
class ep_Callback {
	
	// public functions
	public:
#ifdef __GNUC__
	virtual ~ep_Callback() {}
#endif
	
	virtual void DebugMessage(int level, const char* string) throw() {}
	
	virtual void DrawBody(bool isstatic, bool issleeping, double x, double y) {}
	virtual void DrawSegment(double x1, double y1, double x2, double y2) {}
	virtual void DrawCircle(double x, double y, double r) {}
	
	virtual void DrawShapeLink(bool isstatic, double x1, double y1, double x2, double y2) {}
	virtual void DrawContactLink(bool isstatic, double x1, double y1, double x2, double y2) {}
	virtual void DrawJointLink(bool isstatic, double x1, double y1, double x2, double y2) {}
	
	virtual void DrawView(double x1, double y1, double x2, double y2) {}
	
	virtual void DrawVelocity(double x, double y, double xvel, double yvel, double rotvel) {}
	virtual void DrawForce(double x, double y, double xforce, double yforce, double torque) {}
	
	virtual void DrawContactPoint(double x1, double y1, double x2, double y2) {}
	virtual void DrawHingeJoint(double x1, double y1, double x2, double y2) {}
	virtual void DrawDistanceJoint(double x1, double y1, double x2, double y2) {}
	virtual void DrawRailJoint(double x1, double y1, double x2a, double y2a, double x2b, double y2b) {}
	virtual void DrawSliderJoint(double x1, double y1, double x2a, double y2a, double x2b, double y2b) {}
	//JOINTS//
	
};

#endif // EP_CALLBACK_H

