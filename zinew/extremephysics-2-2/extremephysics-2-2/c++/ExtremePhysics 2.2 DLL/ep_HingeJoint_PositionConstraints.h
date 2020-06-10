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
 * File: ep_HingeJoint_PositionConstraints.h
 * Solves hinge joint position constraints.
 * Included in ep_World_SolvePositionConstraints.cpp.
 */

#ifndef EP_HINGEJOINT_POSITIONCONSTRAINTS_H
#define EP_HINGEJOINT_POSITIONCONSTRAINTS_H

#include "ExtremePhysics.h"

EP_FORCEINLINE void ep_HingeJoint::InitPositionConstraints() {
	
	pseudoxforce = xforce;
	pseudoyforce = yforce;
	pseudolimittorque = limittorque;
	
	if(limit1_maxtorque!=0.0 || limit2_maxtorque!=0.0) {
		motormass = world->position_factor/(body1->invinertia+body2->invinertia);
	}
	
}

EP_FORCEINLINE void ep_HingeJoint::SolvePositionConstraints() {
	
	// limits
	// The motor mass never changes.
	double rot = body1->rot-body2->rot-referencerotation;
	if(maxlimittorque!=0.0) {
		double f = (limit1_rot-rot)*motormass;
		if(pseudolimittorque+f<minlimittorque) {
			f = minlimittorque-pseudolimittorque;
			pseudolimittorque = minlimittorque;
		} else if(pseudolimittorque+f>maxlimittorque) {
			f = maxlimittorque-pseudolimittorque;
			pseudolimittorque = maxlimittorque;
		} else {
			pseudolimittorque += f;
		}
		body1->_ApplyPseudoTorque(+f);
		body2->_ApplyPseudoTorque(-f);
	} else if(minlimittorque!=0.0) {
		double f = (limit2_rot-rot)*motormass;
		if(pseudolimittorque+f<minlimittorque) {
			f = minlimittorque-pseudolimittorque;
			pseudolimittorque = minlimittorque;
		} else if(pseudolimittorque+f>maxlimittorque) {
			f = maxlimittorque-pseudolimittorque;
			pseudolimittorque = maxlimittorque;
		} else {
			pseudolimittorque += f;
		}
		body1->_ApplyPseudoTorque(+f);
		body2->_ApplyPseudoTorque(-f);
	}
	
	// update relative coordinates
	double current_xx1, current_yy1, current_xx2, current_yy2;
	current_xx1 = ep_transform_x(body1->rot_sin, body1->rot_cos, x1-body1->xoff, y1-body1->yoff);
	current_yy1 = ep_transform_y(body1->rot_sin, body1->rot_cos, x1-body1->xoff, y1-body1->yoff);
	current_xx2 = ep_transform_x(body2->rot_sin, body2->rot_cos, x2-body2->xoff, y2-body2->yoff);
	current_yy2 = ep_transform_y(body2->rot_sin, body2->rot_cos, x2-body2->xoff, y2-body2->yoff);
	
	// calculate pseudoforce
	double fx, fy;
	{
		double aa, bb, dd;
		aa = body1->invmass+body2->invmass
		  +body1->invinertia*current_yy1*current_yy1
		  +body2->invinertia*current_yy2*current_yy2;
		bb =
		  -body1->invinertia*current_yy1*current_xx1
		  -body2->invinertia*current_yy2*current_xx2;
		dd = body1->invmass+body2->invmass
		  +body1->invinertia*current_xx1*current_xx1
		  +body2->invinertia*current_xx2*current_xx2;
		double d = world->position_factor/(aa*dd-bb*bb);
		double xx = (body2->x+current_xx2)-(body1->x+current_xx1);
		double yy = (body2->y+current_yy2)-(body1->y+current_yy1);
		fx = d*(xx*dd-yy*bb);
		fy = d*(aa*yy-bb*xx);
	}
	
	// clamp pseudoforce
	if(maxforce==0.0) {
		pseudoxforce += fx;
		pseudoyforce += fy;
	} else {
		double fxx = pseudoxforce+fx;
		double fyy = pseudoyforce+fy;
		double d = ep_sqr(fxx)+ep_sqr(fyy);
		if(d>ep_sqr(maxforce)) {
			d = sqrt(d);
			fxx *= maxforce/d;
			fyy *= maxforce/d;
			fx = fxx-pseudoxforce;
			fy = fyy-pseudoyforce;
			pseudoxforce = fxx;
			pseudoyforce = fyy;
		} else {
			pseudoxforce += fx;
			pseudoyforce += fy;
		}
	}
	
	// apply pseudoforce
	body1->_ApplyPseudoImpulse(current_xx1, current_yy1, +fx, +fy);
	body2->_ApplyPseudoImpulse(current_xx2, current_yy2, -fx, -fy);
	
}

#endif // EP_HINGEJOINT_POSITIONCONSTRAINTS_H

