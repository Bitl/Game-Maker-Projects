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
 * File: ep_SliderJoint_PositionConstraints.h
 * Solves slider joint position constraints.
 * Included in ep_World_SolvePositionConstraints.cpp.
 */

#ifndef EP_SLIDERJOINT_POSITIONCONSTRAINTS_H
#define EP_SLIDERJOINT_POSITIONCONSTRAINTS_H

#include "ExtremePhysics.h"

EP_FORCEINLINE void ep_SliderJoint::InitPositionConstraints() {
	
	pseudonormalforce = normalforce;
	pseudotorque = torque;
	pseudolimitforce = limitforce;
	
}

EP_FORCEINLINE void ep_SliderJoint::SolvePositionConstraints() {
	
	// limits
	if(minlimitforce!=maxlimitforce) {
		
		// update relative coordinates
		double current_xx1, current_yy1, current_xx2, current_yy2, current_vxx, current_vyy;
		current_xx1 = ep_transform_x(body1->rot_sin, body1->rot_cos, x1-body1->xoff, y1-body1->yoff);
		current_yy1 = ep_transform_y(body1->rot_sin, body1->rot_cos, x1-body1->xoff, y1-body1->yoff);
		current_xx2 = ep_transform_x(body2->rot_sin, body2->rot_cos, x2-body2->xoff, y2-body2->yoff);
		current_yy2 = ep_transform_y(body2->rot_sin, body2->rot_cos, x2-body2->xoff, y2-body2->yoff);
		current_vxx = ep_transform_x(body2->rot_sin, body2->rot_cos, vx, vy);
		current_vyy = ep_transform_y(body2->rot_sin, body2->rot_cos, vx, vy);
		double railpointx, railpointy, current_position;
		railpointx = body1->x-body2->x+current_xx1-current_xx2;
		railpointy = body1->y-body2->y+current_yy1-current_yy2;
		current_position = current_vxx*railpointx+current_vyy*railpointy;
		current_xx2 += current_vxx*current_position;
		current_yy2 += current_vyy*current_position;
		
		double f;
		if(maxlimitforce!=0.0) {
			double r1 = current_yy1*current_vxx-current_xx1*current_vyy;
			double r2 = current_yy2*current_vxx-current_xx2*current_vyy;
			f = (limit1_position-current_position)*world->position_factor/(
			  body1->invmass+body2->invmass
			  +ep_sqr(r1)*body1->invinertia
			  +ep_sqr(r2)*body2->invinertia);
			if(pseudolimitforce+f<minlimitforce) {
				f = minlimitforce-pseudolimitforce;
				pseudolimitforce = minlimitforce;
			} else if(pseudolimitforce+f>maxlimitforce) {
				f = maxlimitforce-pseudolimitforce;
				pseudolimitforce = maxlimitforce;
			} else {
				pseudolimitforce += f;
			}
		} else {
			double r1 = current_yy1*current_vxx-current_xx1*current_vyy;
			double r2 = current_yy2*current_vxx-current_xx2*current_vyy;
			f = (limit2_position-current_position)*world->position_factor/(
			  body1->invmass+body2->invmass
			  +ep_sqr(r1)*body1->invinertia
			  +ep_sqr(r2)*body2->invinertia);
			if(pseudolimitforce+f<minlimitforce) {
				f = minlimitforce-pseudolimitforce;
				pseudolimitforce = minlimitforce;
			} else if(pseudolimitforce+f>maxlimitforce) {
				f = maxlimitforce-pseudolimitforce;
				pseudolimitforce = maxlimitforce;
			} else {
				pseudolimitforce += f;
			}
		}
		double fx = f*current_vxx;
		double fy = f*current_vyy;
		body1->_ApplyPseudoImpulse(current_xx1, current_yy1, +fx, +fy);
		body2->_ApplyPseudoImpulse(current_xx2, current_yy2, -fx, -fy);
		
	}
	
	// update relative coordinates
	double current_xx1, current_yy1, current_xx2, current_yy2, current_vxx, current_vyy;
	current_xx1 = ep_transform_x(body1->rot_sin, body1->rot_cos, x1-body1->xoff, y1-body1->yoff);
	current_yy1 = ep_transform_y(body1->rot_sin, body1->rot_cos, x1-body1->xoff, y1-body1->yoff);
	current_xx2 = ep_transform_x(body2->rot_sin, body2->rot_cos, x2-body2->xoff, y2-body2->yoff);
	current_yy2 = ep_transform_y(body2->rot_sin, body2->rot_cos, x2-body2->xoff, y2-body2->yoff);
	current_vxx = ep_transform_x(body2->rot_sin, body2->rot_cos, vx, vy);
	current_vyy = ep_transform_y(body2->rot_sin, body2->rot_cos, vx, vy);
	double railpointx, railpointy, current_position;
	railpointx = body1->x-body2->x+current_xx1-current_xx2;
	railpointy = body1->y-body2->y+current_yy1-current_yy2;
	current_position = current_vxx*railpointx+current_vyy*railpointy;
	current_xx2 += current_vxx*current_position;
	current_yy2 += current_vyy*current_position;
	
	// calculate pseudoforce
	double f, t;
	{
		double r1 = current_yy1*current_vyy+current_xx1*current_vxx;
		double r2 = current_yy2*current_vyy+current_xx2*current_vxx;
		double aa, bb, dd;
		aa = body1->invmass+body2->invmass
			+body1->invinertia*ep_sqr(r1)
			+body2->invinertia*ep_sqr(r2);
		bb = body1->invinertia*r1
			+body2->invinertia*r2;
		dd = body1->invinertia+body2->invinertia;
		double d = world->position_factor/(aa*dd-bb*bb);
		double nv = current_vxx*railpointy-current_vyy*railpointx;
		double rv = rotation-(body1->rot-body2->rot);
		f = d*(nv*dd-rv*bb);
		t = d*(aa*rv-bb*nv);
	}
	
	// always clamp the accumulated impulse
	if(maxnormalforce==0.0) {
		pseudonormalforce += f;
		pseudotorque += t;
	} else {
		double ff = pseudonormalforce+f;
		double tt = pseudotorque+t;
		double d = (torqueradius==0.0)? fabs(ff) : fabs(ff)+fabs(tt)/torqueradius;
		if(d>maxnormalforce) {
			ff *= maxnormalforce/d;
			tt *= maxnormalforce/d;
			f = ff-pseudonormalforce;
			t = tt-pseudotorque;
			pseudonormalforce = ff;
			pseudotorque = tt;
		} else {
			pseudonormalforce += f;
			pseudotorque += t;
		}
	}
	
	// apply pseudoforce
	double fx = +f*current_vyy;
	double fy = -f*current_vxx;
	body1->_ApplyPseudoImpulseTorque(current_xx1, current_yy1, +fx, +fy, +t);
	body2->_ApplyPseudoImpulseTorque(current_xx2, current_yy2, -fx, -fy, -t);
	
}

#endif // EP_SLIDERJOINT_POSITIONCONSTRAINTS_H

