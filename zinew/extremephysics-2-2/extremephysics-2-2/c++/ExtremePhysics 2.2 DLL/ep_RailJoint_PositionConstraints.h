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
 * File: ep_RailJoint_PositionConstraints.h
 * Solves rail joint position constraints.
 * Included in ep_World_SolvePositionConstraints.cpp.
 */

#ifndef EP_RAILJOINT_POSITIONCONSTRAINTS_H
#define EP_RAILJOINT_POSITIONCONSTRAINTS_H

#include "ExtremePhysics.h"

EP_FORCEINLINE void ep_RailJoint::InitPositionConstraints() {
	
	pseudonormalforce = normalforce;
	pseudolimitforce = limitforce;
	
}

EP_FORCEINLINE void ep_RailJoint::SolvePositionConstraints() {
	
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
		} else if(minlimitforce!=0.0) {
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
	
	// normal constraint
	double r1 = current_yy1*current_vyy+current_xx1*current_vxx;
	double r2 = current_yy2*current_vyy+current_xx2*current_vxx;
	double f = (current_vxx*railpointy-current_vyy*railpointx)*world->position_factor/(
		body1->invmass+body2->invmass
		+ep_sqr(r1)*body1->invinertia
		+ep_sqr(r2)*body2->invinertia);
	
	// clamp pseudoforce
	if(maxnormalforce==0.0) {
		pseudonormalforce += f;
	} else {
		if(f<-maxnormalforce-pseudonormalforce) {
			f = -maxnormalforce-pseudonormalforce;
			pseudonormalforce = -maxnormalforce;
		} else if(f>+maxnormalforce-pseudonormalforce) {
			f = +maxnormalforce-pseudonormalforce;
			pseudonormalforce = +maxnormalforce;
		} else {
			pseudonormalforce += f;
		}
	}
	
	// apply pseudoforce
	double fx = +f*current_vyy;
	double fy = -f*current_vxx;
	body1->_ApplyPseudoImpulse(current_xx1, current_yy1, +fx, +fy);
	body2->_ApplyPseudoImpulse(current_xx2, current_yy2, -fx, -fy);
	
}

#endif // EP_RAILJOINT_POSITIONCONSTRAINTS_H

