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
 * File: ep_DistanceJoint_PositionConstraints.h
 * Solves distance joint position constraints.
 * Included in ep_World_SolvePositionConstraints.cpp.
 */

#ifndef EP_DISTANCEJOINT_POSITIONCONSTRAINTS_H
#define EP_DISTANCEJOINT_POSITIONCONSTRAINTS_H

#include "ExtremePhysics.h"

EP_FORCEINLINE void ep_DistanceJoint::InitPositionConstraints() {
	
	pseudolimitforce = limitforce;
	
}

EP_FORCEINLINE void ep_DistanceJoint::SolvePositionConstraints() {
	
	// update relative coordinates
	double current_xx1, current_yy1, current_xx2, current_yy2;
	current_xx1 = ep_transform_x(body1->rot_sin, body1->rot_cos, x1-body1->xoff, y1-body1->yoff);
	current_yy1 = ep_transform_y(body1->rot_sin, body1->rot_cos, x1-body1->xoff, y1-body1->yoff);
	current_xx2 = ep_transform_x(body2->rot_sin, body2->rot_cos, x2-body2->xoff, y2-body2->yoff);
	current_yy2 = ep_transform_y(body2->rot_sin, body2->rot_cos, x2-body2->xoff, y2-body2->yoff);
	
	// update vector and distance
	double current_vx, current_vy, current_distance;
	current_vx = ((body1->x+current_xx1)-(body2->x+current_xx2));
	current_vy = ((body1->y+current_yy1)-(body2->y+current_yy2));
	current_distance = sqrt(ep_sqr(current_vx)+ep_sqr(current_vy));
	if(current_distance<EP_EPSILON_MINNORMALVECTOR) {
		current_vx = 1.0;
		current_vy = 0.0;
	} else {
		current_vx /= current_distance;
		current_vy /= current_distance;
	}
	
	// calculate pseudoforce
	double r1 = current_yy1*current_vx-current_xx1*current_vy;
	double r2 = current_yy2*current_vx-current_xx2*current_vy;
	double f = (((maxlimitforce!=0.0)? limit1_distance : limit2_distance)-current_distance)*world->position_factor/(
	  body1->invmass+body2->invmass
	  +ep_sqr(r1)*body1->invinertia
	  +ep_sqr(r2)*body2->invinertia);
	
	// clamp pseudoforce
	if(pseudolimitforce+f<minlimitforce) {
		f = minlimitforce-pseudolimitforce;
		pseudolimitforce = minlimitforce;
	} else if(pseudolimitforce+f>maxlimitforce) {
		f = maxlimitforce-pseudolimitforce;
		pseudolimitforce = maxlimitforce;
	} else {
		pseudolimitforce += f;
	}
	
	// apply pseudoforce
	double fx = current_vx*f;
	double fy = current_vy*f;
	body1->_ApplyPseudoImpulse(current_xx1, current_yy1, +fx, +fy);
	body2->_ApplyPseudoImpulse(current_xx2, current_yy2, -fx, -fy);
	
}

#endif // EP_DISTANCEJOINT_POSITIONCONSTRAINTS_H

