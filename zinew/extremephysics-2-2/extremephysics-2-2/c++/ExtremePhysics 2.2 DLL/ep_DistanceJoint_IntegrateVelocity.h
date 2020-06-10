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
 * File: ep_DistanceJoint_IntegrateVelocity.h
 * Applies forces every step for hinge joints.
 * Included in ep_World_IntegrateVelocity.cpp.
 */

#ifndef EP_DISTANCEJOINT_INTEGRATEVELOCITY_H
#define EP_DISTANCEJOINT_INTEGRATEVELOCITY_H

#include "ExtremePhysics.h"

EP_FORCEINLINE void ep_DistanceJoint::IntegrateVelocity() {
	
	// update relative coordinates
	xx1 = ep_transform_x(body1->rot_sin, body1->rot_cos, x1-body1->xoff, y1-body1->yoff);
	yy1 = ep_transform_y(body1->rot_sin, body1->rot_cos, x1-body1->xoff, y1-body1->yoff);
	xx2 = ep_transform_x(body2->rot_sin, body2->rot_cos, x2-body2->xoff, y2-body2->yoff);
	yy2 = ep_transform_y(body2->rot_sin, body2->rot_cos, x2-body2->xoff, y2-body2->yoff);
	
	// update vector and distance
	vx = ((body1->x+xx1)-(body2->x+xx2));
	vy = ((body1->y+yy1)-(body2->y+yy2));
	distance = sqrt(ep_sqr(vx)+ep_sqr(vy));
	if(distance<EP_EPSILON_MINNORMALVECTOR) {
		vx = 1.0;
		vy = 0.0;
	} else {
		vx /= distance;
		vy /= distance;
	}
	
	// calculate motor mass
	// Uses the same derivation as contact constraints.
	double r1 = yy1*vx-xx1*vy;
	double r2 = yy2*vx-xx2*vy;
	motormass = (1.0+world->mass_bias)/(
	  body1->invmass+body2->invmass
	  +ep_sqr(r1)*body1->invinertia
	  +ep_sqr(r2)*body2->invinertia);
	
	double f = 0.0;
	if(spring1_k!=0.0 && distance<spring1_distance) {
		//TODO// remove, doesn't work correctly if timestep!=1.0
		//double k = ep_min(motormass*2.0, spring1_k*world->timestep); // if k>m*2 the system becomes unstable
		f -= (distance-spring1_distance)*spring1_k*world->timestep;
		double currdistancevel =
		   ((body1->xvel+yy1*body1->rotvel)-(body2->xvel+yy2*body2->rotvel))*vx
		  +((body1->yvel-xx1*body1->rotvel)-(body2->yvel-xx2*body2->rotvel))*vy;
		f -= currdistancevel*spring1_damping*world->timestep;
	}
	if(spring2_k!=0.0 && distance>=spring2_distance) {
		//TODO// remove, doesn't work correctly if timestep!=1.0
		//double k = ep_min(motormass*2.0, spring2_k*world->timestep); // if k>m*2 the system becomes unstable
		f -= (distance-spring2_distance)*spring2_k*world->timestep;
		double currdistancevel =
		   ((body1->xvel+yy1*body1->rotvel)-(body2->xvel+yy2*body2->rotvel))*vx
		  +((body1->yvel-xx1*body1->rotvel)-(body2->yvel-xx2*body2->rotvel))*vy;
		f -= currdistancevel*spring2_damping*world->timestep;
	}
	
	if(f!=0.0) {
		double fx = f*vx;
		double fy = f*vy;
		body1->_ApplyImpulse(xx1, yy1, +fx, +fy);
		body2->_ApplyImpulse(xx2, yy2, -fx, -fy);
	}
	
}

#endif // EP_DISTANCEJOINT_INTEGRATEVELOCITY_H

