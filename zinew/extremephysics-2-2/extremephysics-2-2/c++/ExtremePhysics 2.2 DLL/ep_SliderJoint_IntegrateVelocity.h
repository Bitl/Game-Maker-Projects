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
 * File: ep_SliderJoint_IntegrateVelocity.h
 * Applies forces every step for slider joints.
 * Included in ep_World_IntegrateVelocity.cpp.
 */

#ifndef EP_SLIDERJOINT_INTEGRATEVELOCITY_H
#define EP_SLIDERJOINT_INTEGRATEVELOCITY_H

#include "ExtremePhysics.h"

EP_FORCEINLINE void ep_SliderJoint::IntegrateVelocity() {
	
	// update relative coordinates
	xx1 = ep_transform_x(body1->rot_sin, body1->rot_cos, x1-body1->xoff, y1-body1->yoff);
	yy1 = ep_transform_y(body1->rot_sin, body1->rot_cos, x1-body1->xoff, y1-body1->yoff);
	xx2 = ep_transform_x(body2->rot_sin, body2->rot_cos, x2-body2->xoff, y2-body2->yoff);
	yy2 = ep_transform_y(body2->rot_sin, body2->rot_cos, x2-body2->xoff, y2-body2->yoff);
	vxx = ep_transform_x(body2->rot_sin, body2->rot_cos, vx, vy);
	vyy = ep_transform_y(body2->rot_sin, body2->rot_cos, vx, vy);
	{
		double railpointx, railpointy;
		railpointx = body1->x-body2->x+xx1-xx2;
		railpointy = body1->y-body2->y+yy1-yy2;
		double sep = vyy*railpointx-vxx*railpointy;
		position = vxx*railpointx+vyy*railpointy;
		xx2 += vxx*position;
		yy2 += vyy*position;
		// baumgarte
		targetnormalvel = -world->baumgarte_factor/world->timestep*sep;
		targetrotvel = world->baumgarte_factor/world->timestep*(rotation-(body1->rot-body2->rot));
	}
	
	// calculate mass
	// Uses the same derivation as contact constraints.
	{
		double r1 = yy1*vxx-xx1*vyy;
		double r2 = yy2*vxx-xx2*vyy;
		motormass = (1.0+world->mass_bias)/(
		  body1->invmass+body2->invmass
		  +ep_sqr(r1)*body1->invinertia
		  +ep_sqr(r2)*body2->invinertia);
	}
	
	// springs
	double f = 0.0;
	if(spring1_k!=0.0 && position<spring1_position) {
		//double k = ep_min(motormass*2.0, spring1_k*world->timestep); // if k>m*2 the system becomes unstable
		f -= (position-spring1_position)*spring1_k*world->timestep;
		double currvel =
		   ((body1->xvel+yy1*body1->rotvel)-(body2->xvel+yy2*body2->rotvel))*vxx
		  +((body1->yvel-xx1*body1->rotvel)-(body2->yvel-xx2*body2->rotvel))*vyy;
		f -= currvel*spring1_damping*world->timestep;
	}
	if(spring2_k!=0.0 && position>=spring2_position) {
		//double k = ep_min(motormass*2.0, spring2_k*world->timestep); // if k>m*2 the system becomes unstable
		f -= (position-spring2_position)*spring2_k*world->timestep;
		double currvel =
		   ((body1->xvel+yy1*body1->rotvel)-(body2->xvel+yy2*body2->rotvel))*vxx
		  +((body1->yvel-xx1*body1->rotvel)-(body2->yvel-xx2*body2->rotvel))*vyy;
		f -= currvel*spring2_damping*world->timestep;
	}
	if(f!=0.0) {
		double fx = f*vxx;
		double fy = f*vyy;
		body1->_ApplyImpulse(xx1, yy1, +fx, +fy);
		body2->_ApplyImpulse(xx2, yy2, -fx, -fy);
	}
	
}

#endif // EP_SLIDERJOINT_INTEGRATEVELOCITY_H

