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
 * File: ep_HingeJoint_IntegrateVelocity.h
 * Applies forces every step for hinge joints.
 * Included in ep_World_IntegrateVelocity.cpp.
 */

#ifndef EP_HINGEJOINT_INTEGRATEVELOCITY_H
#define EP_HINGEJOINT_INTEGRATEVELOCITY_H

#include "ExtremePhysics.h"

EP_FORCEINLINE void ep_HingeJoint::IntegrateVelocity() {
	
	// update relative coordinates
	xx1 = ep_transform_x(body1->rot_sin, body1->rot_cos, x1-body1->xoff, y1-body1->yoff);
	yy1 = ep_transform_y(body1->rot_sin, body1->rot_cos, x1-body1->xoff, y1-body1->yoff);
	xx2 = ep_transform_x(body2->rot_sin, body2->rot_cos, x2-body2->xoff, y2-body2->yoff);
	yy2 = ep_transform_y(body2->rot_sin, body2->rot_cos, x2-body2->xoff, y2-body2->yoff);
	
	// baumgarte
	targetvel_x = -world->baumgarte_factor/world->timestep*((body1->x+xx1)-(body2->x+xx2));
	targetvel_y = -world->baumgarte_factor/world->timestep*((body1->y+yy1)-(body2->y+yy2));
	
	// calculate motor/limit mass
	if(maxmotortorque!=0.0 || limit1_maxtorque!=0.0 || limit2_maxtorque!=0.0 || spring1_k!=0.0 || spring2_k!=0.0) {
		motormass = (1.0+world->mass_bias)/(body1->invinertia+body2->invinertia);
	}
	
	double rotation = body1->rot-body2->rot-referencerotation;
	double f = 0.0;
	if(spring1_k!=0.0 && rotation<spring1_rotation) {
		//double k = ep_min(motormass*2.0, spring1_k*world->timestep); // if k>m*2 the system becomes unstable
		f -= (rotation-spring1_rotation)*spring1_k*world->timestep;
		double currrotvel = body1->rotvel-body2->rotvel;
		f -= currrotvel*spring1_damping*world->timestep;
	}
	if(spring2_k!=0.0 && rotation>=spring2_rotation) {
		//double k = ep_min(motormass*2.0, spring2_k*world->timestep); // if k>m*2 the system becomes unstable
		f -= (rotation-spring2_rotation)*spring2_k*world->timestep;
		double currrotvel = body1->rotvel-body2->rotvel;
		f -= currrotvel*spring2_damping*world->timestep;
	}
	
	if(f!=0.0) {
		body1->_ApplyTorque(+f);
		body2->_ApplyTorque(-f);
	}
	
}

#endif // EP_HINGEJOINT_INTEGRATEVELOCITY_H

