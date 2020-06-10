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
 * File: ep_Force_IntegrateVelocity.h
 * Applies forces every step.
 * Included in ep_World_IntegrateVelocity.cpp.
 */

#ifndef EP_FORCE_INTEGRATEVELOCITY_H
#define EP_FORCE_POSITIONCONSTRAINTS_H

#include "ExtremePhysics.h"

EP_FORCEINLINE void ep_Force::IntegrateVelocity() {
	if(local) {
		body->xvel += ep_transform_x(body->rot_sin, body->rot_cos, xforce, yforce)*((ignoremass)? 1.0 : body->invmass)*world->timestep;
		body->yvel += ep_transform_y(body->rot_sin, body->rot_cos, xforce, yforce)*((ignoremass)? 1.0 : body->invmass)*world->timestep;
		body->rotvel += ((y-body->yoff)*xforce-(x-body->xoff)*yforce)*body->invinertia*((ignoremass)? body->mass : 1.0)*world->timestep
		  +torque*((ignoremass)? 1.0 : body->invinertia)*world->timestep;
	} else {
		body->xvel += xforce*((ignoremass)? 1.0 : body->invmass)*world->timestep;
		body->yvel += yforce*((ignoremass)? 1.0 : body->invmass)*world->timestep;
		body->rotvel +=
		  ((y-body->yoff)*ep_invtransform_x(body->rot_sin, body->rot_cos, xforce, yforce)
		  -(x-body->xoff)*ep_invtransform_y(body->rot_sin, body->rot_cos, xforce, yforce))*body->invinertia*((ignoremass)? body->mass : 1.0)*world->timestep
		  +torque*((ignoremass)? 1.0 : body->invinertia)*world->timestep;
	}
}

#endif // EP_HINGEJOINT_INTEGRATEVELOCITY_H

