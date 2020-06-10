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
 * File: ep_World_IntegrateVelocity.cpp
 * Integrates the velocity of the bodies in the world.
 */

#include "ExtremePhysics.h"

#include "ep_Force_IntegrateVelocity.h"
#include "ep_HingeJoint_IntegrateVelocity.h"
#include "ep_DistanceJoint_IntegrateVelocity.h"
#include "ep_RailJoint_IntegrateVelocity.h"
#include "ep_SliderJoint_IntegrateVelocity.h"
//JOINTS//

void ep_World::IntegrateVelocity() {
	
	for(ep_Body *body = first_body; body!=NULL; body = body->next) {
		if(body->issleeping) continue;
		
		body->xvel *= body->damping_factor;
		body->yvel *= body->damping_factor;
		body->rotvel *= body->rotdamping_factor;
		body->xvel += body->gravity_x*timestep;
		body->yvel += body->gravity_y*timestep;
		
		for(ep_Force *force = body->first_force; force!=NULL; force = force->next) {
			force->IntegrateVelocity();
		}
	}
	
	for(ep_HingeJoint *hingejoint = first_hingejoint; hingejoint!=NULL; hingejoint = hingejoint->next) {
		if(hingejoint->body1->issleeping || hingejoint->body2->issleeping) continue;
		hingejoint->IntegrateVelocity();
	}
	for(ep_DistanceJoint *distancejoint = first_distancejoint; distancejoint!=NULL; distancejoint = distancejoint->next) {
		if(distancejoint->body1->issleeping || distancejoint->body2->issleeping) continue;
		distancejoint->IntegrateVelocity();
	}
	for(ep_RailJoint *railjoint = first_railjoint; railjoint!=NULL; railjoint = railjoint->next) {
		if(railjoint->body1->issleeping || railjoint->body2->issleeping) continue;
		railjoint->IntegrateVelocity();
	}
	for(ep_SliderJoint *sliderjoint = first_sliderjoint; sliderjoint!=NULL; sliderjoint = sliderjoint->next) {
		if(sliderjoint->body1->issleeping || sliderjoint->body2->issleeping) continue;
		sliderjoint->IntegrateVelocity();
	}
	//JOINTS//
	
	SimulateWater();
	
}

