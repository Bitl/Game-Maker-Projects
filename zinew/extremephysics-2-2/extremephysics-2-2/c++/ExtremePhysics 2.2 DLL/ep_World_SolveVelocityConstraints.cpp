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
 * File: ep_World_SolveVelocityConstraints.cpp
 * Solves velocity constraints for the simulation.
 */

#include "ExtremePhysics.h"

#include "ep_Contact_VelocityConstraints.h"
#include "ep_HingeJoint_VelocityConstraints.h"
#include "ep_DistanceJoint_VelocityConstraints.h"
#include "ep_RailJoint_VelocityConstraints.h"
#include "ep_SliderJoint_VelocityConstraints.h"
//JOINTS//

void ep_World::SolveVelocityConstraints() {
	
	ep_Contact *activecontacts = NULL;
	ep_HingeJoint *activehingejoints = NULL;
	ep_DistanceJoint *activedistancejoints = NULL;
	ep_RailJoint *activerailjoints = NULL;
	ep_SliderJoint *activesliderjoints = NULL;
	//JOINTS//
	
	// create active constraint lists and initialize constraints
	// The lists are in reverse order because old constraints should be simulated last.
	{
		for(ep_Contact *contact = first_contact; contact!=NULL; contact = contact->next) {
			if(contact->body1->issleeping || contact->body2->issleeping) continue;
			contact->InitVelocityConstraints();
			contact->nextconstraint = activecontacts;
			activecontacts = contact;
		}
		for(ep_HingeJoint *hingejoint = first_hingejoint; hingejoint!=NULL; hingejoint = hingejoint->next) {
			if(hingejoint->body1->issleeping || hingejoint->body2->issleeping) continue;
			hingejoint->InitVelocityConstraints();
			hingejoint->nextconstraint = activehingejoints;
			activehingejoints = hingejoint;
		}
		for(ep_DistanceJoint *distancejoint = first_distancejoint; distancejoint!=NULL; distancejoint = distancejoint->next) {
			if(distancejoint->body1->issleeping || distancejoint->body2->issleeping) continue;
			distancejoint->InitVelocityConstraints();
			if(distancejoint->maxmotorforce==0.0 && distancejoint->minlimitforce==distancejoint->maxlimitforce) continue;
			distancejoint->nextconstraint = activedistancejoints;
			activedistancejoints = distancejoint;
		}
		for(ep_RailJoint *railjoint = first_railjoint; railjoint!=NULL; railjoint = railjoint->next) {
			if(railjoint->body1->issleeping || railjoint->body2->issleeping) continue;
			railjoint->InitVelocityConstraints();
			railjoint->nextconstraint = activerailjoints;
			activerailjoints = railjoint;
		}
		for(ep_SliderJoint *sliderjoint = first_sliderjoint; sliderjoint!=NULL; sliderjoint = sliderjoint->next) {
			if(sliderjoint->body1->issleeping || sliderjoint->body2->issleeping) continue;
			sliderjoint->InitVelocityConstraints();
			sliderjoint->nextconstraint = activesliderjoints;
			activesliderjoints = sliderjoint;
		}
		//JOINTS//
	}
	
	// apply impulses (this should be done after initializing)
	{
		for(ep_Contact *contact = activecontacts; contact!=NULL; contact = contact->nextconstraint) {
			contact->ApplyImpulses();
		}
		for(ep_HingeJoint *hingejoint = activehingejoints; hingejoint!=NULL; hingejoint = hingejoint->nextconstraint) {
			hingejoint->ApplyImpulses();
		}
		for(ep_DistanceJoint *distancejoint = activedistancejoints; distancejoint!=NULL; distancejoint = distancejoint->nextconstraint) {
			distancejoint->ApplyImpulses();
		}
		for(ep_RailJoint *railjoint = activerailjoints; railjoint!=NULL; railjoint = railjoint->nextconstraint) {
			railjoint->ApplyImpulses();
		}
		for(ep_SliderJoint *sliderjoint = activesliderjoints; sliderjoint!=NULL; sliderjoint = sliderjoint->nextconstraint) {
			sliderjoint->ApplyImpulses();
		}
		//JOINTS//
	}
	
	// solve constraints
	{
		for(unsigned long it = 0; it<velocity_iterations; ++it) {
			
			// solve constraints
			for(ep_Contact *contact = activecontacts; contact!=NULL; contact = contact->nextconstraint) {
				contact->SolveVelocityConstraints();
			}
			for(ep_HingeJoint *hingejoint = activehingejoints; hingejoint!=NULL; hingejoint = hingejoint->nextconstraint) {
				hingejoint->SolveVelocityConstraints();
			}
			for(ep_DistanceJoint *distancejoint = activedistancejoints; distancejoint!=NULL; distancejoint = distancejoint->nextconstraint) {
				distancejoint->SolveVelocityConstraints();
			}
			for(ep_RailJoint *railjoint = activerailjoints; railjoint!=NULL; railjoint = railjoint->nextconstraint) {
				railjoint->SolveVelocityConstraints();
			}
			for(ep_SliderJoint *sliderjoint = activesliderjoints; sliderjoint!=NULL; sliderjoint = sliderjoint->nextconstraint) {
				sliderjoint->SolveVelocityConstraints();
			}
			//JOINTS//
			
			// clamp velocity
			for(ep_Body *body = first_body; body!=NULL; body = body->next) {
				if(body->issleeping) continue;
				if(body->maxvel!=0.0) {
					double vel = ep_sqr(body->xvel)+ep_sqr(body->yvel);
					if(vel>ep_sqr(body->maxvel)) {
						vel = sqrt(vel);
						body->xvel *= body->maxvel/vel;
						body->yvel *= body->maxvel/vel;
					}
				}
				if(body->maxrotvel!=0.0) {
					body->rotvel = ep_clamp(-body->maxrotvel, +body->maxrotvel, body->rotvel);
				}
			}
			
		}
	}
}

