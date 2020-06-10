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
 * File: ep_World_SolvePositionConstraints.cpp
 * Solves position constraints for the simulation.
 */

#include "ExtremePhysics.h"

#include "ep_Contact_PositionConstraints.h"
#include "ep_HingeJoint_PositionConstraints.h"
#include "ep_DistanceJoint_PositionConstraints.h"
#include "ep_RailJoint_PositionConstraints.h"
#include "ep_SliderJoint_PositionConstraints.h"
//JOINTS//

void ep_World::SolvePositionConstraints() {
	
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
			contact->InitPositionConstraints();
			contact->nextconstraint = activecontacts;
			activecontacts = contact;
		}
		for(ep_HingeJoint *hingejoint = first_hingejoint; hingejoint!=NULL; hingejoint = hingejoint->next) {
			if(hingejoint->body1->issleeping || hingejoint->body2->issleeping) continue;
			hingejoint->InitPositionConstraints();
			hingejoint->nextconstraint = activehingejoints;
			activehingejoints = hingejoint;
		}
		for(ep_DistanceJoint *distancejoint = first_distancejoint; distancejoint!=NULL; distancejoint = distancejoint->next) {
			if(distancejoint->body1->issleeping || distancejoint->body2->issleeping) continue;
			distancejoint->InitPositionConstraints();
			if(distancejoint->minlimitforce==distancejoint->maxlimitforce) continue;
			distancejoint->nextconstraint = activedistancejoints;
			activedistancejoints = distancejoint;
		}
		for(ep_RailJoint *railjoint = first_railjoint; railjoint!=NULL; railjoint = railjoint->next) {
			if(railjoint->body1->issleeping || railjoint->body2->issleeping) continue;
			railjoint->InitPositionConstraints();
			railjoint->nextconstraint = activerailjoints;
			activerailjoints = railjoint;
		}
		for(ep_SliderJoint *sliderjoint = first_sliderjoint; sliderjoint!=NULL; sliderjoint = sliderjoint->next) {
			if(sliderjoint->body1->issleeping || sliderjoint->body2->issleeping) continue;
			sliderjoint->InitPositionConstraints();
			sliderjoint->nextconstraint = activesliderjoints;
			activesliderjoints = sliderjoint;
		}
		//JOINTS//
	}
	
	// solve constraints
	{
		for(unsigned long it = 0; it<position_iterations; ++it) {
			
			// solve constraints
			for(ep_Contact *contact = activecontacts; contact!=NULL; contact = contact->nextconstraint) {
				contact->SolvePositionConstraints();
			}
			for(ep_HingeJoint *hingejoint = activehingejoints; hingejoint!=NULL; hingejoint = hingejoint->nextconstraint) {
				hingejoint->SolvePositionConstraints();
			}
			for(ep_DistanceJoint *distancejoint = activedistancejoints; distancejoint!=NULL; distancejoint = distancejoint->nextconstraint) {
				distancejoint->SolvePositionConstraints();
			}
			for(ep_RailJoint *railjoint = activerailjoints; railjoint!=NULL; railjoint = railjoint->nextconstraint) {
				railjoint->SolvePositionConstraints();
			}
			for(ep_SliderJoint *sliderjoint = activesliderjoints; sliderjoint!=NULL; sliderjoint = sliderjoint->nextconstraint) {
				sliderjoint->SolvePositionConstraints();
			}
			//JOINTS//
			
		}
	}
	
}

