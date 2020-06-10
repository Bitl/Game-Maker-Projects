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
 * File: ep_World_UpdateSleeping.cpp
 * Updates the sleeping system.
 */

#include "ExtremePhysics.h"

void ep_World::_UpdateSleeping() {
	
	// update timers
	for(ep_Body *body = first_body; body!=NULL; body = body->next) {
		if(!body->isstatic) {
			if(body->sleepstable && time_stable!=0.0) {
				if(!body->issleeping) body->stabletimer += timestep;
				if(ep_sqr(body->xvel)+ep_sqr(body->yvel)>stable_maxvel || fabs(body->rotvel)>fabs(stable_maxrotvel)) {
					body->stabletimer = 0.0;
				}
			}
			if(body->sleepoutofview && time_outofview!=0.0) {
				if(!body->issleeping) body->outofviewtimer += timestep;
				for(ep_Shape *shape = body->first_shape; shape!=NULL; shape = shape->next) {
					if(shape->updateaabb) {
						shape->UpdateTransformedCoordinates();
						shape->UpdateAABB();
					}
					ep_View *view;
					for(view = first_view; view!=NULL; view = view->next) {
						if(!(shape->aabb_x1>view->x2 || view->x1>shape->aabb_x2 ||
						     shape->aabb_y1>view->y2 || view->y1>shape->aabb_y2)) {
							body->outofviewtimer = 0.0;
							break;
						}
					}
					if(view!=NULL) break; // the body is in a view
				}
			}
		}
	}
	
	// awake bodies
	for(ep_Body *body = first_body; body!=NULL; body = body->next) {
		if(body->issleeping && 
		  (body->stabletimer<time_stable || time_stable==0.0 || !body->sleepstable) &&
		  (body->outofviewtimer<time_outofview || time_outofview==0.0 || !body->sleepoutofview)) {
			// awake the island
			body->issleeping = false;
			for(ep_Body *body2 = body->nextislandbody; body2!=body && body2!=NULL; body2 = body2->nextislandbody) {
				body2->issleeping = false;
			}
		}
	}
	
	// clear nextislandbody pointers
	// this is done separately because 'issleeping' can change
	for(ep_Body *body = first_body; body!=NULL; body = body->next) {
		if(!body->issleeping) {
			body->nextislandbody = NULL;
		}
	}
	
	// create islands
	for(ep_Body *body = first_body; body!=NULL; body = body->next) {
		if(!body->isstatic && body->nextislandbody==NULL) {
			// if nextislandbody!=NULL, the object is sleeping
			// we can assume the old islands are not connected to the new islands
			// because linkage changes should awake islands
			
			bool sleep = true;
			
			// flood-fill algorithm for connected bodies
			// This loop will add bodies to the island and read
			// bodies from the island at the same time.
			// It's like a queue, but the linked list that stores
			// the island IS also the queue at the same time.
			ep_Body *last = body;
			for(ep_Body *body2 = body; body2!=NULL; body2 = body2->nextislandbody) {
				
				// should this body be awake?
				if((body2->stabletimer<time_stable || time_stable==0.0 || !body2->sleepstable) &&
				   (body2->outofviewtimer<time_outofview || time_outofview==0.0 || !body2->sleepoutofview)) {
					sleep = false;
					// This island is not sleeping, but continuing now
					// will save processing time in the future.
				}
				
				// find connected bodies
				for(ep_Shape *shape = body2->first_shape; shape!=NULL; shape = shape->next) {
					for(ep_ContactLink *cl = shape->first_contactlink; cl!=NULL; cl = cl->next) {
						if(!cl->other_shape->body->isstatic && cl->other_shape->body->nextislandbody==NULL && cl->other_shape->body!=last) {
							last->nextislandbody = cl->other_shape->body;
							last = last->nextislandbody;
						}
					}
				}
				for(ep_HingeJointLink *jl = body2->first_hingejointlink; jl!=NULL; jl = jl->next) {
					if(!jl->other_body->isstatic && jl->other_body->nextislandbody==NULL && jl->other_body!=last) {
						last->nextislandbody = jl->other_body;
						last = last->nextislandbody;
					}
				}
				for(ep_DistanceJointLink *jl = body2->first_distancejointlink; jl!=NULL; jl = jl->next) {
					if(!jl->other_body->isstatic && jl->other_body->nextislandbody==NULL && jl->other_body!=last) {
						last->nextislandbody = jl->other_body;
						last = last->nextislandbody;
					}
				}
				for(ep_RailJointLink *jl = body2->first_railjointlink; jl!=NULL; jl = jl->next) {
					if(!jl->other_body->isstatic && jl->other_body->nextislandbody==NULL && jl->other_body!=last) {
						last->nextislandbody = jl->other_body;
						last = last->nextislandbody;
					}
				}
				for(ep_SliderJointLink *jl = body2->first_sliderjointlink; jl!=NULL; jl = jl->next) {
					if(!jl->other_body->isstatic && jl->other_body->nextislandbody==NULL && jl->other_body!=last) {
						last->nextislandbody = jl->other_body;
						last = last->nextislandbody;
					}
				}
				//JOINTS//
				
			}
			
			if(sleep) {
				// put the island to sleep
				for(ep_Body *body2 = body; body2!=NULL; body2 = body2->nextislandbody) {
					body2->issleeping = true;
				}
			}
			
			// close the loop
			last->nextislandbody = body;
			
		}
	}
	
}

