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
 * File: ep_Force.cpp
 * Implementation of ep_Force.
 */

#include "ExtremePhysics.h"

void ep_Force::Init(ep_Body* _body, double _x, double _y, bool _local, bool _ignoremass, unsigned long _id) {
	
	// add to linked list
	main = _body->main;
	world = _body->world;
	body = _body;
	prev = body->last_force;
	next = NULL;
	if(prev==NULL) body->first_force = this;
	else prev->next = this;
	body->last_force = this;
	
	// initialize variables
	id = _id;
	
	x = _x;
	y = _y;
	xforce = 0.0;
	yforce = 0.0;
	torque = 0.0;
	local = _local;
	ignoremass = _ignoremass;
	
	body->Awake(false, ignoremass);
	
	++world->forcecount;
	++body->forcecount;
	
}

void ep_Force::DeInit() {
	
	// remove from linked list
	if(prev==NULL) body->first_force = next;
	else prev->next = next;
	if(next==NULL) body->last_force = prev;
	else next->prev = prev;
	if(body->current_force==this) body->current_force = NULL;
	
	body->Awake(false, ignoremass);
	
	--world->forcecount;
	--body->forcecount;
	
}

