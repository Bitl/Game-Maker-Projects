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
 * File: ep_View.cpp
 * Implementation of ep_View.
 */

#include "ExtremePhysics.h"

void ep_View::Init(ep_World* _world, unsigned long _id) {
	
	// add to linked list
	main = _world->main;
	world = _world;
	prev = world->last_view;
	next = NULL;
	if(prev==NULL) world->first_view = this;
	else prev->next = this;
	world->last_view = this;
	
	// initialize variables
	id = _id;
	
	x1 = 0.0;
	y1 = 0.0;
	x2 = 0.0;
	y2 = 0.0;
	
	++world->viewcount;
	
}

void ep_View::DeInit() {
	
	// remove from linked list
	if(prev==NULL) world->first_view = next;
	else prev->next = next;
	if(next==NULL) world->last_view = prev;
	else next->prev = prev;
	if(world->current_view==this) world->current_view = NULL;
	
	--world->viewcount;
	
}

void ep_View::SetRectangle(double _x1, double _y1, double _x2, double _y2) {
	x1 = _x1;
	y1 = _y1;
	x2 = _x2;
	y2 = _y2;
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_NORMALCHANGE, "Changed rectangle of view %lu in world %lu.", id, world->id);
#endif
}

