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

void ep_Water::Init(ep_World* _world, unsigned long _id, bool awake) {
	
	// add to linked list
	main = _world->main;
	world = _world;
	prev = world->last_water;
	next = NULL;
	if(prev==NULL) world->first_water = this;
	else prev->next = this;
	world->last_water = this;
	
	// initialize variables
	id = _id;
	
	density = 0.0;
	lineardrag = 0.0;
	quadraticdrag = 0.0;
	xvel = 0.0;
	yvel = 0.0;
	gravity_x = 0.0;
	gravity_y = 0.0;
	x1 = 0.0;
	y1 = 0.0;
	x2 = 0.0;
	y2 = 0.0;
	
	// awake all bodies
	if(awake) {
		world->Awake();
	}
	
	++world->watercount;
	
}

void ep_Water::DeInit() {
	
	// remove from linked list
	if(prev==NULL) world->first_water = next;
	else prev->next = next;
	if(next==NULL) world->last_water = prev;
	else next->prev = prev;
	if(world->current_water==this) world->current_water = NULL;
	
	// awake all bodies
	world->Awake();
	
	--world->watercount;
	
}

void ep_Water::SetParameters(double _density, double _lineardrag, double _quadraticdrag, double _xvel, double _yvel, double _gravity_x, double _gravity_y) {
	if(_density<0.0) _density = 0.0;
	if(_lineardrag<0.0) _lineardrag = 0.0;
	if(_quadraticdrag<0.0) _quadraticdrag = 0.0;
	if(density!=_density || lineardrag!=_lineardrag || quadraticdrag!=_quadraticdrag || xvel!=_xvel || yvel!=_yvel || gravity_x!=_gravity_x || gravity_y!=_gravity_y) {
		density = _density;
		lineardrag = _lineardrag;
		quadraticdrag = _quadraticdrag;
		xvel = _xvel;
		yvel = _yvel;
		gravity_x = _gravity_x;
		gravity_y = _gravity_y;
		world->Awake();
	}
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_NORMALCHANGE, "Changed parameters of water %lu in world %lu.", id, world->id);
#endif
}

void ep_Water::SetRectangle(double _x1, double _y1, double _x2, double _y2) {
	if(x1!=_x1 || y1!=_y1 || x2!=_x2 || y2!=_y2) {
		x1 = _x1;
		y1 = _y1;
		x2 = _x2;
		y2 = _y2;
		world->Awake();
	}
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_NORMALCHANGE, "Changed rectangle of water %lu in world %lu.", id, world->id);
#endif
}

