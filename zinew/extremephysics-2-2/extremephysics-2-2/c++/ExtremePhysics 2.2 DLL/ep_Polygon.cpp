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
 * File: ep_Polygon.cpp
 * Implementation of ep_Polygon.
 */

#include "ExtremePhysics.h"

void ep_Polygon::Init(ep_World* _world, unsigned long _vertexcount, unsigned long _id) {
	
	// add to linked list
	main = _world->main;
	world = _world;
	prev = world->last_polygon;
	next = NULL;
	if(prev==NULL) world->first_polygon = this;
	else prev->next = this;
	world->last_polygon = this;
	
	// initialize variables
	id = _id;
#if EP_USE_IDHASHTABLE
	world->idhashtable_polygons.Insert(this);
#endif
	
	vertexcount = _vertexcount;
	vertices = (ep_PolygonVertex*)(this+1);
	for(unsigned long i=0;i<vertexcount;++i) {
		vertices[i].x = 0.0;
		vertices[i].y = 0.0;
	}
	
#if EP_COMPILE_DEBUGCHECKS
	referencecount = 0;
	initialized = false;
#endif
	
	++world->polygoncount;
	
}

void ep_Polygon::DeInit() {
	
	// remove from linked list
	if(prev==NULL) world->first_polygon = next;
	else prev->next = next;
	if(next==NULL) world->last_polygon = prev;
	else next->prev = prev;
	if(world->current_polygon==this) world->current_polygon = NULL;
#if EP_USE_IDHASHTABLE
	world->idhashtable_polygons.Remove(this);
#endif
	
	--world->polygoncount;
	
}

// other

bool ep_Polygon::SetVertex(unsigned long index, double x, double y) {
	
#if EP_COMPILE_DEBUGCHECKS
	if(index>=vertexcount) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not set vertex %lu of polygon %lu in world %lu, the polygon has only %lu vertices.", index, id, world->id, vertexcount);
#endif
		return false;
	}
	if(initialized) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not set vertex %lu of polygon %lu in world %lu, the polygon has already been initialized.", index, id, world->id);
#endif
		return false;
	}
#endif
	
	vertices[index].x = x;
	vertices[index].y = y;
	
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_NORMALCHANGE, "Set vertex %lu of polygon %lu in world %lu.", index, id, world->id);
#endif
	
	return true;
}

bool ep_Polygon::Initialize() {
	
#if EP_COMPILE_DEBUGCHECKS
	if(initialized) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not initialize polygon %lu in world %lu, the polygon has already been initialized.", id, world->id);
#endif
		return false;
	}
#endif
	
	xoff = 0.0;
	yoff = 0.0;
	mass = 0.0;
	inertia = 0.0;
	
	for(unsigned long i = 0; i<vertexcount; ++i) {
		unsigned long j = (i+1)%vertexcount;
		vertices[i].len = sqrt(ep_sqr(vertices[i].x-vertices[j].x)+ep_sqr(vertices[i].y-vertices[j].y));
#if EP_COMPILE_DEBUGCHECKS
		if(vertices[i].len<EP_EPSILON_MINPOLYEDGELEN) {
#if EP_COMPILE_DEBUGMESSAGES
			main->Message(EP_MESSAGELEVEL_ERROR, "Could not initialize polygon %lu in world %lu, the length of edge %lu is zero.", id, world->id, i);
#endif
			return false;
		}
#endif
		vertices[i].nx = (vertices[j].y-vertices[i].y)/vertices[i].len;
		vertices[i].ny = (vertices[i].x-vertices[j].x)/vertices[i].len;
		vertices[i].dist = vertices[i].x*vertices[i].nx+vertices[i].y*vertices[i].ny;
		vertices[i].pos = vertices[i].x*vertices[i].ny-vertices[i].y*vertices[i].nx;
		double d = vertices[i].x*vertices[j].y-vertices[i].y*vertices[j].x;
		mass += d;
		xoff += d*(vertices[i].x+vertices[j].x)/3.0;
		yoff += d*(vertices[i].y+vertices[j].y)/3.0;
	}
	
#if EP_COMPILE_DEBUGCHECKS
	for(unsigned long i = 0; i<vertexcount; ++i) {
		unsigned long j = (i+1)%vertexcount;
		if((vertices[i].nx*vertices[j].ny-vertices[i].ny*vertices[j].nx)<EP_EPSILON_MINCROSSPRODUCT) { // /(vertices[i].len*vertices[j].len)
#if EP_COMPILE_DEBUGMESSAGES
			main->Message(EP_MESSAGELEVEL_ERROR, "Could not initialize polygon %lu in world %lu, the polygon is not convex at vertex %d.", id, world->id, j);
#endif
			return false;
		}
	}
	if(mass<EP_EPSILON_MINPOLYMASS) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not initialize polygon %lu in world %lu, the area of the polygon is zero.", id, world->id);
#endif
		return false;
	}
#endif
	
	xoff /= mass;
	yoff /= mass;
	mass *= 0.5;
	
	inertia = 0.0;
	for(unsigned long i = 0; i<vertexcount; ++i) {
		unsigned long j = (i+1)%vertexcount;
		double vx1 = vertices[i].x-xoff, vy1 = vertices[i].y-yoff;
		double vx2 = vertices[j].x-xoff, vy2 = vertices[j].y-yoff;
		inertia += (vx1*vy2-vy1*vx2)*(ep_sqr(vx1)+ep_sqr(vy1)+ep_sqr(vx2)+ep_sqr(vy2)+(vx1)*(vx2)+(vy1)*(vy2));
	}
	inertia /= 12.0;
	
#if EP_COMPILE_DEBUGCHECKS
	if(inertia<EP_EPSILON_MINPOLYINERTIA) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not initialize polygon %d in world %d, the moment of inertia of the polygon is zero.", id, world->id);
#endif
		return false;
	}
	initialized = true;
#endif
	
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_NORMALCHANGE, "Initialized polygon %lu in world %lu.", id, world->id);
#endif
	
	return true;
}

