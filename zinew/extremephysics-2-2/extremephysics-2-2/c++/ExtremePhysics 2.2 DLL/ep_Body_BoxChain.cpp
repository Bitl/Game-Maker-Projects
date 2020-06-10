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
 * File: ep_Body.cpp
 * Implementation of ep_Body.
 */

#include "ExtremePhysics.h"

#if EP_COMPILE_BOXCHAIN

bool ep_Body::BoxChainBegin(unsigned long vertexcount) {
#if EP_COMPILE_DEBUGCHECKS
	if(vertexcount<2) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not begin box chain for body %lu in world %lu, the number of vertices is smaller than 2.", id, world->id);
#endif
		return false;
	}
#endif
	ep_free(boxchain_vertices);
	boxchain_vertices = (ep_BoxChain_Vertex*)(ep_malloc(vertexcount*sizeof(ep_BoxChain_Vertex)));
	if(boxchain_vertices==NULL) {
		boxchain_vertexcount = 0;
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not begin box chain for body %lu in world %lu, memory allocation failed.", id, world->id);
#endif
		return false;
	}
	boxchain_vertexcount = vertexcount;
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_NORMALCHANGE, "Started box chain for body %lu in world %lu.", id, world->id);
#endif
	return true;
}

bool ep_Body::BoxChainEnd(bool circular, bool ignorefirstlast, double width_top, double width_bottom, double density) {
	
#if EP_COMPILE_DEBUGCHECKS
	if(boxchain_vertices==NULL) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not end box chain for body %lu in world %lu, the box chain was never started.", id, world->id);
#endif
		return false;
	}
#endif
	
	if(circular) ignorefirstlast = false;
	boxchain_first_shape = NULL;
	boxchain_last_shape = NULL;
	
	// normalize
	unsigned long c = (circular)? boxchain_vertexcount : boxchain_vertexcount-1;
	for(unsigned long i = 0; i<c; ++i) {
		unsigned long j = (i+1)%boxchain_vertexcount;
		double vx = boxchain_vertices[j].x-boxchain_vertices[i].x;
		double vy = boxchain_vertices[j].y-boxchain_vertices[i].y;
		double len = sqrt(ep_sqr(vx)+ep_sqr(vy));
#if EP_COMPILE_DEBUGCHECKS
		// is the length zero?
		if(len<EP_EPSILON_MINNORMALVECTOR) {
#if EP_COMPILE_DEBUGMESSAGES
			main->Message(EP_MESSAGELEVEL_ERROR, "Could not create box chain for body %lu in world %lu, the length of edge %lu is zero.", id, world->id, i);
#endif
			return false;
		}
#endif
		boxchain_vertices[i].len = len;
		boxchain_vertices[i].vx = vx/len;
		boxchain_vertices[i].vy = vy/len;
	}
	
	// calculate the tangents
	c = (circular)? boxchain_vertexcount : boxchain_vertexcount-2;
	for(unsigned long i = 0; i<c; ++i) {
		unsigned long j = (i+1)%boxchain_vertexcount;
		double dot = boxchain_vertices[i].vx*boxchain_vertices[j].vx+boxchain_vertices[i].vy*boxchain_vertices[j].vy;
		if(dot>1.0) dot = 1.0; // to fix rounding errors
		double t = (dot>0.0)? sqrt(1.0-dot*dot)/(1.0+dot) : 1.0; // = tan(acos(max(0.0, dot))*0.5)
		if(boxchain_vertices[i].vy*boxchain_vertices[j].vx<boxchain_vertices[i].vx*boxchain_vertices[j].vy) {
			boxchain_vertices[j].t = t*width_top;
		} else {
			boxchain_vertices[j].t = t*width_bottom;
		}
	}
	if(!circular && !ignorefirstlast) {
		boxchain_vertices[0].t = 0.0;
		boxchain_vertices[boxchain_vertexcount-1].t = 0.0;
	}
	
	// create the shapes
	c = (circular)? boxchain_vertexcount : ((ignorefirstlast)? boxchain_vertexcount-3 : boxchain_vertexcount-1);
	for(unsigned long i = (ignorefirstlast)? 1 : 0; i<c; ++i) {
		unsigned long j = (i+1)%boxchain_vertexcount;
		double w = boxchain_vertices[i].len+boxchain_vertices[i].t+boxchain_vertices[j].t;
		if(w>0.0) {
			double _rot = atan2(-boxchain_vertices[i].vy, boxchain_vertices[i].vx);
			double a = (boxchain_vertices[i].len+boxchain_vertices[j].t-boxchain_vertices[i].t)*0.5;
			double b = (width_top-width_bottom)*0.5;
			double xx = boxchain_vertices[i].x+boxchain_vertices[i].vx*a+boxchain_vertices[i].vy*b;
			double yy = boxchain_vertices[i].y+boxchain_vertices[i].vy*a-boxchain_vertices[i].vx*b;
			ep_Shape *s = CreateBoxShape(w, fabs(width_top+width_bottom), xx, yy, _rot, density);
			if(s==NULL) {
				for(; boxchain_first_shape!=NULL; boxchain_first_shape = s) {
					s = boxchain_first_shape->next;
					DestroyShape(boxchain_first_shape);
				}
				boxchain_last_shape = NULL;
				return false;
			}
			if(boxchain_first_shape==NULL) boxchain_first_shape = s;
			boxchain_last_shape = s;
		}
	}
	
	ep_free(boxchain_vertices);
	boxchain_vertexcount = 0;
	boxchain_vertices = NULL;
	
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_NORMALCHANGE, "Ended box chain for body %lu in world %lu.", id, world->id);
#endif
	
	return true;
}

bool ep_Body::BoxChainSetVertex(unsigned long index, double x, double y) {
#if EP_COMPILE_DEBUGCHECKS
	if(boxchain_vertices==NULL) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not set vertex %lu of box chain for body %lu in world %lu, the box chain was never started.", index, id, world->id);
#endif
		return false;
	}
	if(index>=boxchain_vertexcount) {
#if EP_COMPILE_DEBUGMESSAGES
		main->Message(EP_MESSAGELEVEL_ERROR, "Could not set vertex %lu of box chain for body %lu in world %lu, the box chain has only %lu vertices.", index, id, world->id, boxchain_vertexcount);
#endif
		return false;
	}
#endif
	boxchain_vertices[index].x = x;
	boxchain_vertices[index].y = y;
#if EP_COMPILE_DEBUGMESSAGES
	main->Message(EP_MESSAGELEVEL_NORMALCHANGE, "Set vertex %lu of box chain for body %lu in world %lu.", index, id, world->id);
#endif
	return true;
}

#endif // EP_COMPILE_BOXCHAIN

