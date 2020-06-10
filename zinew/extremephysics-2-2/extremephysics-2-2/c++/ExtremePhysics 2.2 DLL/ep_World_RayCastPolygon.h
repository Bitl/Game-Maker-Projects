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
 * File: ep_World_RayCastPolygon.h
 * Calculates collisions between rays and polygon shapes.
 * Included in ep_World_RayCast.cpp.
 */

#ifndef EP_WORLD_RAYCASTPOLYGON_H
#define EP_WORLD_RAYCASTPOLYGON_H

#include "ExtremePhysics.h"

EP_FORCEINLINE double ep_World::RayCastPolygon(ep_Shape* shape, double x, double y, double vx, double vy) {
	
	// copy some variables for quick access
	ep_PolygonVertex *vertices = shape->shapedata.polygon.polygon->vertices;
	unsigned long vc = shape->shapedata.polygon.polygon->vertexcount;
	
	double p = (shape->x_t-x)*vx+(shape->y_t-y)*vy;
	double q = -(shape->x_t-x)*vy+(shape->y_t-y)*vx;
	
	// create transformation: x = p, y = q
	double transform_sin = ep_totalinvtransform_sin(shape->sin_t, shape->cos_t, -vy, vx);
	double transform_cos = ep_totalinvtransform_cos(shape->sin_t, shape->cos_t, -vy, vx);
	
	// find intersections
	unsigned long j = vc-1;
	double prev_q = q+ep_transform_y(transform_sin, transform_cos, vertices[j].x, vertices[j].y);
	bool intersects = false;
	for(unsigned long i = 0; i<vc; ++i) {
		double vertex_q = q+ep_transform_y(transform_sin, transform_cos, vertices[i].x, vertices[i].y);
		if(vertex_q<=0.0 && prev_q>=0.0) {
			// the ray enters the polygon
			double vertex_p = p+ep_transform_x(transform_sin, transform_cos, vertices[i].x, vertices[i].y);
			double prev_p = p+ep_transform_x(transform_sin, transform_cos, vertices[j].x, vertices[j].y);
			double d = prev_q-vertex_q, intersectionpos;
			if(d<EP_EPSILON_MINCONTACTPOINTS) {
				intersectionpos = ep_min(vertex_p, prev_p);
			} else {
				intersectionpos = prev_p+(vertex_p-prev_p)*prev_q/d;
			}
			if(intersectionpos>=0.0) {
				// the ray hits the polygon
				return intersectionpos;
			}
			if(intersects) return 0.0; // inside the polygon
			intersects = true;
		}
		if(vertex_q>=0.0 && prev_q<=0.0) {
			// the ray leaves the polygon
			double vertex_p = p+ep_transform_x(transform_sin, transform_cos, vertices[i].x, vertices[i].y);
			double prev_p = p+ep_transform_x(transform_sin, transform_cos, vertices[j].x, vertices[j].y);
			double d = vertex_q-prev_q, intersectionpos;
			if(d<EP_EPSILON_MINCONTACTPOINTS) {
				intersectionpos = ep_max(vertex_p, prev_p);
			} else {
				intersectionpos = vertex_p+(prev_p-vertex_p)*vertex_q/d;
			}
			if(intersectionpos<0.0) {
				// the ray can't hit the polygon
				return -1.0;
			}
			if(intersects) return 0.0; // inside the polygon
			intersects = true;
		}
		j = i;
		prev_q = vertex_q;
	}
	
	return -1.0;
	
}

#endif // EP_WORLD_RAYCASTPOLYGON_H

