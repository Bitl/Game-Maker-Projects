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
 * File: ep_World_RayCastBox.h
 * Calculates collisions between rays and box shapes.
 * Included in ep_World_RayCast.cpp.
 */

#ifndef EP_WORLD_RAYCASTBOX_H
#define EP_WORLD_RAYCASTBOX_H

#include "ExtremePhysics.h"

EP_FORCEINLINE double ep_World::RayCastBox(ep_Shape* shape, double x, double y, double vx, double vy) {
	
	// copy some variables for quick access
	double w = shape->shapedata.box.w;
	double h = shape->shapedata.box.h;
	double p = (shape->x_t-x)*vx+(shape->y_t-y)*vy;
	double q = -(shape->x_t-x)*vy+(shape->y_t-y)*vx;
	
	// create transformation: x = p, y = q
	double transform_sin = ep_totalinvtransform_sin(shape->sin_t, shape->cos_t, -vy, vx);
	double transform_cos = ep_totalinvtransform_cos(shape->sin_t, shape->cos_t, -vy, vx);
	
	// transform coordinates
	double vertex_p1 = -transform_cos*((transform_sin>0.0)? w : -w)+transform_sin*((transform_cos>0.0)? h : -h);
	double vertex_q1 = fabs(transform_sin)*w+fabs(transform_cos)*h;
	double vertex_p2 = fabs(transform_cos)*w+fabs(transform_sin)*h;
	double vertex_q2 = -transform_sin*((transform_cos>0.0)? w : -w)+transform_cos*((transform_sin>0.0)? h : -h);
	if(q+vertex_q1<0.0 || q-vertex_q1>0.0) return -1.0;
	
	// find intersections
	double intersectionpos;
	if(q-vertex_q2<0.0) {
		double d = vertex_q1+vertex_q2;
		if(d<EP_EPSILON_MINCONTACTPOINTS) {
			intersectionpos = p+ep_min(vertex_p1, -vertex_p2);
		} else {
			intersectionpos = p-vertex_p2-(vertex_p1+vertex_p2)*(q-vertex_q2)/d;
		}
	} else {
		double d = -vertex_q2+vertex_q1;
		if(d<EP_EPSILON_MINCONTACTPOINTS) {
			intersectionpos = p+ep_min(-vertex_p2, -vertex_p1);
		} else {
			intersectionpos = p-vertex_p1-(-vertex_p2+vertex_p1)*(q-vertex_q1)/d;
		}
	}
	if(intersectionpos>=0.0) {
		// the ray hits the box
		return intersectionpos;
	}
	if(q+vertex_q2<0.0) {
		double d = vertex_q1-vertex_q2;
		if(d<EP_EPSILON_MINCONTACTPOINTS) {
			intersectionpos = p+ep_max(vertex_p1, vertex_p2);
		} else {
			intersectionpos = p+vertex_p2-(vertex_p1-vertex_p2)*(q+vertex_q2)/d;
		}
	} else {
		double d = vertex_q2+vertex_q1;
		if(d<EP_EPSILON_MINCONTACTPOINTS) {
			intersectionpos = p+ep_max(vertex_p2, -vertex_p1);
		} else {
			intersectionpos = p-vertex_p1-(vertex_p2+vertex_p1)*(q-vertex_q1)/d;
		}
	}
	if(intersectionpos<0.0) {
		// the ray can't hit the box
		return -1.0;
	}
	
	return 0.0;
	
}

#endif // EP_WORLD_RAYCASTBOX_H

